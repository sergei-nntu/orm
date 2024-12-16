#!/usr/bin/env python
import sys
import socket
import cv2
import rospy
import time
import moveit_commander
from cv_bridge import CvBridge
from moveit_msgs.msg import DisplayTrajectory, PlanningScene
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32, Bool
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler
from flask import Flask, request, Response
from sensor_msgs.msg import JointState, Image
import threading
import imp
import program
import textwrap
import math

program_thread = None

moveit_commander.roscpp_initialize(sys.argv)
group = MoveGroupCommander("arm")

group.set_max_velocity_scaling_factor(1)
group.set_max_acceleration_scaling_factor(1)

app = Flask(__name__)
bridge = CvBridge()

# Dynamic constants (global states)
pub_arm = None
pub_oqp = None
pub_grip = None

active_block_id = None

should_terminate_flag = False
blockly_running = False
connection = False

joint_trajectory = []
gripper_state = 0.0

# Static constants
JOINT_NAMES = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
JOINTS = {}

OQP_JOINT_NAMES = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8', 'joint9',
                   'joint10', 'joint11']
OQP_JOINTS = {}

current_pose_state = dict(x=0, y=0.1, z=0.4, pitch=0, roll=0, yaw=0)


def detect_tf(data):
    start = time.time()
    for m in data.transforms:
        id = m.fiducial_id
        trans = m.transform.translation
        rot = m.transform.rotation
        rospy.loginfo(rospy.get_caller_id() + "\n%s", trans)

        pose_goal = geometry_msgs.msg.Pose()

        scale = 0.3
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = round(trans.x / 3, 8)
        pose_goal.position.y = round(trans.z / 3, 8)
        pose_goal.position.z = abs(round(trans.y / 3, 8)) + scale

        group.set_pose_target(pose_goal)

        success = group.go(wait=True)
        print("Progress...", success)

        # group.clear_pose_targets()
        end = time.time()
        print("Time:", end - start)


def joint_states_callback(msg):
    for i in range(0, len(msg.position)):
        name = msg.name[i]
        position = msg.position[i]
        JOINTS[name] = position
        # joint_index = JOINT_NAMES.index(name)
        # print(joint_index, msg.position[i])


def usb_connection_callback(msg):
    global connection
    connection = msg.data


def oqp_joint_states_callback(msg):
    for i in range(0, len(msg.position)):
        name = msg.name[i]
        position = msg.position[i]
        OQP_JOINTS[name] = position


def gripper_state_callback(msg):
    global gripper_state
    gripper_state = msg.data


def joint_trajectory_callback(msg):
    global joint_trajectory, gripper_state
    joint_trajectory.clear()
    points = msg.trajectory[0].joint_trajectory.points
    for point in points:
        joints_state = list(point.positions)
        joints_state.append(gripper_state)
        joint_trajectory.append(joints_state)


def create_joint_state(name, position):
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()

    joint_state.name = name
    joint_state.position = position

    joint_state.velocity = []
    joint_state.effort = []
    return joint_state


def create_pose_message(x, y, z, pitch, roll, yaw):
    pose_msg = Pose()

    # Set the position
    pose_msg.position.x = x
    pose_msg.position.y = y
    pose_msg.position.z = z

    # Convert Euler angles to quaternion
    quaternion = quaternion_from_euler(roll, pitch, yaw)

    # Set the orientation
    pose_msg.orientation.x = quaternion[0]
    pose_msg.orientation.y = quaternion[1]
    pose_msg.orientation.z = quaternion[2]
    pose_msg.orientation.w = quaternion[3]

    return pose_msg


def set_pose(x, y, z, pitch, roll, yaw):
    pose_msg = create_pose_message(x, y, z, pitch, roll, yaw)
    group.set_pose_target(pose_msg)
    group.set_planning_time(0.2)

    success = group.go(wait=True)

    global current_pose_state
    if success:
        current_pose_state = {
            "x": x,
            "y": y,
            "z": z,
            "pitch": pitch,
            "roll": roll,
            "yaw": yaw
        }

    return {"execute": success, "data": current_pose_state}


def put_gripper_state(data):
    gripper_state_msg = Float32()
    gripper_state_msg.data = data

    # Publish Gripper Pose
    pub_grip.publish(gripper_state_msg)

    group.set_planning_time(0.1)
    success = group.go(wait=True)

    # That's no working correct
    # group.execute(plan, wait=True)
    return {"execute": success}


def publish_grip_state(state):
    pub_grip.publish(state)
    time.sleep(1)


def set_active_block(id):
    global active_block_id
    active_block_id = id


def orm_blockly_set_gripper_state(gripper_state):
    return round(gripper_state * math.pi / 180, 2)


def orm_blockly_set_position(x, y, z, pitch, roll, yaw):
    return set_pose(x, y, z, pitch, roll, yaw)


def orm_blockly_delay(ms):
    time.sleep(ms / 1000)


def should_terminate():
    return should_terminate_flag


@app.route("/get_pose_state", methods=["GET"])
def get_pose_state():
    global current_pose_state
    return {"data": current_pose_state}


@app.route("/convert_pose", methods=["POST"])
def convert_pose():
    data = request.json

    x = data["x"]
    y = data["y"]
    z = data["z"]
    pitch = data["pitch"]
    roll = data["roll"]
    yaw = data["yaw"]

    return set_pose(x, y, z, pitch, roll, yaw)


@app.route("/set_gripper_state", methods=["POST"])
def set_gripper_state():
    data = request.json
    gripper_state = data["gripper"]
    return put_gripper_state(gripper_state)


@app.route("/current_ip", methods=["GET"])
def get_current_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))

    return {"ip": s.getsockname()[0]}


@app.route("/start_program", methods=["GET"])
def start_program():
    global program_thread
    global should_terminate_flag
    global blockly_running

    blockly_running = True
    should_terminate_flag = False

    if program_thread is None or not program_thread.is_alive():
        imp.reload(program)
        program_thread = threading.Thread(target=program.program_main, args=(
            should_terminate, set_active_block, publish_grip_state, orm_blockly_delay, orm_blockly_set_position,
            orm_blockly_set_gripper_state))
        program_thread.start()
        return {"success": True}
    else:
        return {"success": False}


@app.route("/stop_program", methods=["GET"])
def stop_program():
    global should_terminate_flag, blockly_running
    if should_terminate_flag:
        return {"success": False}
    else:
        blockly_running = False
        should_terminate_flag = True
        return {"success": True}


@app.route("/get_active_program", methods=["GET"])
def get_active_program():
    structure = get_structure('structure.json')
    return {"structure": structure}


def insert_code(file_path, dynamic_code):
    try:
        code_with_indent = textwrap.indent(dynamic_code, '  ')
        code = f"def program_main(should_terminate_function, set_active_block_id, publish_grip_state, orm_blockly_delay, orm_blockly_set_position, orm_blockly_set_gripper_state):\n{code_with_indent}"

        with open(file_path, 'w') as file:
            file.write(code)

    except Exception as e:
        print(f"Error: {e}")


def save_structure(file_path, structure):
    with open(file_path, 'w') as file:
        file.write(structure)


def get_structure(file_path):
    structure = None
    with open(file_path, 'r') as file:
        structure = file.read(structure)

    return structure


def manipulator_image_callback(msg):
    global frame
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    _, frame = cv2.imencode('.jpg', cv_image)


def generate_image():
    while True:
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame.tobytes() + b'\r\n')


@app.route("/set_active_program", methods=["POST"])
def set_active_program():
    data = request.json
    source = data["source"]
    structure = data["structure"]

    insert_code('program.py', source)
    save_structure('structure.json', structure)
    return {"success": True}


@app.route("/get_program_state", methods=["GET"])
def get_program_state():
    global active_block_id
    return {"id": active_block_id}


@app.route("/get_joint_trajectory", methods=["GET"])
def get_joint_trajectory():
    global joint_trajectory
    return joint_trajectory


@app.route('/check_server_status', methods=["GET"])
def check_server_status():
    return "true", 200


@app.route('/get_usb_connection_status', methods=["GET"])
def get_usb_connection_status():
    global connection
    return {"connection": connection}


@app.route('/get_blockly_state', methods=["GET"])
def get_blockly_state():
    global blockly_running
    return {"state": blockly_running}


@app.route("/get_joints_state", methods=["GET"])
def get_joints_state():
    global JOINTS, JOINT_NAMES
    return {
        "shoulder": JOINTS[JOINT_NAMES[0]],
        "upperArm": JOINTS[JOINT_NAMES[1]],
        "forearm": JOINTS[JOINT_NAMES[2]],
        "wrist1": JOINTS[JOINT_NAMES[3]],
        "wrist2": JOINTS[JOINT_NAMES[4]],
        "endEffectorLink": JOINTS[JOINT_NAMES[5]]
    }


@app.get("/get_oqp_joint_state")
def get_oqp_joint_state():
    global OQP_JOINTS, OQP_JOINT_NAMES
    return {
        "shoulder1": OQP_JOINTS[OQP_JOINT_NAMES[0]],
        "reductor1": OQP_JOINTS[OQP_JOINT_NAMES[1]],
        "knee1": OQP_JOINTS[OQP_JOINT_NAMES[2]],
        "shoulder2": OQP_JOINTS[OQP_JOINT_NAMES[3]],
        "reductor2": OQP_JOINTS[OQP_JOINT_NAMES[4]],
        "knee2": OQP_JOINTS[OQP_JOINT_NAMES[5]],
        "shoulder3": OQP_JOINTS[OQP_JOINT_NAMES[6]],
        "reductor3": OQP_JOINTS[OQP_JOINT_NAMES[7]],
        "knee3": OQP_JOINTS[OQP_JOINT_NAMES[8]],
        "shoulder4": OQP_JOINTS[OQP_JOINT_NAMES[9]],
        "reductor4": OQP_JOINTS[OQP_JOINT_NAMES[10]],
        "knee4": OQP_JOINTS[OQP_JOINT_NAMES[11]],
    }


@app.post("/post_oqp_joint_state")
def post_dog_joints_state():
    global OQP_JOINT_NAMES, pub_oqp
    data = request.json

    shoulder1 = data["shoulder1"]
    reductor1 = data["reductor1"]
    knee1 = data["knee1"]
    shoulder2 = data["shoulder2"]
    reductor2 = data["reductor2"]
    knee2 = data["knee2"]
    shoulder3 = data["shoulder3"]
    reductor3 = data["reductor3"]
    knee3 = data["knee3"]
    shoulder4 = data["shoulder4"]
    reductor4 = data["reductor4"]
    knee4 = data["knee4"]

    position = [shoulder1, reductor1, knee1, shoulder2, reductor2, knee2, shoulder3, reductor3, knee3, shoulder4,
                reductor4, knee4]
    joint_state = create_joint_state(OQP_JOINT_NAMES, position)
    pub_oqp.publish(joint_state)

    return {"success": "true"}


@app.post("/post_joints_state")
def post_joints_state():
    data = request.json

    joint0 = data["joint0"]
    joint1 = data["joint1"]
    joint2 = data["joint2"]
    joint3 = data["joint3"]
    joint4 = data["joint4"]
    joint5 = data["joint5"]

    position = [joint0, joint1, joint2, joint3, joint4, joint5]
    joint_state = create_joint_state(JOINT_NAMES, position)
    pub_arm.publish(joint_state)

    return {"success": "true"}


@app.route("/manipulator_video_feed")
def manipulator_video_feed():
    return Response(generate_image(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    rospy.init_node('moveit_controller')

    global pub_arm
    global pub_oqp
    global pub_grip

    pub_arm = rospy.Publisher('/joint_states_manual', JointState, queue_size=10)
    pub_oqp = rospy.Publisher('/oqp_joint_states', JointState, queue_size=10)
    pub_grip = rospy.Publisher('/gripper_state', Float32, queue_size=10)

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    rospy.Subscriber('/gripper_state', Float32, gripper_state_callback)
    rospy.Subscriber('/oqp_joint_states', JointState, oqp_joint_states_callback)
    rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, joint_trajectory_callback)
    rospy.Subscriber('/usb_connection', Bool, usb_connection_callback)

    # manipulator camera image
    rospy.Subscriber('/camera/color/image_raw', Image, manipulator_image_callback)

    # rospy.spin()
    app.run(host="0.0.0.0", port=5001)


if __name__ == '__main__':
    try:
        main()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        print("Something went wrong!")
