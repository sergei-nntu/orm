#!/usr/bin/env python
import sys
import socket
import rospy
import time
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, PlanningScene
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
#from fiducial_msgs.msg import FiducialTransformArray
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler
from flask import Flask, request, jsonify
import threading
import imp
import program
import textwrap
import re

program_thread = None

moveit_commander.roscpp_initialize(sys.argv)
group = MoveGroupCommander("arm")

group.set_max_velocity_scaling_factor(1)
group.set_max_acceleration_scaling_factor(1)

app = Flask(__name__)

pub = None
active_block_id = None
should_terminate_flag = False

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
    return {"execute": success}

def set_gripper_state(data):
    gripper_state_msg = Float32()
    gripper_state_msg.data = data

    # Publish Gripper Pose
    pub.publish(gripper_state_msg)

    group.set_planning_time(0.1)
    plan = group.go(wait=True)
    
    group.execute(plan, wait=True)
    
    return {"execute": plan}

def set_active_block(id):
    global active_block_id
    active_block_id = id

def orm_blockly_set_gripper_state(gripper_state):
    return set_gripper_state(gripper_state)

def orm_blockly_set_position(x, y, z, pitch, roll, yaw):
    return set_pose(x, y, z, pitch, roll, yaw)

def orm_blockly_delay(ms):
    time.sleep(ms / 1000)

def should_terminate():
    return should_terminate_flag

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
def gripper_state():
    data = request.json
    gripper_state = data["gripper"]
    return set_gripper_state(gripper_state)

@app.route("/current_ip", methods=["GET"])
def get_current_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))

    return {"ip": s.getsockname()[0]}

@app.route("/start_program", methods=["GET"])
def start_program():
    global program_thread
    global should_terminate_flag

    should_terminate_flag = False
    if program_thread is None or not program_thread.is_alive():
        imp.reload(program)
        program_thread = threading.Thread(target=program.program_main, args=(should_terminate, set_active_block))
        program_thread.start()
        print("Program started!")
    else:
        print("Program is already running!")

@app.route("/stop_program", methods=["GET"])
def stop_program():
    global should_terminate_flag
    if should_terminate_flag:
        print("Program has been stopped!")
    else:
        should_terminate_flag = True
        print("Program stopped!")

@app.route("/get_active_program", methods=["GET"])
def get_active_program():
    pass

def insert_code(file_path, dynamic_code):
    try:
        code_with_indent = textwrap.indent(dynamic_code, '  ')
        code = f"from orm_http_server import *\ndef program_main(should_terminate_function, set_active_block_id):\n{code_with_indent}"
        
        with open(file_path, 'w') as file:
            file.write(code)

    except Exception as e:
        print(f"Error: {e}")

@app.route("/set_active_program", methods=["POST"])
def set_active_program():
    data = request.json
    source = data["source"]
    insert_code('program.py', source)

@app.route("/get_program_state", methods=["GET"])
def get_program_state():
    global active_block_id
    return {"id": active_block_id}

def main():
    rospy.init_node('moveit_controller')

    global pub

    pub = rospy.Publisher('/gripper_state', Float32, queue_size = 10)
    #rospy.spin()
    app.run(host="0.0.0.0", port=5000)

if __name__ == '__main__':
    try:
        main()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        print("Something went wrong!")
