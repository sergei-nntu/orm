#!/usr/bin/env python
import sys
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

moveit_commander.roscpp_initialize(sys.argv)
group = MoveGroupCommander("arm")

group.set_max_velocity_scaling_factor(1)
group.set_max_acceleration_scaling_factor(1)

app = Flask(__name__)

pub = None

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

@app.route("/convert_pose", methods=["POST"])
def convert_pose():
    data = request.json

    x = data["x"]
    y = data["y"]
    z = data["z"]
    pitch = data["pitch"]
    roll = data["roll"]
    yaw = data["yaw"]

    pose_msg = create_pose_message(x, y, z, pitch, roll, yaw)
    
    group.set_pose_target(pose_msg)
    

    group.set_planning_time(0.2)
    
    print("Planning...")
    plan = group.go(wait=True)
    #group.stop()
    #group.clear_pose_targets()
    print("Executing....")
    group.execute(plan, wait=True)
    
    # Publish Gripper Pose
    gripper_state_msg = Float32()
    gripper_state_msg.data = data["gripper"]
    pub.publish(gripper_state_msg)
    
    return {"execute": success}



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

