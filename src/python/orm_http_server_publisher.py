#!/usr/bin/env python

# Import necessary libraries
import rospy
from sensor_msgs.msg import JointState

def publish_joint_states():
    # Initialize the ROS node
    rospy.init_node('joint_state_publisher', anonymous=True)

    # Create a publisher for JointState messages
    pub = rospy.Publisher('/oqp_joint_states', JointState, queue_size=10)

    # Create a new JointState message
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()

    # Define joint names (change these to your specific joint names)
    joint_state.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11']

    # Define joint positions (change these values as needed)
    joint_state.position = [0, -0.872665, 1.5708, 0, -0.872665, 1.5708, 0, -0.872665, 1.5708, 0, -0.872665, 1.5708]

    # Optionally define velocities and efforts
    joint_state.velocity = []
    joint_state.effort = []

    rate = rospy.Rate(10) # 10hz
    # Publish the JointState message
    while not rospy.is_shutdown():
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass