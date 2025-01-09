import rospy 
import math
import time
from datetime import datetime
#from we_r2_robot import RobotWeR2
from osp import OSP
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

import threading
from tkinter.constants import CURRENT

JOINT_NAMES = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

actuators = ["a0", "a1", "a2", "a3", "a4", "a5"]

orm = OSP("/dev/ttyUSB0")

STATUS_CHECK_TIMEOUT = 0.2 # Seconds # 5 Hz
MAX_WAIT_ITERATION = 1*1  # Corresponds to 1 second

TIME_SCALE_FACTOR = 1.0

scheduled_trajectory = None

def apply_trajectory(joint_names, points):
    print("Applying Trajectory: "+str(points))
    prev_time = 0.0
    prev_point = None
    for point in points:
        if prev_point is None:
            prev_point = point
            continue
        # CHECK IF BUSY
        wait_counter = 0
        #while orm.orm_is_running() is True: 
        #    print("ORM IS RUNNING")
        #    time.sleep(STATUS_CHECK_TIMEOUT)
        #    wait_counter = wait_counter + 1
        #    if wait_counter > MAX_WAIT_ITERATION:
        #        print("ORM. Error. Timeout for waiting to achieve the position. Applying the next point")
        #        break
        
        curr_time = point.time_from_start.secs + point.time_from_start.nsecs * 1e-9;
        print("CURR TIME:" + str(curr_time))
        time_diff = curr_time - prev_time
        print("TIME DIFF:" + str(time_diff))
        time.sleep(time_diff * TIME_SCALE_FACTOR)
        for i in range(0,len(point.positions)):
            position = point.positions[i]
            #prev_position = prev_point.positions[i]
            #speed = (position - prev_position) / (time_diff * TIME_SCALE_FACTOR)
            #orm.orm_set_speed(i, abs(speed))
            orm.orm_set_angle(i, position)
            print("JOINT_"+str(i)+" POSITION = "+str(position))
        prev_time = curr_time 
        prev_point = point   
        
    #for i in range(0,len(prev_point.positions)): 
    #    orm.set_speed(i, 1024) # Angle correction speed  
      	    

def trajectory_applying_thread():
    global scheduled_trajectory
    current_trajectory = None
    while True:
        if scheduled_trajectory is not None:
            if current_trajectory!=scheduled_trajectory:
                current_trajectory = scheduled_trajectory
                apply_trajectory(current_trajectory.joint_names, current_trajectory.points)
        time.sleep(0.1)
        

def joint_trajectory_callback(msg):
    global scheduled_trajectory
    print(str(msg))
    scheduled_trajectory = msg.trajectory[0].joint_trajectory
    apply_trajectory(msg.trajectory[0].joint_trajectory.joint_names, msg.trajectory[0].joint_trajectory.points)
    #for joint_name in msg.name:
    #    if joint_name in JOINT_NAMES:
    #        # index of actuator
    #        i = JOINT_NAMES.index(joint_name)
    #        # index of joint position
    #        j = msg.name.index(joint_name)
    #        print("Joint Index: "+str(i))
    #        print("Joint State Index in Message: "+str(j))
    #        print("Joint "+str(i)+" Angle = "+str(msg.position[j])+"(rad)")
    #        angle_parts = ORM_MIDDLE_ANGLE + int(ORM_PARTS_IN_ANGLE * msg.position[j] / (math.pi * 2))
    #        print("Joint "+str(i)+" Angle = "+str(angle_parts))
    #        #orm.set_angle(i, angle_parts)
    #        #self.actuators[i].set_angle(self.jointsAng[j])

last_time = datetime.now()

def joint_states_callback(msg):
    global last_time
    current_time = datetime.now()
    delta = current_time - last_time 
    if delta.total_seconds() > 0.05:
        last_time = current_time
        #print("JointState: "+str(msg))
        for i in range(0,len(msg.position)):
            name = msg.name[i]
            joint_index = JOINT_NAMES.index(name)
            #orm.orm_set_angle(joint_index, msg.position[i])
    

def gripper_state_callback(msg):
    print("gripper angle:",msg.data)
    orm.orm_set_angle(6, msg.data)
         
rospy.init_node("we_r2_control")
#we_r2 = RobotWeR2()

#rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, joint_trajectory_callback)
rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, joint_trajectory_callback)
#rospy.Subscriber('/joint_states', JointState, joint_states_callback)
rospy.Subscriber('/gripper_state', Float32, gripper_state_callback)

t = threading.Thread(target=trajectory_applying_thread, daemon=True)
t.start()
rospy.spin()

   
        
