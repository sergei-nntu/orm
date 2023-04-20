import rospy 
import math
import time
#from we_r2_robot import RobotWeR2
from osp import OSP
from moveit_msgs.msg import ExecuteTrajectoryActionGoal

JOINT_NAMES = ['joint0', 'joint1', 'joint2',
               'joint3', 'joint4', 'joint5']

actuators = ["a0", "a1", "a1", "a1", "a1", "a1"]

ORM_PARTS_IN_ANGLE = 16000
ORM_MIDDLE_ANGLE = ORM_PARTS_IN_ANGLE/2

orm = OSP("/dev/ttyACM1")

STATUS_CHECK_TIMEOUT = 0.05 # Seconds
MAX_WAIT_ITERATION = 20*1  # Corresponds to 1 second

def apply_trajectory(joint_names, points):
    print("Applying Trajectory: "+str(points))
    for point in points:
        # CHECK IF BUSY
        wait_counter = 0
        while orm.orm_is_running() is True: 
            print("ORM IS RUNNING")
            time.sleep(STATUS_CHECK_TIMEOUT)
            wait_counter = wait_counter + 1
            if wait_counter > MAX_WAIT_ITERATION:
                print("ORM. Error. Timeout for waiting to achieve the position. Applying the next point")
                break
        
        for i in range(0,len(point.positions)):
            position = point.positions[i]
            angle_parts = ORM_MIDDLE_ANGLE + int(ORM_PARTS_IN_ANGLE * position / (math.pi * 2))
            orm.set_angle(i, int(angle_parts))
            print("JOINT_"+str(i)+" POSITION = "+str(angle_parts))
      	    

def joint_states_callback(msg):
    print(str(msg))
    apply_trajectory(msg.goal.trajectory.joint_trajectory.joint_names, msg.goal.trajectory.joint_trajectory.points)
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

     
rospy.init_node("we_r2_control")
#we_r2 = RobotWeR2()

rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, joint_states_callback)
rospy.spin()

   
        
