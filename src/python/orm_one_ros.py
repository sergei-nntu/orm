import rospy
import math
import time
from datetime import datetime
# from we_r2_robot import RobotWeR2
from osp import OSP
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

import threading

JOINT_NAMES = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
STATUS_CHECK_TIMEOUT = 0.2  # Seconds # 5 Hz
MAX_WAIT_ITERATION = 1 * 1  # Corresponds to 1 second
TIME_SCALE_FACTOR = 1.0


class OneROS:
    def __init__(self, port):
        self.actuators = ["a0", "a1", "a2", "a3", "a4", "a5"]
        self.orm = OSP(port)
        self.scheduled_trajectory = None
        self.last_time = datetime.now()

        rospy.init_node("we_r2_control")
        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.joint_trajectory_callback)
        rospy.Subscriber('/gripper_state', Float32, self.gripper_state_callback)

        self.trajectory_thread = threading.Thread(target=self.trajectory_applying_thread, daemon=True)
        self.trajectory_thread.start()
        rospy.spin()

    def apply_trajectory(self, points):
        print("Applying Trajectory: " + str(points))
        prev_time = 0.0
        prev_point = None
        for point in points:
            if prev_point is None:
                prev_point = point
                continue

            # CHECK IF BUSY
            wait_counter = 0
            # while self.orm.orm_is_running() is True:
            #    print("ORM IS RUNNING")
            #    time.sleep(STATUS_CHECK_TIMEOUT)
            #    wait_counter += 1
            #    if wait_counter > MAX_WAIT_ITERATION:
            #        print("ORM. Error. Timeout for waiting to achieve the position. Applying the next point")
            #        break

            curr_time = point.time_from_start.secs + point.time_from_start.nsecs * 1e-9
            print("CURR TIME:" + str(curr_time))
            time_diff = curr_time - prev_time
            print("TIME DIFF:" + str(time_diff))
            time.sleep(time_diff * TIME_SCALE_FACTOR)
            for i in range(0, len(point.positions)):
                position = point.positions[i]
                # prev_position = prev_point.positions[i]
                # speed = (position - prev_position) / (time_diff * TIME_SCALE_FACTOR)
                # self.orm.orm_set_speed(i, abs(speed))
                self.orm.orm_set_angle(i, position)
                print("JOINT_" + str(i) + " POSITION = " + str(position))
            prev_time = curr_time
            prev_point = point

    def trajectory_applying_thread(self):
        current_trajectory = None
        while True:
            if self.scheduled_trajectory is not None:
                if current_trajectory != self.scheduled_trajectory:
                    current_trajectory = self.scheduled_trajectory
                    self.apply_trajectory(current_trajectory.joint_names, current_trajectory.points)
            time.sleep(0.1)

    def joint_trajectory_callback(self, msg):
        print(str(msg))
        self.scheduled_trajectory = msg.trajectory[0].joint_trajectory
        self.apply_trajectory(msg.trajectory[0].joint_trajectory.joint_names, msg.trajectory[0].joint_trajectory.points)

    def gripper_state_callback(self, msg):
        print("gripper angle:", msg.data)
        self.orm.orm_set_angle(6, msg.data)


if __name__ == '__main__':
    OneROS("/dev/ttyUSB0")
