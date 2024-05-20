import rospy
import time
from osp import OSP
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, DisplayTrajectory
from std_msgs.msg import Float32
import threading


class OneROS:
    def __init__(self):
        self.orm = OSP("/dev/ttyUSB0")
        rospy.init_node("we_r2_control")
        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.joint_trajectory_callback)
        rospy.Subscriber('/gripper_state', Float32, self.gripper_state_callback)
        self.scheduled_trajectory = None
        self.STATUS_CHECK_TIMEOUT = 0.2  # Seconds # 5 Hz
        self.MAX_WAIT_ITERATION = 1 * 1  # Corresponds to 1 second
        self.TIME_SCALE_FACTOR = 1.0
        t = threading.Thread(target=self.trajectory_applying_thread, daemon=True)
        t.start()

    def apply_trajectory(self, points):
        print("Applying Trajectory: " + str(points))
        prev_time = 0.0
        prev_point = None
        for point in points:
            if prev_point is None:
                prev_point = point
                continue
            curr_time = point.time_from_start.secs + point.time_from_start.nsecs * 1e-9
            time_diff = curr_time - prev_time
            print("CURR TIME:", curr_time)
            print("TIME DIFF:", time_diff)
            time.sleep(time_diff * self.TIME_SCALE_FACTOR)
            for i in range(len(point.positions)):
                position = point.positions[i]
                self.orm.orm_set_angle(i, position)
                print("JOINT_" + str(i) + " POSITION = " + str(position))
            prev_time = curr_time
            prev_point = point

    def trajectory_applying_thread(self):
        while True:
            if self.scheduled_trajectory is not None:
                self.apply_trajectory(self.scheduled_trajectory.joi)
            time.sleep(0.1)

    def joint_trajectory_callback(self, msg):
        self.scheduled_trajectory = msg.trajectory[0].joint_trajectory

    def gripper_state_callback(self, msg):
        print("gripper angle:", msg.data)
        self.orm.orm_set_angle(6, msg.data)


if __name__ == '__main__':
    try:
        one_ros = OneROS()
        rospy.spin()
    except Exception as err:
        print(f"Unexpected {err=}, {type(err)=}")
