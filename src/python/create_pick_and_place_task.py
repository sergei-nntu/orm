import sys
import rospy
import moveit_commander
import moveit.task_constructor.core as core
import moveit.task_constructor.stages as stages
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface


def create_pick_and_place_task(start_pose, end_pose):
    task = core.Task()
    task.enableIntrospection()

    current_state = stages.CurrentState("current_state")
    task.add(current_state)

    planner = core.PipelinePlanner()

    move_to_pregrasp = stages.MoveTo("move_to_pregrasp", planner)
    move_to_pregrasp.group = "arm"
    move_to_pregrasp.setGoal(start_pose)
    task.add(move_to_pregrasp)

    open_hand = stages.MoveTo("open_hand", planner)
    open_hand.group = "hand"
    open_hand.setGoal("open")
    task.add(open_hand)

    move_to_grasp = stages.MoveTo("move_to_grasp", planner)
    move_to_grasp.group = "arm"
    move_to_grasp.setGoal(start_pose)
    task.add(move_to_grasp)

    close_hand = stages.MoveTo("close_hand", planner)
    close_hand.group = "hand"
    close_hand.setGoal("closed")
    task.add(close_hand)

    move_to_place = stages.MoveTo("move_to_place", planner)
    move_to_place.group = "arm"
    move_to_place.setGoal(end_pose)
    task.add(move_to_place)

    open_hand_place = stages.MoveTo("open_hand_place", planner)
    open_hand_place.group = "hand"
    open_hand_place.setGoal("open")
    task.add(open_hand_place)

    return task


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_and_place_task')

arm = "panda_arm"
eef = "hand"

object_name = "grasp_object"
object_radius = 0.02

psi = PlanningSceneInterface(synchronous=True)
psi.remove_world_object()

objectPose = PoseStamped()
objectPose.header.frame_id = "world"
objectPose.pose.orientation.x = 1.0
objectPose.pose.position.x = 0.30702
objectPose.pose.position.y = 0.0
objectPose.pose.position.z = 0.285

psi.add_box(object_name, objectPose, size=[0.1, 0.05, 0.03])

start_pose = PoseStamped()
start_pose.header.frame_id = "base_link"
start_pose.pose.position.x = 0.4
start_pose.pose.position.y = 0.1
start_pose.pose.position.z = 0.3
start_pose.pose.orientation.w = 1.0

end_pose = PoseStamped()
end_pose.header.frame_id = "base_link"
end_pose.pose.position.x = 0.4
end_pose.pose.position.y = -0.1
end_pose.pose.position.z = 0.3
end_pose.pose.orientation.w = 1.0

task = create_pick_and_place_task(start_pose, end_pose)

if task.plan():
    solutions = task.solutions
    if solutions:
        solution = solutions[0]
        task.execute(solution)

moveit_commander.roscpp_shutdown()
