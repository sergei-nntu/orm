import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from we_r2_arm_interfaces.action import MoveJointGroup


class MoveJointGroupClient(Node):
    def __init__(self):
        super().__init__('move_joint_group_client')
        self._action_client = ActionClient(self, MoveJointGroup, 'move_joint_group')

    def send_goal(self, joint_position_group, joints_position):
        goal_msg = MoveJointGroup.Goal()
        goal_msg.joints_position = joints_position

        self.get_logger().info(
            f'Sending goal:
            joints_position={joints_position},
            joint_position_group={joint_position_group}'
        )

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback:')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}, message={message}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    logger = get_logger("move_joint_group_client")

    node = None

    try:
        logger.info('Starting move joint group client...')

        node = MoveJointGroupClient()

        # FIXME: add valid args to the send_goal()
        # node.send_goal()

        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Application interrupted by user.")
    except Exception as e:
        logger.error('Move joint group client failed...', e)
    finally:
        if node is not None:
            node.destroy_node()


if __name__ == '__main__':
    main()
