import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class DynamicNavigator(Node):
    def __init__(self):
        super().__init__('dynamic_navigator')
        
        # ActionClient for Nav2
        self.nav_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)

        self.current_goal = None

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f'Received Goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.send_goal_to_nav2(msg)

    def path_callback(self, msg: Path):
        # 장애물 등으로 Path가 바뀌면, 필요 시 goal 재전송 가능
        pass

    def send_goal_to_nav2(self, pose: PoseStamped):
        if not self.nav_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action server not available!')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info('Sending goal to Nav2...')
        self._send_goal_future = self.nav_action.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return
        self.get_logger().info('Goal accepted. Moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached or failed!')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
