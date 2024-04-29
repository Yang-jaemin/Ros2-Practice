import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int64
from rp_hw2_interfaces.srv import Multiply
from rp_hw2_interfaces.action import AddDigits
from rp_hw2_interfaces.msg import UniversityID

class Node2(Node):
    def __init__(self):
        super().__init__('node2')
        self.declare_parameter('multiplier_values', [4, 5])
        multiplier_values = self.get_parameter('multiplier_values').get_parameter_value().integer_array_value

        self.subscription = self.create_subscription(
            UniversityID,
            'university_id_topic',
            self.listener_callback,
            10)

        self.client = self.create_client(Multiply, 'multiply_two_ints')
        
        self.action_client = ActionClient(self, AddDigits, 'add_digits')
        
        self.send_request(multiplier_values[0], multiplier_values[1])
        self.send_goal()

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.uid}')

    def send_request(self, a, b):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for server multiply_two_ints...')
        req = Multiply.Request()
        req.a = a
        req.b = b
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response of multiplication: {response.result}')
        except Exception as e:
            self.get_logger().error(f'Service call failed %r' % (e,))

    def send_goal(self):
        goal_msg = AddDigits.Goal()
        goal_msg.uid = 2019741067
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action result: {result.final_result}')

def main(args=None):
    rclpy.init(args=args)
    node2 = Node2()
    rclpy.spin(node2)
    node2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()