import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Int64
from rp_hw2_interfaces.srv import Multiply
from rp_hw2_interfaces.action import AddDigits
from rp_hw2_interfaces.msg import UniversityID



class Node1(Node):
    def __init__(self):
        super().__init__('node1')

        self.declare_parameter('university_id', 2019741067)
        self.university_id = self.get_parameter('university_id').get_parameter_value().integer_value


        self.publisher_ = self.create_publisher(UniversityID, 'university_id_topic', 10)
        self.timer = self.create_timer(1, self.publish_university_id)

        self.service_ = self.create_service(Multiply, 'multiply_two_ints', self.multiply_two_ints_callback)
        
        self.action_server = ActionServer(self, AddDigits, 'add_digits', self.execute_callback)

    def publish_university_id(self):
        msg = UniversityID()
        msg.uid = self.university_id
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.uid}')

    def multiply_two_ints_callback(self, request, response):
        response.result = request.a * request.b
        self.get_logger().info(f'Multiplying: {request.a} * {request.b}')
        return response

    def execute_callback(self, goal_handle):
        uid_str = str(goal_handle.request.uid)
        result = 0
        feedback_msg = AddDigits.Feedback()

        for digit in uid_str:
            result += int(digit)
            feedback_msg.feedback = result
            self.get_logger().info(f'Sending Feedback: {feedback_msg.feedback}')
            goal_handle.publish_feedback(feedback_msg)
            
        goal_handle.succeed()
        result_msg = AddDigits.Result()
        result_msg.final_result = result
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    rclpy.spin(node1)
    node1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
