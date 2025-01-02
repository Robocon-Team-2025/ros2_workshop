import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.publisher import Publisher
from example_interfaces.srv import SetBool
from std_msgs.msg import String
from action_tutorials_interfaces.action import Fibonacci
import time

class FibonacciActionServiceTopic(Node):
    def __init__(self):
        super().__init__('fibonacci_action_service_topic')

        # Format state: True = Comma-separated string, False = List of numbers
        self.feedback_format = False

        # Action Server
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'calculate_fibonacci',
            self.execute_callback
        )

        # Service Server
        self.service = self.create_service(
            SetBool,
            'set_feedback_format',
            self.service_callback
        )

        # Topic Publisher
        self.publisher = self.create_publisher(String, 'fibonacci_feedback', 10)

        self.get_logger().info("Node is running...")

    def service_callback(self, request, response):
        self.feedback_format = request.data
        response.success = True
        response.message = 'Feedback format set to: ' + ('String' if self.feedback_format else 'List')
        self.get_logger().info(response.message)
        return response

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal: Calculate Fibonacci up to order {goal_handle.request.order}')

        sequence = [0, 1]
        for i in range(2, goal_handle.request.order):
            sequence.append(sequence[i - 1] + sequence[i - 2])
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.partial_sequence = sequence

            # Publish feedback to topic
            feedback_str = ', '.join(map(str, sequence)) if self.feedback_format else str(sequence)
            
            self.publisher.publish(String(data=f'Feedback: {feedback_str}'))

            # Send feedback to the action client
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Published feedback: {feedback_str}')

            time.sleep(1)  # Simulate computation time

        # Return the result
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info(f'Action complete: {sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServiceTopic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()