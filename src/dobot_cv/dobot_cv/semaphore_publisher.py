import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SemaphorePublisher(Node):

    def __init__(self):
        super().__init__('dobot_cv')
        self.publisher = self.create_publisher(String, '/semaphore', 10)
        self.timer = self.create_timer(10.0, self.timer_callback)
        self.state = "red"

    def timer_callback(self):
        msg = String()
        msg.data = self.state
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    publisher = SemaphorePublisher()
    rclpy.spin(publisher)


if __name__ == '__main__':
    main()

