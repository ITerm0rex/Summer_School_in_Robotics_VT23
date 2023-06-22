import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__("talker")  # 'talker' is name of node
        self.publisher_ = self.create_publisher(
            String,
            # "chatter",
            "hi",
            10,
        )
        # 10 is the queue of messages being kept
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"aaaa Hello World from Otto: {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher = Talker()
    rclpy.spin(publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
