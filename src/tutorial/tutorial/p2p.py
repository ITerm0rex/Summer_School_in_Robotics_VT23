import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class P2P(Node):
    def __init__(self):
        super().__init__("p2p")

        self.subscription = self.create_subscription(
            String,
            "hi",
            self.listener_callback,
            10,
        )

        self.publisher = self.create_publisher(
            String,
            "hi",
            10,
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"aaaa Hello World from Otto: {self.i}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.i += 1

    def listener_callback(self, msg):
        if int(msg.data) % 2 == 0:
            msg.data = "1"
            self.publisher.publish(msg)

        # if(str(msg.data).startswith("111")):
        self.get_logger().info(f"I heard: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    p2p = P2P()
    rclpy.spin(p2p)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    p2p.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
