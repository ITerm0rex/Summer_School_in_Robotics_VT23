import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.subscription = self.create_subscription(
            String,
            "by",
            self.listener_callback,
            10,
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # if(str(msg.data).startswith("111")):
        self.get_logger().info(f"I heard: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    subscriber = Listener()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
