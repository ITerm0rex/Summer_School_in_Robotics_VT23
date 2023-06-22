#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Class for handle shell inputs
class Shell_Input(Node):
    import sshkeyboard

    def __init__(self):
        super().__init__("shell_input")

        # Setup ROS publisher
        self.press_publisher = self.create_publisher(String, "press", 10)
        self.release_publisher = self.create_publisher(String, "release", 10)

        self.sshkeyboard.listen_keyboard(
            on_press=self.press,
            on_release=self.release,
        )

    def press(self, key):
        msg = String()
        msg.data = key
        self.press_publisher.publish(msg)
        self.get_logger().info(f"'{key}' pressed")

    def release(self, key):
        msg = String()
        msg.data = key
        self.release_publisher.publish(msg)
        self.get_logger().info(f"'{key}' released")

    def stop(self):
        self.sshkeyboard.stop_listening()


# Main function
def main(args=None):
    rclpy.init(args=args)

    try:
        shell_input = Shell_Input()
        try:
            rclpy.spin(shell_input)
        finally:
            shell_input.stop()
            shell_input.destroy_node()
            rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
