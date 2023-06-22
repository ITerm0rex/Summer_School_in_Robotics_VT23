#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import brickpi3


# Controller for handle EV3 Motor commands
class MotorController(Node):
    def __init__(self):
        super().__init__("lego_motor_controller")

        # Setup ROS subscriber
        self.left_subscription = self.create_subscription(
            Int32, "left_speed", self.speed_callback, 10
        )

        self.right_subscription = self.create_subscription(
            Int32, "right_speed", self.speed_callback, 10
        )

        # Declare port parameter
        self.declare_parameter("left_port", "A")
        self.declare_parameter("right_port", "C")
        self.declare_parameter("limit", "1000")


        # Init BrickPi3 and set up motor port
        self.brick = brickpi3.BrickPi3()  # Create an instance of the BrickPi3 class.

        self.limit = int(self.get_parameter("limit").value)
        
        left_port = self.get_parameter("left_port").value
        right_port = self.get_parameter("right_port").value

        self.left_port = self.brick["PORT_" + left_port]
        self.right_port = self.brick["PORT_" + right_port]

        self.get_logger().info(
            f"Motor ouput left_port: {left_port}, right_port: {right_port}"
        )
        # self.brick.set_motor_power(self.port, self.brick.MOTOR_FLOAT)  # float motor
        self.brick.reset_motor_encoder(self.left_port)
        self.brick.reset_motor_encoder(self.right_port)

        # Setup ROS publisher
        self.left_publisher = self.create_publisher(Int32, "left_encoder", 10)
        self.right_publisher = self.create_publisher(Int32, "right_encoder", 10)
        
        timer_period = 0.05  # seconds
        self.left_timer = self.create_timer(timer_period, self.encoder_callback(self.left_port, self.left_publisher))
        self.right_timer = self.create_timer(timer_period, self.encoder_callback(self.right_port, self.right_publisher))

    # Write and set motor speed
    def speed_callback(self, msg):
        def self_port(port):
            try:
                speed = msg.data
                speed = self.limit if speed > self.limit else speed
                speed = -self.limit if speed < -self.limit else speed
                self.brick.set_motor_dps(port, speed)
            except IOError as e:
                self.get_logger().error(f"Motor controller: {e}", throttle_duration_sec=1)
        return self_port

    # Read and publish motor encoder value
    def encoder_callback(self):
        def self_port(port, publisher):
            try:
                msg = Int32()
                msg.data = self.brick.get_motor_encoder(port)
                publisher.publish(msg)
            except IOError as e:
                self.get_logger().error(f"Motor controller: {e}", throttle_duration_sec=1)
        return self_port

    # Reset all motor ports
    def stop(self):
        try:
            self.brick.set_motor_dps(self.left_port, 0)
            self.brick.set_motor_dps(self.right_port, 0)
        except IOError as e:
            self.get_logger().error(f"Motor controller: {e}", throttle_duration_sec=1)
        finally:
            self.brick.reset_all()


# Main function
def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass

    # Stop the motor and destroy the node (explicitly)
    motor_controller.stop()
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
