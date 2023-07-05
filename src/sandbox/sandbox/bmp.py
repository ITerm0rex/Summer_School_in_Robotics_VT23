#!/usr/bin/env python3
import asyncio
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32, String

from rclpy.duration import Duration

import math

from brickpi3 import BrickPi3


# WHEEL_DISTANCE = 14.5e-2
WHEEL_DISTANCE = 13.8e-2
WHEEL_DIAMETER = 5.5e-2


class bp3(BrickPi3):
    def reset_motors(self):
        # turn off all motors
        self.set_motor_power(
            self.PORT_A + self.PORT_B + self.PORT_C + self.PORT_D, self.MOTOR_FLOAT
        )
        # reset motor limits
        self.set_motor_limits(self.PORT_A + self.PORT_B + self.PORT_C + self.PORT_D)


class BMP(Node):
    def __init__(self):
        # super().__init__("basic_movement_patterns")
        super().__init__("b_m_p")

        self.bp3 = bp3()

        self.left_publisher = self.create_publisher(Int32, "left/speed", 10)
        self.right_publisher = self.create_publisher(Int32, "right/speed", 10)

        # self.left_encoder_subscription = self.create_subscription(
        #     Int32, "left/encoder", self.left_encoder_callback, 10
        # )
        # self.right_encoder_subscription = self.create_subscription(
        #     Int32, "right/encoder", self.right_encoder_callback, 10
        # )

        self.color_subscription = self.create_subscription(
            String, "color", self.color_callback, 10
        )

        self.ultrasonic_subscription = self.create_subscription(
            Float32, "ultrasonic", self.ultrasonic_callback, 10
        )

        self.gyro_subscription = self.create_subscription(
            Float32, "gyro", self.gyro_callback, 10
        )

        self.touch_subscription = self.create_subscription(
            Bool, "touch", self.touch_callback, 10
        )

        self.key_press_subscription = self.create_subscription(
            String, "press", self.key_press_callback, 10
        )

        # self.key_release_subscription = self.create_subscription(
        #     String, "release", self.key_release_callback, 10
        # )

    def go_right_dps(self, right_dps: int = 0):
        right_msg = Int32()
        right_msg.data = int(right_dps)
        self.right_publisher.publish(right_msg)

    def go_left_dps(self, left_dps: int = 0):
        left_msg = Int32()
        left_msg.data = int(left_dps)
        self.left_publisher.publish(left_msg)

    def go_dps(self, left_dps: int = 0, right_dps: int = 0):
        self.go_left_dps(left_dps)
        self.go_right_dps(right_dps)

    def go_mps(self, left_mps: int = 0, right_mps: int = 0):
        self.go_dps(
            360 / (math.pi * WHEEL_DIAMETER) * left_mps,
            360 / (math.pi * WHEEL_DIAMETER) * right_mps,
        )

    def key_to_motor(self, key: str):
        KEY_DIR_MAP = {
            "w": [360, 360],
            "a": [-360, 360],
            "s": [-360, -360],
            "d": [360, -360],
            "e": [360, 0],
            "q": [0, 360],
            "c": [-360, 0],
            "z": [0, -360],
        }

        dir = KEY_DIR_MAP.get(key, [0, 0])
        dir = [int(e * self.wasd_speed) for e in dir]
        self.go_dps(*dir)

    # 1
    async def move_square(self, delay=5, mps=0.1):
        self.is_running = True
        while self.is_running:
            self.go_mps(mps, mps)
            await asyncio.sleep(delay)
            # self.get_clock().sleep_for(Duration(seconds=5))
            self.go_mps(WHEEL_DISTANCE * math.pi * 2 / 4, 0)
            await asyncio.sleep(1)
            # self.get_clock().sleep_for(Duration(seconds=1))

    # 2
    async def move_circle(self, radius_in_meter=0.25, mps=0.1):
        self.go_mps(mps, mps / 2)

    # 3
    async def move_figure_eight(self, delay=2, mps=0.1):
        self.is_running = True
        while self.is_running:
            self.go_mps(mps, mps / 2)
            await asyncio.sleep(5)
            self.go_mps(mps / 2, mps)
            await asyncio.sleep(5)

    # 4
    async def move_line_following(self):
        tick = 0

        def color_event_callback(color):
            nonlocal tick

            if color == "white":
                if tick == 0:
                    self.go_dps(-100, 200)
                elif tick == 10:
                    self.go_dps(200, -100)
                elif tick == 100:
                    self.go_dps(-100, 200)
                elif tick == 700:
                    self.go_dps(200, 200)
                elif tick == 1000:
                    tick = 0
                tick += 1
            else:
                tick = 0
                self.go_dps(200, 200)

            # #!!!!
            # if color == "white":
            #     if tick == 0:
            #         self.go_dps(-100, 200)
            #     elif tick == 10:
            #         self.go_dps(200, -100)
            #     elif tick == 40:
            #         self.go_dps(-100, 200)
            #     tick += 1
            # else:
            #     tick = 0
            #     self.go_dps(100, 100)

            # if color == "white":
            #     if tick == 0:
            #         self.go_dps(-100, 200)
            #     elif tick == 5:
            #         self.go_dps(200, -100)
            #     elif tick == 40:
            #         self.go_dps(-100, 200)
            #     tick += 1
            # else:
            #     tick = 0
            #     self.go_dps(200, 200)

            # if color == "white":
            #     if tick == 0:
            #         self.go_dps(-100, 200)
            #     elif tick == 10:
            #         self.go_dps(200, -100)
            #     elif tick == 20:
            #         self.go_dps(-100, 200)
            #     elif tick == 40:
            #         self.go_dps(200, -100)
            #     elif tick == 40:
            #         self.go_dps(200, -100)
            #     tick += 1
            # else:
            #     tick = 0
            #     self.go_dps(300, 300)

        self.event_callback_set["color"] = color_event_callback

    event_callback_set = dict()
    is_running = True
    wasd_speed = 1.0

    def key_press_callback(self, msg: String):
        self.get_logger().info(f"key press: {msg.data}")
        key = msg.data
        if key == "1":
            asyncio.run(self.move_square())
            pass
        elif key == "2":
            asyncio.run(self.move_circle())
            pass
        elif key == "3":
            asyncio.run(self.move_figure_eight())
            pass
        elif key == "4":
            asyncio.run(self.move_line_following())
            pass
        elif key == "5":
            self.go_mps(0.1, 0.1)
            pass
        elif key == "6":
            self.event_callback_set["touch"] = (
                lambda touched: self.bp3.reset_all() if touched else None
            )
            pass
        elif key == "k":
            for s in [
                self.gyro_subscription,
                self.color_subscription,
                self.touch_subscription,
                self.ultrasonic_subscription,
            ]:
                s.destroy()
            self.bp3.reset_all()
            pass
        elif key == "7":
            self.bp3.reset_all()
            pass
        elif key == "8":
            self.wasd_speed = (int(self.wasd_speed * 10) - 2) / 10
            self.wasd_speed %= 2.2
            self.get_logger().info(f"wasd_speed: {self.wasd_speed}")
            pass
        elif key == "9":
            self.wasd_speed = (int(self.wasd_speed * 10) + 2) / 10
            self.wasd_speed %= 2.2
            self.get_logger().info(f"wasd_speed: {self.wasd_speed}")
            pass
        elif key == "0":
            self.is_running = False
            self.event_callback_set.clear()
            self.bp3.reset_motors()
            self.get_logger().info(f"is running: {self.is_running}")

        # elif key == "space":
        #     self.bp3.reset_motors()
        else:
            self.key_to_motor(key)

    def key_release_callback(self, msg: String):
        self.get_logger().info(f"key release: {msg.data}")

    # Read color module
    def color_callback(self, msg: String):
        if cb := self.event_callback_set.get("color", None):
            cb(msg.data)
        self.get_logger().info(f"color: {msg.data}", throttle_duration_sec=1)

    # Read ultrasonic module
    def ultrasonic_callback(self, msg: Float32):
        if cb := self.event_callback_set.get("ultrasonic", None):
            cb(msg.data)
        self.get_logger().info(f"ultrasonic: {msg.data}", throttle_duration_sec=1)

    # Read touch module
    def touch_callback(self, msg: Bool):
        if cb := self.event_callback_set.get("touch", None):
            cb(msg.data)
        self.get_logger().info(f"touch: {msg.data}", throttle_duration_sec=1)

    # Read gyro module
    def gyro_callback(self, msg: Float32):
        if cb := self.event_callback_set.get("gyro", None):
            cb(msg.data)
        self.get_logger().info(f"gyro: {msg.data}", throttle_duration_sec=1)

    # Read motor encoder value
    def left_encoder_callback(self, msg: Int32):
        self.get_logger().info(f"left encoder: {msg.data}", throttle_duration_sec=1)

    def right_encoder_callback(self, msg: Int32):
        self.get_logger().info(f"right encoder: {msg.data}", throttle_duration_sec=1)

    # Reset sensor and motor port
    def reset(self):
        self.bp3.reset_all()


# Main function
def main(args=None):
    rclpy.init(args=args)

    try:
        bmp = BMP()
        try:
            rclpy.spin(bmp)
        finally:
            bmp.destroy_node()
            bmp.reset()
            rclpy.try_shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
