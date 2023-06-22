#!/usr/bin/env python3
import asyncio
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from rclpy.duration import Duration

import math


from brickpi3 import BrickPi3


class BMP(Node):
    def __init__(self):
        super().__init__("basic_movement_patterns")
        # self.logger = self.get_logger()

        self.reset_all = BrickPi3().reset_all

        self.left_publisher = self.create_publisher(Int32, "/rp5/left/speed", 10)
        self.right_publisher = self.create_publisher(Int32, "/rp5/right/speed", 10)
        # self.left_publisher = self.create_publisher(Int32, "speed", 10)
        # self.right_publisher = self.create_publisher(Int32, "speed", 10)

        # self.subscription = self.create_subscription(
        #     Int32, "encoder", self.encoder_callback, 10
        # )

        # self.subscription

        self.key_press_subscription = self.create_subscription(
            String, "press", self.key_press_callback, 10
        )

        self.key_release_subscription = self.create_subscription(
            String, "release", self.key_release_callback, 10
        )

        # self.color_subscription = self.create_subscription(String, "color", self.color_callback, 10)

        # Setup ROS publisher
        # self.publisher = self.create_publisher(Int32, "encoder", 10)
        # timer_period = 0.05  # seconds
        # self.timer = self.create_timer(timer_period, self.encoder_callback)

    def go_dps(self, left_dps: int = 0, right_dps: int = 0):
        left_msg = Int32()
        left_msg.data = int(left_dps)

        right_msg = Int32()
        right_msg.data = int(right_dps)

        self.left_publisher.publish(left_msg)
        self.right_publisher.publish(right_msg)

    def go_mps(self, left_mps: int = 0, right_mps: int = 0):
        WHEEL_DIAMETER = 5.5e-2
        self.go_dps(
            360 / (math.pi * WHEEL_DIAMETER) * left_mps,
            360 / (math.pi * WHEEL_DIAMETER) * right_mps,
        )

    dirMap = {
        "w": [360, 360],
        "d": [360, -360],
        "s": [-360, -360],
        "a": [-360, 360],
    }

    def key_to_motor(self, key):
        dir = self.dirMap.get(key, [0, 0])
        dir = [int(e * 1.2) for e in dir]
        self.go_dps(*dir)

    async def move_square(self, delay=2, mps=0.1):
        # self.go_mps(mps, mps)
        # self.get_clock().sleep_for(Duration(seconds=5))
        # self.go_mps(0, 0)

        # self.go_mps(-mps, -mps)
        # self.get_clock().sleep_for(Duration(seconds=5))
        # self.reset_all()

        # turn_timer.

        # while True:
        # for ls in [1, -1]:
        #     for rs in [1, -1]:
        #         if self.is_running:
        #             return
        #         self.go_mps(ls * mps, rs * mps)
        #         await asyncio.sleep(delay)

        while True:
            try:
                self.go_mps(mps, mps)
                await asyncio.sleep(5)
                self.go_mps(14.5e-2*math.pi*2/4, 0)
                await asyncio.sleep(1)
            except:
                break


        # self.go_mps(mps, mps)
        # # time.sleep(1)
        # await asyncio.sleep(delay)
        # self.go_mps(mps, -mps)
        # # time.sleep(1)
        # await asyncio.sleep(delay)
        # self.go_mps(-mps, mps)
        # # time.sleep(1)
        # await asyncio.sleep(delay)
        # self.go_mps(-mps, -mps)
        # # time.sleep(1)
        # await asyncio.sleep(delay)
        # self.reset_all()

    # async def run_move(self):
    #     self.move_square(1, 0.1)

    # loop = asyncio.get_event_loop()

    # def move_(self):

    def key_press_callback(self, msg):
        print("press:", msg)
        key = msg.data
        if key == "space":
            self.go_dps(0, 0)

        elif key == "1":
            asyncio.run(self.move_square())

            # and not self.task:
            # co = self.move_square(1, 0.1)
            # self.task = asyncio.create_task(co)

            # asyncio.Future(co)

            # self.task.result()

            # asyncio.to_thread(self.move_square, args=(1, 0.1))
            # self.task = self.loop.create_task(co)

            # tt = asyncio.to_thread(self.move_square, (1, 0.1))
            # tt = asyncio.to_thread(self.run_move)

            # # self.run_move()
            # co = self.run_move()

            # # self.task = self.loop.create_task(co)
            # self.loop.run_until_complete(co)
            # self.loop.run_forever()

            # asyncio.a

            # asyncio.run(co)

            # self.loop.call_soon_threadsafe()

            # self.task.get_loop().run_until_complete()
            # self.task.get_loop().create_task

            # asyncio.run(asyncio.to_thread(self.move_square, args=(1, 0.1)))
            pass
        elif key == "2":
            # if self.loop.is_running():
            #     self.loop.close()
            # if self.task:
            #     self.task.cancel()
            #     self.task = None

            pass
        else:
            self.key_to_motor(key)

    def key_release_callback(self, msg):
        print("release:", msg)

    # Write and set motor speed
    def speed_callback(self, msg):
        print("speed:", msg)

    # Read and publish motor encoder value
    def encoder_callback(self, msg):
        print("encoder:", msg)

    def stop(self):
        self.reset_all()


# Main function
def main(args=None):
    rclpy.init(args=args)

    try:
        bmp = BMP()
        try:
            rclpy.spin(bmp)
        finally:
            bmp.destroy_node()
            bmp.stop()
            # rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
