#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Point, Pose2D

# import builtins

def Odometry_to_Pose2D(odom: Odometry):
    pose2d = Pose2D()

    q = odom.pose.pose.orientation

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

    pose2d.theta = np.arctan2(siny_cosp, cosy_cosp)

    if np.abs(q.x * q.y + q.z * q.w) == 0.5:
        pose2d.theta = 0.0

    pose2d.x = odom.pose.pose.position.x
    pose2d.y = odom.pose.pose.position.y

    return pose2d


class DifferentialDrive(Node):
    def __init__(self):
        super().__init__("cord_node")

        self.cmd_publisher = self.create_publisher(Twist, "cmd", 10)

        self.odom_subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        timer_period_sec = 0.05
        self.timer = self.create_timer(timer_period_sec, self.event_loop)

        self.current_odometry: Odometry = None

        self.get_clock().sleep_for(Duration(seconds=1))
        self.set_speed(0, 0.5)

    def event_loop(self):
        pass

    def set_speed(self, lx=0.0, az=0.0):
        twist = Twist()
        twist.linear.x = float(lx)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(az)
        # twist = Twist(linear=Vector3(x=lx), angular=Vector3(z=az))
        self.cmd_publisher.publish(twist)

    # @builtins.staticmethod
    def get_Pose2D(self):
        current = Pose2D()
        current.x = current.y = current.theta = 0.0
        if not self.current_odometry:
            return current

        q = self.current_odometry.pose.pose.orientation

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

        current.theta = np.arctan2(siny_cosp, cosy_cosp)

        if np.abs(q.x * q.y + q.z * q.w) == 0.5:
            current.theta = 0.0

        current.x = self.current_odometry.pose.pose.position.x
        current.y = self.current_odometry.pose.pose.position.y

        return current

    # class target:
    #     x, y, theta = 1, 0, 0

    # class current:
    #     pass

    points = [[0.0, 0.0]]

    # target = Pose2D()
    # target.x = 1.0
    # target.y = 0.0
    # target.theta = 0.0
    
    target = Pose2D(x=1.0,y=1.0,theta=0.0)

    def odom_callback(self, msg: Odometry):
        self.current_odometry = msg

        # if msg.pose.pose.position.x == 0:
        #     self.set_speed(0.0, 0.5)
        # if msg.pose.pose.position.x >= 1:
        #     # self.set_speed(0.0, 1.0)
        #     # self.get_clock().sleep_for(Duration(seconds=2))
        #     # self.set_speed(0.0, 1.0)
        #     self.set_speed(0.0, 0.0)

        # self.get_logger().info(str(theta), throttle_duration_sec=0)

        # av = np.arctan2(d_y, d_x) - current.theta
        # if av > np.pi:
        #     av = av - 2 * np.pi
        # elif av < -np.pi:
        #     av = av + 2 * np.pi

        current = self.get_Pose2D()

        d_x = self.target.x - current.x
        d_y = self.target.y - current.y

        av = np.arctan2(d_y, d_x) - current.theta

        if av > np.pi:
            av = av - 2 * np.pi
        elif av < -np.pi:
            av = av + 2 * np.pi

        distance = np.hypot(d_x, d_y)

        if distance >= 0.1:
            # if abs(av - current.theta) > 0.2:
            # if abs(av) >= 0.1:
            #     # self.set_speed(0.0, 1 * np.sign(av))
            #     self.set_speed(0.0, av)
            # else:
            #     self.set_speed(0.2, 0.0)

            self.set_speed(0.2, av)
        else:
            # if len(self.points):
            #     self.target.x, self.target.x = self.points.pop()
            #     self.get_logger().info(str(self.target))
            # else:
            # self.target = Pose2D(x=1.0,y=1.0,theta=0.0)

            # if (self.target.x, self.target.y) == (1.0, 1.0):
            #     self.target.x, self.target.y = 0.0, 0.0
            # else:
            #     self.target.x, self.target.y = 1.0, 1.0


            if (self.target.x, self.target.y) == (1.0, 1.0):
                self.target.x, self.target.y = 0.0, 0.0
            elif (self.target.x, self.target.y) == (0.0, 1.0):
                self.target.x, self.target.y = 0.0, 1.0
            else:
                self.target.x, self.target.y = 1.0, 0.0



            # self.set_speed(0.0, 0.0)

        # if msg.pose.pose.position.x == 0:
        # lx = 0 if np.abs(d_x) < 0.01 or np.abs(d_y) < 0.01 else 0.2

        # lx = 0 if np.abs(d_x) < 0.05 or np.abs(d_y) < 0.05 else 0.2

        # av = 0 if np.abs(av) < 0.1 else av

        # self.set_speed(lx, av)

        self.get_logger().info(
            str(
                {
                    # "theta": theta,
                    "target": self.target,
                    "current": current,
                    "dx": d_x,
                    "dy": d_y,
                    # "lx": lx,
                    "av": av / np.pi * 360,
                }
            ),
            throttle_duration_sec=0.1,
        )

        # self.get_logger().info(str(av), throttle_duration_sec=1)

        # self.get_logger().info(str(msg.pose.pose.position.x))
        # self.get_logger().info(str(msg.pose.pose))

        # self.get_logger().info(
        #     f"Odom twist: {msg.twist.twist}", throttle_duration_sec=1
        # )

        # self.get_logger().info(f"Odom pos: {msg.pose.pose}", throttle_duration_sec=1)


# Main function
def main(args=None):
    rclpy.init(args=args)

    try:
        diff_drive = DifferentialDrive()
        try:
            rclpy.spin(diff_drive)
        finally:
            diff_drive.destroy_node()
            rclpy.try_shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
