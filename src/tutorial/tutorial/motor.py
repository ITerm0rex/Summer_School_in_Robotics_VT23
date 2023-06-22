import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import brickpi3


class MotorController(Node):
    def __init__(self):
        super().__init__("lego_motor_controller")

        self.subscription = self.create_subscription(
            Int32,
            "speed",
            self.callback,
        )

        self.declare_parameter("port", "A")


from brickpi3 import BrickPi3
import time

# Create an instance of the BrickPi3 class
brickpi = BrickPi3()

# Set motor parameters
motor_power = 50  # Power level for the motors
turn_delay = 0.5  # Delay between turns

from brickpi3 import BrickPi3
import time

# Create an instance of the BrickPi3 class
brickpi = BrickPi3()

# Set motor parameters
motor_power = 50  # Power level for the motors
turn_delay = 0.5  # Delay between turns

from brickpi3 import BrickPi3
import time

# Create an instance of the BrickPi3 class
brickpi = BrickPi3()

# Set motor parameters
motor_power = 50  # Power level for the motors
turn_delay = 0.5  # Delay between turns

# Drive the car straight
brickpi.set_motor_power(brickpi.PORT_A, motor_power)
brickpi.set_motor_power(brickpi.PORT_B, motor_power)
time.sleep(2)  # Drive straight for 2 seconds

# Stop the car
brickpi.set_motor_power(brickpi.PORT_A, 0)
brickpi.set_motor_power(brickpi.PORT_B, 0)
time.sleep(1)  # Pause for 1 second

# Drive the car back in a square
# Drive forward
brickpi.set_motor_power(brickpi.PORT_A, motor_power)
brickpi.set_motor_power(brickpi.PORT_B, motor_power)
time.sleep(2)  # Drive straight for 2 seconds

# Stop and turn
brickpi.set_motor_power(brickpi.PORT_A, 0)
brickpi.set_motor_power(brickpi.PORT_B, 0)
time.sleep(1)  # Pause for 1 second

# Turn right
brickpi.set_motor_power(brickpi.PORT_A, motor_power)
brickpi.set_motor_power(brickpi.PORT_B, -motor_power)
time.sleep(turn_delay)  # Turn for specified delay
brickpi.set_motor_power(brickpi.PORT_A, 0)
brickpi.set_motor_power(brickpi.PORT_B, 0)
time.sleep(1)  # Pause for 1 second

# Stop the car
brickpi.set_motor_power(brickpi.PORT_A, 0)
brickpi.set_motor_power(brickpi.PORT_B, 0)
time.sleep(1)  # Pause for 1 second

# Drive the car in a circle
# Drive forward
brickpi.set_motor_power(brickpi.PORT_A, motor_power)
brickpi.set_motor_power(brickpi.PORT_B, motor_power)
time.sleep(2)  # Drive straight for 2 seconds

# Stop and turn
brickpi.set_motor_power(brickpi.PORT_A, 0)
brickpi.set_motor_power(brickpi.PORT_B, 0)
time.sleep(1)  # Pause for 1 second

# Turn right
brickpi.set_motor_power(brickpi.PORT_A, motor_power)
brickpi.set_motor_power(brickpi.PORT_B, -motor_power)
time.sleep(turn_delay)  # Turn for specified delay
brickpi.set_motor_power(brickpi.PORT_A, 0)
brickpi.set_motor_power(brickpi.PORT_B, 0)
time.sleep(1)  # Pause for 1 second

# Stop the car
brickpi.set_motor_power(brickpi.PORT_A, 0)
brickpi.set_motor_power(brickpi.PORT_B, 0)
time.sleep(1)  # Pause for 1 second

# Drive the car in an infinity sign
# Drive forward
brickpi.set_motor_power(brickpi.PORT_A, motor_power)
brickpi.set_motor_power(brickpi.PORT_B, motor_power)
time.sleep(2)  # Drive straight for 2 seconds

# Stop and turn
brickpi.set_motor_power(brickpi.PORT_A, 0)
brickpi.set_motor_power(brickpi.PORT_B, 0)
time.sleep(1)
