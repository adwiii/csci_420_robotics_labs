#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np
from std_msgs.msg import Float64


def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass

# Create the rocket engine class
class RocketEngine(Node):
    # On node initilization
    def __init__(self):
        super().__init__('RocketEngineNode')
        # Create the publisher and subscriber
        self.position_sub = self.create_subscription(
            Float64,
            '/cmd_vel',
            self.velocity_callback,
            10)
        # Create the sensor reading message we will send
        self.sensor_vel = Float64()
        self.sensor_vel.data = 0.0
        self.velocity_pub = self.create_publisher(Float64, '/sensor_vel', 1)
        self._timer = self.create_timer(0.1, self.do_publish)

    # Callback for the command velocity
    def velocity_callback(self, msg):
        # Set the sensor mean and deviation
        mu, sigma = 0, 10
        # Generate gaussian random noise
        s = np.random.normal(mu, sigma, 1)
        # Sensor reading is the velocity command + gaussian noise
        self.sensor_vel.data = msg.data + s[0]

    def do_publish(self):
        # Publish the sensor reading
        self.velocity_pub.publish(self.sensor_vel)

# Main function
if __name__ == '__main__':
    rclpy.init()
    try:
        rclpy.spin(RocketEngine())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()