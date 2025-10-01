#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import os
import copy
import numpy as np
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan


class LineSegment:

    def __init__(self, a, b, c, d):
        self.start_point = np.array([a, b])
        self.end_point = np.array([c, d])
        self.point_difference = np.subtract(self.end_point, self.start_point)

    def __repr__(self):
        return '(%d, %d, %d, %d)' % (self.start_point[0], self.start_point[1], self.end_point[0], self.end_point[1])


# Create a class which saves the location of the drone and simulates the lidar
class Lidar(Node):

    # Node initialization
    def __init__(self):
        super().__init__('LidarNode')
        # Get the default path (need to remove src)
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.dir_path = self.dir_path + "/data/"

        # Get image name and load it from data folder
        self.declare_parameter('/lidar_node/map_path', "room.txt")
        self.file_path = self.get_parameter('/lidar_node/map_path').value


        self.declare_parameter('/lidar_node/angle_min', -np.pi, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('/lidar_node/angle_max', np.pi, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('/lidar_node/reading_count', 360, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('/lidar_node/range_max', 30.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('/lidar_node/lidar_range_noise', 0.5, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('/lidar_node/lidar_position_noise', 0.5, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.angle_min = self.get_parameter('/lidar_node/angle_min').value
        self.angle_max = self.get_parameter('/lidar_node/angle_max').value
        self.reading_count = self.get_parameter('/lidar_node/reading_count').value
        self.angle_increment = (self.angle_max - self.angle_min) / self.reading_count

        self.range_max = self.get_parameter('/lidar_node/range_max').value
        
        self.range_noise = self.get_parameter('/lidar_node/lidar_range_noise').value

        self.lidar_position_noise = self.get_parameter('/lidar_node/lidar_position_noise').value

        self.walls = []
        with open(self.dir_path + self.file_path, 'r') as f:
            for line in f.readlines():
                (a, b), (c, d) = eval(line)
                self.walls.append(LineSegment(a, b, c, d))
        # Create the subscribers
        self.position_sub = self.create_subscription(PoseStamped, '/uav/sensors/ground_truth', self.getPosition, 1)

        # Create the publishers
        self.lidar_pub = self.create_publisher(LaserScan, "/uav/sensors/lidar", 10)
        self.lidar_position_pub = self.create_publisher(PointStamped, "/uav/sensors/lidar_position", 10)

        # Save the drones positions
        self.position = np.zeros(3, dtype=np.float64)
        self.yaw = 0

        self.rate = 10
        self.dt = 1.0 / self.rate
        # Call the mainloop of our class
        self._timer = self.create_timer(self.dt, self.mainloop)

    # Callback for the keyboard manager
    def getPosition(self, msg):
        # Save the drones alitude
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z

        # Get the drones attitude
        quarternion_pose = (msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w)
        self.yaw = np.array(euler_from_quaternion(quarternion_pose))[2]

        self.noisy_position = PointStamped()
        self.noisy_position.header = copy.deepcopy(msg.header)
        self.noisy_position.point.x = np.random.normal(msg.pose.position.x, self.lidar_position_noise)
        self.noisy_position.point.y = np.random.normal(msg.pose.position.y, self.lidar_position_noise)
        self.noisy_position.point.z = np.random.normal(msg.pose.position.z, self.lidar_position_noise)
        self.lidar_position_pub.publish(self.noisy_position)

    # The main loop of the function
    def mainloop(self):
        position = np.array([self.position[0], self.position[1]])
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "uav/imu_level"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.scan_time = self.dt  # unused
        scan.intensities = []  # unused
        scan.range_min = 0.0
        scan.range_max = self.range_max
        ranges = np.zeros(self.reading_count, dtype=np.float32)

        # simulate the ray tracing for the lidar to the simulated walls
        # adapted from https://stackoverflow.com/questions/14307158/how-do-you-check-for-intersection-between-a-line-segment-and-a-line-ray-emanatin
        for reading in range(self.reading_count):
            min_dist = np.inf
            angle = self.yaw + self.angle_min + reading * self.angle_increment
            x = np.cos(angle)
            y = np.sin(angle)
            v3 = np.array([-y, x])
            for wall in self.walls:
                v1 = np.subtract(position, wall.start_point)
                dot = np.dot(wall.point_difference, v3)
                # if the ray and the line are parallel, they cannot intersect
                if abs(dot) < 1e-6:
                    continue  # the value is infinite, so we do not need to update anything
                dist = np.cross(wall.point_difference, v1) / dot
                intersection_point = np.dot(v1, v3) / dot
                # if the distance along the ray is positive and the intersection is between the start and end, check the distance
                if dist >= 0.0 and 0.0 <= intersection_point <= 1.0:
                    min_dist = min(min_dist, dist)
            # if the value is beyond our maximum range, return inf
            ranges[reading] = np.inf if min_dist > self.range_max else min_dist

        # Add noise
        ranges += np.random.normal(0, self.range_noise, ranges.shape)
        scan.ranges = ranges
        # Publish
        self.lidar_pub.publish(scan)


if __name__ == '__main__':
    rclpy.init()
    try:
        rclpy.spin(Lidar())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()