#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import time
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Quaternion
import numpy as np
import copy
import tf2_ros
from transforms3d._gohlketransforms import euler_from_quaternion, quaternion_from_euler

from rcl_interfaces.msg import ParameterDescriptor, ParameterType


# Create a class which saves the altitude of the drone and estimates the pressure
class NoiseAndTransformationsNode(Node):

    # Node initialization
    def __init__(self):
        super().__init__('noise_and_transformations_node')
        # Sleep to allow simulation to start
        time.sleep(5)

        # Create the publisher and subscriber
        self.declare_parameter('/uav/sensors/position_noise_lvl', 0.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('/uav/sensors/attitude_noise_lvl', 0.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('/uav/sensors/velocity_noise_lvl', 0.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('/uav/sensors/velocity_attitude_noise_lvl', 0.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.position_noise_lvl = self.get_parameter('/uav/sensors/position_noise_lvl').value
        self.attitude_noise_lvl = self.get_parameter('/uav/sensors/attitude_noise_lvl').value
        self.velocity_noise_lvl = self.get_parameter('/uav/sensors/velocity_noise_lvl').value
        self.velocity_attitude_noise_lvl = self.get_parameter('/uav/sensors/velocity_attitude_noise_lvl').value

        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.position = PoseStamped()
        self.ground_truth = PoseStamped()
        self.ground_truth_velocity = TwistStamped()
        self.velocity = TwistStamped()
        self.gps_noise_pub = self.create_publisher(PoseStamped, "/uav/sensors/gps_noisy", 1)
        self.ground_truth_pub = self.create_publisher(PoseStamped, "/uav/sensors/ground_truth", 1)
        self.velocity_noise_pub = self.create_publisher(TwistStamped, "/uav/sensors/velocity_noisy", 1)

        self.position_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.get_position, 1)
        self.velocity_sub = self.create_subscription(TwistStamped, '/uav/sensors/velocity', self.get_velocity, 1)

    def get_velocity(self, msg):
        self.ground_truth_velocity = msg
        self.velocity = copy.deepcopy(msg)
        self.velocity.twist.linear.x += np.random.normal(0, self.velocity_noise_lvl)
        self.velocity.twist.linear.y += np.random.normal(0, self.velocity_noise_lvl)
        self.velocity.twist.linear.z += np.random.normal(0, self.velocity_noise_lvl)

        self.velocity.twist.angular.x += np.random.normal(0, self.velocity_attitude_noise_lvl)
        self.velocity.twist.angular.y += np.random.normal(0, self.velocity_attitude_noise_lvl)
        self.velocity.twist.angular.z += np.random.normal(0, self.velocity_attitude_noise_lvl)

        self.velocity_noise_pub.publish(self.velocity)

    def get_position(self, msg):
        self.ground_truth = msg
        self.ground_truth_pub.publish(self.ground_truth)
        self.position = copy.deepcopy(msg)
        self.position.pose.position.x += np.random.normal(0, self.position_noise_lvl)
        self.position.pose.position.y += np.random.normal(0, self.position_noise_lvl)
        self.position.pose.position.z += np.random.normal(0, self.position_noise_lvl)
        quat = self.position.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        roll_noisy = np.random.normal(roll, self.attitude_noise_lvl)
        pitch_noisy = np.random.normal(pitch, self.attitude_noise_lvl)
        yaw_noisy = np.random.normal(yaw, self.attitude_noise_lvl)
        self.position.pose.orientation.x, \
            self.position.pose.orientation.y, \
            self.position.pose.orientation.z, \
            self.position.pose.orientation.w = quaternion_from_euler(roll_noisy, pitch_noisy, yaw_noisy)
        self.gps_noise_pub.publish(self.position)
        imu_level = TransformStamped()
        imu_level.header.frame_id = "world"
        imu_level.child_frame_id = "uav/imu_level"
        imu_level.header.stamp = self.get_clock().now().to_msg()
        imu_level.transform.translation.x = self.ground_truth.pose.position.x
        imu_level.transform.translation.y = self.ground_truth.pose.position.y
        imu_level.transform.translation.z = self.ground_truth.pose.position.z
        imu_level.transform.rotation.x, \
            imu_level.transform.rotation.y, \
            imu_level.transform.rotation.z, \
            imu_level.transform.rotation.w = quaternion_from_euler(0, 0, yaw)
        self.tf_pub.sendTransform(imu_level)
        imu_ground = TransformStamped()
        imu_ground.header.frame_id = "world"
        imu_ground.child_frame_id = "uav/imu_ground"
        imu_ground.header.stamp = self.get_clock().now().to_msg()
        imu_ground.transform.translation.x = self.ground_truth.pose.position.x
        imu_ground.transform.translation.y = self.ground_truth.pose.position.y
        imu_ground.transform.translation.z = 0.0
        imu_ground.transform.rotation.x, \
            imu_ground.transform.rotation.y, \
            imu_ground.transform.rotation.z, \
            imu_ground.transform.rotation.w = quaternion_from_euler(np.pi, np.pi, yaw)
        self.tf_pub.sendTransform(imu_ground)
        self.tf_pub.sendTransform(imu_level)
        imu = TransformStamped()
        imu.header.frame_id = "world"
        imu.child_frame_id = "uav/imu"
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.transform.translation.x = self.ground_truth.pose.position.x
        imu.transform.translation.y = self.ground_truth.pose.position.y
        imu.transform.translation.z = self.ground_truth.pose.position.z
        imu_ground.transform.rotation.x, \
            imu_ground.transform.rotation.y, \
            imu_ground.transform.rotation.z, \
            imu_ground.transform.rotation.w = quaternion_from_euler(roll+np.pi, pitch+np.pi, yaw)
        self.tf_pub.sendTransform(imu)


if __name__ == '__main__':
    rclpy.init()
    try:
        rclpy.spin(NoiseAndTransformationsNode())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()