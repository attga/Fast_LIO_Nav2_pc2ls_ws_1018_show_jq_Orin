#!/usr/bin/env python3
"""Validate Yesense IMU and fill unknown covariance; EKFs remove gravity."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuMeasurements(Node):
    def __init__(self):
        super().__init__('imu_measurements')
        source = self.declare_parameter('input_topic', '/imu/data_raw').value
        target = self.declare_parameter('output_topic', '/imu/measurements').value
        self.frame = self.declare_parameter('expected_frame', 'gyro_link').value
        # Conservative starting values, not calibrated sensor specifications.
        self.gyro_std = self.declare_parameter('angular_velocity_stddev', 0.05).value
        self.accel_std = self.declare_parameter('linear_acceleration_stddev', 1.0).value
        for value in (self.gyro_std, self.accel_std):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('IMU standard deviations must be finite and positive')
        self.publisher = self.create_publisher(Imu, target, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Imu, source, self.receive, qos_profile_sensor_data)

    @staticmethod
    def covariance(values, stddev):
        if not all(math.isfinite(v) for v in values):
            return None
        # -1 means unavailable, rather than unknown accuracy.
        if values[0] < 0:
            return None
        if all(v == 0 for v in values):
            result = [0.0] * 9
            for i in (0, 4, 8):
                result[i] = stddev * stddev
            return result
        if any(values[i] <= 0 for i in (0, 4, 8)):
            return None
        return list(values)

    def receive(self, message):
        q = message.orientation
        vectors = (message.angular_velocity, message.linear_acceleration)
        valid = (
            message.header.frame_id == self.frame
            and message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec > 0
            and message.orientation_covariance[0] != -1
            and all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w))
            and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 0.01
            and all(math.isfinite(v) for vector in vectors for v in (vector.x, vector.y, vector.z))
        )
        gyro_cov = self.covariance(message.angular_velocity_covariance, self.gyro_std)
        accel_cov = self.covariance(message.linear_acceleration_covariance, self.accel_std)
        if not valid or gyro_cov is None or accel_cov is None:
            self.get_logger().warning('Dropped invalid IMU: check frame, stamp, orientation and covariance', throttle_duration_sec=5.0)
            return
        message.angular_velocity_covariance = gyro_cov
        message.linear_acceleration_covariance = accel_cov
        # Keep the sensor stamp, frame, quaternion and acceleration INCLUDING gravity.
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = ImuMeasurements()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
