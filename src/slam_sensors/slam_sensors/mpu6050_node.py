#!/usr/bin/env python3
"""MPU6050 IMU ROS2 Node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time
import math

class MPU6050Node(Node):
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    
    def __init__(self):
        super().__init__('mpu6050_node')
        
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate', 100)
        
        i2c_bus = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('rate').value
        
        try:
            self.bus = smbus2.SMBus(i2c_bus)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
            time.sleep(0.1)
            self.get_logger().info(f'MPU6050 initialized on bus {i2c_bus}')
        except Exception as e:
            self.get_logger().error(f'Init failed: {e}')
            raise
        
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(1.0 / rate, self.read_and_publish)
        
        self.get_logger().info(f'MPU6050 node started at {rate} Hz')
    
    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val
    
    def read_and_publish(self):
        try:
            accel_x = self.read_word_2c(self.ACCEL_XOUT_H)
            accel_y = self.read_word_2c(self.ACCEL_XOUT_H + 2)
            accel_z = self.read_word_2c(self.ACCEL_XOUT_H + 4)
            
            gyro_x = self.read_word_2c(self.GYRO_XOUT_H)
            gyro_y = self.read_word_2c(self.GYRO_XOUT_H + 2)
            gyro_z = self.read_word_2c(self.GYRO_XOUT_H + 4)
            
            accel_scale = 9.80665 / 16384.0
            gyro_scale = (math.pi / 180.0) / 131.0
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            msg.linear_acceleration.x = accel_x * accel_scale
            msg.linear_acceleration.y = accel_y * accel_scale
            msg.linear_acceleration.z = accel_z * accel_scale
            
            msg.angular_velocity.x = gyro_x * gyro_scale
            msg.angular_velocity.y = gyro_y * gyro_scale
            msg.angular_velocity.z = gyro_z * gyro_scale
            
            msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                                   0.0, 0.01, 0.0,
                                                   0.0, 0.0, 0.01]
            msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                                0.0, 0.01, 0.0,
                                                0.0, 0.0, 0.01]
            msg.orientation_covariance[0] = -1
            
            self.imu_pub.publish(msg)
        
        except Exception as e:
            self.get_logger().error(f'Read error: {e}')
    
    def destroy_node(self):
        try:
            self.bus.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
