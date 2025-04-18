#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion
import numpy as np
from ekf import ExtendedKalmanFilter  # Import the EKF module

class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')

        # Parameters (can be set via ROS 2 parameters or hardcoded here)
        self.declare_parameter('dt', 0.01)  # Time step
        self.declare_parameter('wheel_radius', 0.0590)  # Radius of wheels (meters)
        self.declare_parameter('wheel_base', 0.2)  # Distance between wheels (meters)
        # Load parameters
        dt = self.get_parameter('dt').value
        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_base = self.get_parameter('wheel_base').value
        
        # Define process and measurement noise covariance matrices
        process_noise = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.01])
        measurement_noise = np.diag([0.05, 0.05, 0.01])

        # Initialize EKF object
        self.ekf = ExtendedKalmanFilter(dt=dt,
                                        wheel_radius=wheel_radius,
                                        wheel_base=wheel_base,
                                        process_noise=process_noise,
                                        measurement_noise=measurement_noise)

        # Initialize state variables for encoder and IMU data
        self.last_left_wheel_ticks = None
        self.last_right_wheel_ticks = None

        # Subscribers for IMU and encoder data
        self.create_subscription(Imu, '/autodrive/f1tenth/imu', self.imu_callback, 10)
        self.create_subscription(Float64, '/autodrive/f1tenth/left_encoder', self.left_encoder_callback, 10)
        self.create_subscription(Float64, '/autodrive/f1tenth/right_encoder', self.right_encoder_callback, 10)

        # Publisher for the estimated state (e.g., odometry)
        self.state_publisher = self.create_publisher(Odometry, '/ekf/odom', 10)

        self.timer = self.create_timer(dt,self.compute)


    def imu_callback(self, msg):
        """
        Callback for IMU data.
        
        Parameters:
            msg: sensor_msgs/Imu message containing orientation (quaternion), angular velocity,
                 linear acceleration, and their covariance.
        """
        orientation_q = msg.orientation
        angular_velocity = msg.angular_velocity.z
        linear_acceleration_x = msg.linear_acceleration.x

        # Convert quaternion to yaw (orientation about z-axis)
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        # Extract variances from covariance matrices
        orientation_var = msg.orientation_covariance[8]          # Variance for yaw (3rd row, 3rd column)
        angular_vel_var = msg.angular_velocity_covariance[8]     # Variance for angular velocity (z-axis)
        linear_accel_var = msg.linear_acceleration_covariance[0] # Variance for linear acceleration (x-axis)

        # Update the EKF's measurement noise covariance matrix R
        new_R = np.diag([orientation_var, angular_vel_var, linear_accel_var])
        self.ekf.set_measurement_noise(new_R)

        # Store IMU data in a format usable by the EKF
        imu_data = {
            'yaw': yaw,
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration_x,
            'covariance': msg.orientation_covariance,
        }

        # Update EKF state with IMU data if encoder data is present
        if self.last_left_wheel_ticks is not None and self.last_right_wheel_ticks is not None:
            self.update_ekf(imu_data)

    def left_encoder_callback(self, msg):
        """
        Callback for left wheel encoder ticks.
        """
        self.delta_left_ticks = (msg.position[0] - self.last_left_wheel_ticks)/16
        self.last_left_wheel_ticks = msg.position[0]

    def right_encoder_callback(self, msg):
        """
        Callback for right wheel encoder ticks.
        """
        self.delta_right_ticks = (msg.position[0] - self.last_right_wheel_ticks)/16
        self.last_right_wheel_ticks = msg.position[0]

    def update_ekf(self, imu_data):
        ''' Update the EKF with IMU and encoder data and publish the estimated state. '''
                      
        # Prepare control input from encoder ticks (left and right wheel rotations)
        u_encoders = [self.delta_left_ticks, self.delta_right_ticks]

        # Prepare measurement input from IMU data (yaw)
        z_imu = [imu_data['yaw'], imu_data['angular_velocity'], imu_data['linear_acceleration']]

        # Perform EKF step to update state estimate based on sensor inputs
        self.ekf.step(u=u_encoders, z=z_imu)

        # Publish the updated state as an Odometry message
        odom_msg = Odometry()