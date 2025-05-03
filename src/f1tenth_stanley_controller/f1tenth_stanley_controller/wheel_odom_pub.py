#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64,Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState,Imu
import math
import tf_transformations
from tf_transformations import euler_from_quaternion

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

    #     self.declare_parameters(
    #     namespace='',
    #     parameters=[
    #         ('wheel_separation', 0.25),
    #         ('wheel_diameter', 0.13),
    #         ('ticks_per_rev', 2626),
    #         ('base_frame', 'base_link'),
    #         ('odom_frame', 'odom')
    #     ]
    # )
        
        # Initialize encoder counters and pose
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.steering_angle = 0
        # self.x = 0.
        # self.x = 0.74
        # self.y = 2.8418
        # self.theta = 1.57
        self.x=0
        self.y=0
        self.theta=1.57
        
        # Robot physical parameters (modify these for your robot)
        self.wheel_separation = 0.2360   # Distance between wheels in meters
        self.wheel_diameter = 0.0590*2     # Wheel diameter in meters
        self.wheelbase  = 0.3240
        self.ticks_per_rev = 16     # Encoder ticks per full revolution
        self.imu_data = None
        # Calculate distance per tick
        self.distance_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev
        
        # Create publisher and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odometry', 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/left_encoder', self.left_callback, 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/right_encoder', self.right_callback, 10)
        self.create_subscription(Float32, '/autodrive/f1tenth_1/steering', self.steering_callback, 10)
        self.create_subscription(Imu,'/autodrive/f1tenth_1/imu',self.imu_callback,10)
        # Initialize time tracking
        self.last_time = self.get_clock().now()
        self.get_logger().info("hellooo")
        print("yes")

        # Create timer for periodic updates (50Hz)
        self.create_timer(0.02, self.update_odometry)

    def left_callback(self, msg):
        # print("left call")
        self.left_ticks = msg.position[0]
        # print(self.left_ticks)
        # print("li")

    def right_callback(self, msg):
        self.right_ticks = msg.position[0]

        # self.get_logger().info("hellooo")
        # print(self.right_ticks)
        # print("hi")

    def steering_callback(self, msg):
        # print("steer")
        self.steering_angle = msg.data  # Expecting radians
        # print(self.steering_angle)
    
    def imu_callback(self,msg):
        orientation_q = msg.orientation
        angular_velocity = msg.angular_velocity.z
        linear_acceleration_x = msg.linear_acceleration.x

        # Convert quaternion to yaw (orientation about z-axis)
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        # Extract variances from covariance matrices
        # orientation_var = msg.orientation_covariance[8]          # Variance for yaw (3rd row, 3rd column)
        # angular_vel_var = msg.angular_velocity_covariance[8]     # Variance for angular velocity (z-axis)
        # linear_accel_var = msg.linear_acceleration_covariance[0] # Variance for linear acceleration (x-axis)

        # Update the EKF's measurement noise covariance matrix R
        # new_R = np.diag([orientation_var, angular_vel_var, linear_accel_var])
        # self.ekf.set_measurement_noise(new_R)

        # Store IMU data in a format usable by the EKF
        self.imu_data = {
            'yaw': yaw,
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration_x,
            'covariance': msg.orientation_covariance,
        }

    def update_odometry(self):
        if not(self.imu_data):
            return
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        # Calculate rear wheel displacements
        delta_left = self.left_ticks - self.last_left_ticks
        delta_right = self.right_ticks - self.last_right_ticks
        left_dist = delta_left * self.distance_per_tick
        right_dist = delta_right * self.distance_per_tick
        
        # Average rear wheel movement for linear displacement
        linear_displacement = (left_dist + right_dist) / 2.0
        linear_velocity = linear_displacement / dt

        # Ackermann steering kinematics (bicycle model)
        # if abs(self.steering_angle) > 0.001:  # Avoid division by zero
        #     turn_radius = self.wheelbase / math.tan(self.steering_angle)
        #     angular_velocity = linear_velocity / turn_radius
        # else:
        #     angular_velocity = 0.0

        # Update pose
        self.theta = self.imu_data['yaw']
        if self.theta>3.142:
            self.theta-=3.14159

        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt

        # Create and populate odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'f1tenth_1'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        # print("--------")
        # print("x:",self.x)
        # print("y:",self.y)
        # Orientation (convert to quaternion)
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Velocity
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = self.imu_data["angular_velocity"]
        
        # Covariance matrix (adjust values based on your system's uncertainty)
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        # Publish and update state
        self.odom_pub.publish(odom_msg)
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
