#this will be the map controller 
import rclpy
import csv 
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped,Point32
from sensor_msgs.msg import PointCloud,Imu
from std_msgs.msg import Float32



class MapController(Node):
    def __init__(self):
        super().__init__('map_controller')

        # # Transform Listener initialization
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)	


    def getWaypoints(self):
        with open(self.wayfile, mode='r') as file:
            reader = csv.reader(file)
            next(reader)
            #as origin and world differ in location
            #self.waypoints = [[(float(row[0])-3.63), -float(row[1])+8.52] for row in reader]
            self.waypoints = [[(float(row[0])), float(row[1]),float(row[2])] for row in reader]
    
    def control_loop(self):
        """
        Control loop for the MAP controller.
        """
        rate = rospy.Rate(self.loop_rate)

        while not rospy.is_shutdown():
            # Wait for position and waypoints
            if self.position is None or self.waypoints is None:
                continue

            # Send speed and steering commands to mux
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = self.ros_time.now()
            ack_msg.header.frame_id = 'base_link'

            if self.waypoints.shape[0] > 2:
                idx_nearest_waypoint = self.nearest_waypoint(self.position[:2], self.waypoints[:, :2])

                # Desired speed at waypoint closest to car
                target_speed = self.waypoints[idx_nearest_waypoint, 2]

                # Calculate lookahead_distance
                # Define lookahead distance as an affine function with  tuning parameter m and q
                lookahead_distance = self.param_q_map + target_speed*self.param_m_map
                lookahead_distance = np.clip(lookahead_distance, self.param_t_clip_min, self.param_t_clip_max)
                lookahead_point = self.waypoint_at_distance_infront_car(lookahead_distance,
                                                                 self.waypoints[:, :2],
                                                                 idx_nearest_waypoint)

                if lookahead_point.any() is not None:
                    # Vector from the current position to the point at lookahead distance
                    position_la_vector = np.array([lookahead_point[0] - self.position[0], lookahead_point[1] - self.position[1]])
                    yaw = self.position[2]
                    eta = np.arcsin(np.dot([-np.sin(yaw), np.cos(yaw)], position_la_vector)/np.linalg.norm(position_la_vector))
                    lat_acc = 2*target_speed**2 / lookahead_distance * np.sin(eta)

                    steering_angle = self.steer_lookup.lookup_steer_angle(lat_acc, target_speed)
                    ack_msg.drive.steering_angle = steering_angle
                    ack_msg.drive.speed = np.max(target_speed, 0)  # no negative speed

                    # self.visualize_lookahead(lookahead_point)
                    # self.visualize_steering(steering_angle)

            # If there are no waypoints, publish zero speed and steer to STOP
            else:
                ack_msg.drive.speed = 0
                ack_msg.drive.steering_angle = 0
                rospy.logerr('[MAP Controller]: Received no waypoints. STOPPING!!!')

            # Always publish ackermann msg
            self.drive_pub.publish(ack_msg)
            rate.sleep()