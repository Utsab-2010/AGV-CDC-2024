#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_publisher')
        self.broadcaster = TransformBroadcaster(self)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.broadcast_transform)
        self.angle = 0.0  # an example variable that you can update

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'

        # Example: update translation dynamically (you can change this logic)
        t.transform.translation.x = math.cos(self.angle)
        t.transform.translation.y = math.sin(self.angle)
        t.transform.translation.z = 0.0

        # Example: keep the rotation as identity (or you could update it too)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transform
        self.broadcaster.sendTransform(t)

        # Update the dynamic variable (e.g., a changing angle)
        self.angle += 0.1
        # Optionally, wrap the angle:
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
