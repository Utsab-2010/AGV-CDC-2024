import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ParticleCloudBridge(Node):
    def __init__(self):
        super().__init__('particlecloud_bridge')
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(ParticleCloud, '/particle_cloud', self.callback, qos_profile)
        self.pub = self.create_publisher(PoseArray, '/amcl/particlecloud_legacy', 10)

    def callback(self, msg: ParticleCloud):
        pa = PoseArray()
        pa.header = msg.header
        pa.poses = [p.pose for p in msg.particles]
        self.pub.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleCloudBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
#...
