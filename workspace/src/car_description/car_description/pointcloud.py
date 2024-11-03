# laser_to_pointcloud.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import sensor_msgs_py.point_cloud2 as pc2

class LaserToPointCloudNode(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud_node')

        # Initialize laser projector
        self.laser_projector = LaserProjection()

        # Subscribe to the laser scan topic
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your laser scan topic name
            self.laser_callback,
            10
        )

        # Publisher for point cloud
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/point_cloud',  # Topic to publish point cloud
            10
        )

    def laser_callback(self, msg: LaserScan):
        # Convert LaserScan to PointCloud2
        point_cloud = self.laser_projector.projectLaser(msg)

        # Publish the PointCloud2 message
        self.pointcloud_pub.publish(point_cloud)

def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
