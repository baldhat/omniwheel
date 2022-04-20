
import pyrealsense2 as rs
import numpy as np
import cv2
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, PointField

WIDTH = 1024
HEIGHT = 768
# WIDTH = 320
# HEIGHT = 240

# WIDTH = 848
# HEIGHT = 480


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'lidar/points', 5)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)

        self.get_logger().info(str(self.get_clock().now().to_msg()))

        self.pipeline.start(config)

        self.pc = rs.pointcloud()
        self.decimate = rs.decimation_filter()
        self.decimate.set_option(rs.option.filter_magnitude, 1)
        self.colorizer = rs.colorizer()
        self.filters = [rs.disparity_transform(),
                   rs.spatial_filter(),
                   rs.temporal_filter(),
                   rs.disparity_transform(False)]

    def read_points(self):
        success, frames = self.pipeline.try_wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_frame = self.decimate.process(depth_frame)

        for f in self.filters:
            depth_frame = f.process(depth_frame)

        points = self.pc.calculate(depth_frame)
        points = np.asarray(points.get_vertices(2), dtype='float32')
        points = points[(points[:, 0] > 0) | (points[:, 1] > 0) | (points[:, 2] > 0)]

        return points

    def publish_points(self, points):
        msg = PointCloud2()
        msg.header.frame_id = 'lidar_link'
        msg.width = points.shape[0]
        msg.height = 1
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        msg.fields = [PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]
        start = datetime.now()
        msg._data = points.tobytes()
        print(datetime.now() - start)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.point_step = 3 * itemsize
        msg.row_step = 3 * itemsize * msg.width
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)

    def run(self):
        while True:
            rclpy.spin_once(self, timeout_sec=0.0)
            points = self.read_points()
            self.publish_points(points)


def main(args=None):
    rclpy.init(args=args)
    LidarNode().run()


if __name__ == '__main__':
    main()