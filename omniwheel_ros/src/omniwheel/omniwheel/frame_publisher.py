import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class FramePublisher(Node):
    """ Creates a Transformation from the wheel_odometry topic and publishes it.
    Subscriber:
        - /wheel_odometry
    """

    def __init__(self):
        super().__init__('base_link_frame_publisher')
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, '/wheel_odometry', self.handle_turtle_pose, 1)

    def handle_turtle_pose(self, msg: Odometry):
        """ Callback for the wheel_odometry topic messages.
        Creates TransformStamped object, adds the information from the pose message and broadcasts it.
        """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = "base_link"

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z + 0.065  # This is the offset of the center of the robot from the ground
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
