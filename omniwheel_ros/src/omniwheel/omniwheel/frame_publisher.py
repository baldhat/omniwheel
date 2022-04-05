import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations

from geometry_msgs.msg import TransformStamped
from omniwheel_interfaces.msg import Pose as PoseMsg


class FramePublisher(Node):
    """ Creates a Transformation from the omniwheel_pose topic and publishes it. """

    def __init__(self):
        super().__init__('omniwheel_frame_publisher')
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(PoseMsg, 'omniwheel_pose', self.handle_turtle_pose, 1)

    def handle_turtle_pose(self, msg: PoseMsg):
        """ Callback for the omniwheel_pose topic messages.
        Creates TransformStamped object, adds the information from the pose message and broadcasts it.
        """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = "omniwheel"

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, msg.rot)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
