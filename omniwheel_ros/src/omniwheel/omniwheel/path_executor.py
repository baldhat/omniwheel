import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import time
import numpy as np
import math
import cmath

from omniwheel_interfaces.action import Waypoints
from omniwheel_interfaces.msg import Pose, ControllerValue

def to_polar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    if r > 1:
        r = 1
    t = cmath.polar(x + y * 1j)[1]
    return t, r


class PathExecutor(Node):

    def __init__(self):
        super().__init__('path_executor')
        self._action_server = ActionServer(self, Waypoints, 'waypoints', self.execute_callback)
        self.pose_subscription = self.create_subscription(Pose, 'omniwheel_pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(ControllerValue, 'controller_value', 10)

        self.pose = Pose()

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing waypoint mission")

        feedback_msg = Waypoints.Feedback()
        feedback_msg.completed_poses = []

        for pose in goal_handle.request.poses:
            self.drive_to_pose(pose)
            feedback_msg.completed_poses.append(self.pose)
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Waypoints.Result()
        result.final_pose = self.pose

    def pose_callback(self, msg):
        self.pose = msg

    def drive_to_pose(self, pose: Pose):
        self.get_logger().info("Driving to pose " + str(pose) + " from current pose " + str(self.pose))
        while self.distanceTo(pose) > 0.05 or self.angularOffset(pose) > 0.5:
            direction, velocity, rotation = self.calculateControllerValue(pose)
            self.sendControllerValue(direction, velocity, rotation)
            time.sleep(0.05)

    def calculateControllerValue(self, pose: Pose):
        dx = pose.x - self.pose.x
        dy = pose.y - self.pose.y
        drot = pose.rot - self.pose.rot
        dist = np.sqrt(dx**2 + dy**2)
        if dist > 0.05:
            direction = to_polar(dx, dy)[0]
            velocity = 1 if dist > 0.2 else 5 * dist
            rotation = 0
        else:
            direction = 0
            velocity = 0
            rotation = drot/abs(drot) if abs(drot) > 1 else drot
        return direction, velocity, rotation

    def sendControllerValue(self, direction, velocity, rotation):
        msg = ControllerValue()
        msg.direction = float(direction)
        msg.velocity = float(velocity)
        msg.rotation = float(rotation)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    path_executor = PathExecutor()

    rclpy.spin(path_executor)


if __name__ == '__main__':
    main()