import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import time
import numpy as np
import math

from omniwheel.helper.helper import to_polar

from omniwheel_interfaces.action import Waypoints
from omniwheel_interfaces.msg import Pose, ControllerValue
from omniwheel_interfaces.srv import EnableMotors


class PathExecutor(Node):

    def __init__(self):
        super().__init__('path_executor')
        self._action_server = ActionServer(self, Waypoints, 'waypoints',
                                           self.execute_callback, cancel_callback=self.cancel_callback)
        self.publisher_ = self.create_publisher(ControllerValue, 'controller_value', 10)
        self.enable_motors_client = self.create_client(EnableMotors, 'enable_motors')
        self.enable_motors_future = None
        self.pose_subscription = self.create_subscription(Pose, 'omniwheel_pose', self.pose_callback, 10)

        self.pose = Pose()
        self.motors_enabled = False

        self.MAX_POS_ERROR = 0.01
        self.MAX_ROT_ERROR = 0.02

        self.shouldStop = False
        self.last_tick = time.time_ns()
        self.get_logger().info("Ready...")

    def execute_callback(self, goal_handle):
        self.pre_execute()

        for pose in goal_handle.request.poses:
            self.drive_to_pose(pose)
            if not self.shouldStop:
                self.send_feedback(goal_handle)

        return self.post_execute(goal_handle)

    def post_execute(self, goal_handle):
        self.send_enable_motors(False)
        goal_handle.succeed()
        result = Waypoints.Result()
        result.final_pose = self.pose
        return result

    def pre_execute(self):
        self.shouldStop = False
        self.send_enable_motors(True)

    def drive_to_pose(self, pose: Pose):
        while not self.poseReached(pose) and not self.shouldStop:
            direction, velocity, rotation = self.calculate_controller_value(pose)
            self.sendControllerValue(direction, velocity, rotation)
            self.executor.spin_once(timeout_sec=0.0)
            time.sleep(0.05)

    def poseReached(self, pose):
        return self.distance_to(pose) <= self.MAX_POS_ERROR and abs(self.rot_distance_to(pose)) <= self.MAX_ROT_ERROR

    def calculate_controller_value(self, pose: Pose):
        direction, drot, dist = self.calculate_distances(pose)
        velocity = self.calculate_velocity(dist, pose)
        rotation = self.calculate_rotation(drot, pose)
        return direction, velocity, rotation

    def calculate_rotation(self, drot, pose):
        if self.rot_distance_to(pose) > self.MAX_ROT_ERROR:
            rotation = - (drot / abs(drot) if abs(drot) > 0.5 else 2 * drot)
        else:
            rotation = 0
        return rotation

    def calculate_velocity(self, dist, pose):
        if self.distance_to(pose) > self.MAX_POS_ERROR:
            velocity = 1 if dist > 0.1 else 10 * dist
        else:
            velocity = 0
        return velocity

    def calculate_distances(self, pose):
        dx = pose.x - self.pose.x
        dy = pose.y - self.pose.y
        drot = self.rot_distance_to(pose)
        dist = np.sqrt(dx ** 2 + dy ** 2)
        direction = to_polar(dx, dy)[0] - math.pi / 2 - self.pose.rot
        return direction, drot, dist

    def distance_to(self, pose: Pose):
        return np.sqrt((pose.x - self.pose.x)**2 + (pose.y - self.pose.y)**2)

    def rot_distance_to(self, pose):
        drot = (pose.rot - self.pose.rot if pose.rot > self.pose.rot else self.pose.rot - pose.rot) % (2 * math.pi)
        return (2 * math.pi - drot) if drot > math.pi else drot

    def sendControllerValue(self, direction, velocity, rotation):
        msg = ControllerValue()
        msg.direction = float(direction)
        msg.velocity = float(velocity)
        msg.rotation = float(rotation)
        self.publisher_.publish(msg)

    def cancel_callback(self, _):
        self.shouldStop = True
        return 2

    def send_feedback(self, goal_handle):
        feedback_msg = Waypoints.Feedback()
        feedback_msg.completed_pose = self.pose
        goal_handle.publish_feedback(feedback_msg)

    def send_enable_motors(self, value):
        request = EnableMotors.Request()
        request.enable = value
        response = self.enable_motors_client.call(request)
        self.motors_enabled = response.enabled

    def pose_callback(self, msg):
        self.pose = msg


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads=2)
    path_executor = PathExecutor()
    executor.add_node(path_executor)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        path_executor.destroy_node()


if __name__ == '__main__':
    main()
