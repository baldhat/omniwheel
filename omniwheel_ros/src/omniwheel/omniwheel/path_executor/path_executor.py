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
                                           self.execute_callback)
        self.publisher_ = self.create_publisher(ControllerValue, 'controller_value', 10)
        self.enable_motors_client = self.create_client(EnableMotors, 'enable_motors')
        self.enable_motors_future = None

        self.pose = Pose()
        self.get_logger().info(str(self.pose))
        self.motors_enabled = False

        self.MAX_POS_ERROR = 0.01
        self.MAX_ROT_ERROR = 0.02

        self.shouldStop = False
        self.last_tick = time.time_ns()

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing waypoint mission")
        self.shouldStop = False

        self.send_enable_motors(True)

        for pose in goal_handle.request.poses:
            self.drive_to_pose(pose)
            self.send_feedback(goal_handle)

        self.send_enable_motors(False)

        self.get_logger().info("Finished command with end pose: " + str(self.pose))
        goal_handle.succeed()
        result = Waypoints.Result()
        result.final_pose = self.pose
        return result

    def send_feedback(self, goal_handle):
        feedback_msg = Waypoints.Feedback()
        feedback_msg.completed_pose = self.pose
        goal_handle.publish_feedback(feedback_msg)

    def send_enable_motors(self, value):
        request = EnableMotors.Request()
        request.enable = value
        response = self.enable_motors_client.call(request)
        self.motors_enabled = response.enabled

    def drive_to_pose(self, pose: Pose):
        while not self.poseReached(pose) and not self.shouldStop:
            direction, velocity, rotation = self.calculateControllerValue(pose)
            self.sendControllerValue(direction, velocity, rotation)
            time.sleep(0.05)

    def poseReached(self, pose):
        return self.distanceTo(pose) <= self.MAX_POS_ERROR and abs(self.getRotDistance(pose)) <= self.MAX_ROT_ERROR

    def calculateControllerValue(self, pose: Pose):
        dx = pose.x - self.pose.x
        dy = pose.y - self.pose.y
        drot = self.getRotDistance(pose)
        dist = np.sqrt(dx**2 + dy**2)
        direction = to_polar(dx, dy)[0] - math.pi / 2 - self.pose.rot
        if self.distanceTo(pose) > self.MAX_POS_ERROR:
            velocity = 1 #if dist > 0.2 else 5 * dist
        else:
            velocity = 0
        if self.getRotDistance(pose) > self.MAX_ROT_ERROR:
            rotation = - (drot/abs(drot) if abs(drot) > 0.5 else 2*drot)
        else:
            rotation = 0
        return direction, velocity, rotation

    def distanceTo(self, pose: Pose):
        return np.sqrt((pose.x - self.pose.x)**2 + (pose.y - self.pose.y)**2)

    def getRotDistance(self, pose):
        drot = (pose.rot - self.pose.rot if pose.rot > self.pose.rot else self.pose.rot - pose.rot) % (2 * math.pi)
        return (2 * math.pi - drot) if drot > math.pi else drot

    def angularOffset(self, pose: Pose):
        return abs(pose.rot - self.pose.rot) % (2 * math.pi)

    def sendControllerValue(self, direction, velocity, rotation):
        msg = ControllerValue()
        msg.direction = float(direction)
        msg.velocity = float(velocity)
        msg.rotation = float(rotation)
        self.publisher_.publish(msg)


class PathExecutorPoseSubscriber(Node):

    def __init__(self, path_executor):
        super().__init__('path_executor_pose_subscriber')
        self.pose_subscription = self.create_subscription(Pose, 'omniwheel_pose', self.pose_callback, 10)
        self.path_executor = path_executor

    def pose_callback(self, msg):
        self.path_executor.pose = msg


def main(args=None):
    rclpy.init(args=args)

    path_executor = PathExecutor()
    executor_pose_subscriber = PathExecutorPoseSubscriber(path_executor)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(path_executor)
    executor.add_node(executor_pose_subscriber)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        path_executor.destroy_node()
        executor_pose_subscriber.destroy_node()


if __name__ == '__main__':
    main()
