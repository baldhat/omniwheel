import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import time
import numpy as np
import math
import cmath

from omniwheel_interfaces.action import Waypoints
from omniwheel_interfaces.msg import Pose, ControllerValue
from omniwheel_interfaces.srv import EnableMotors

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
        self.publisher_ = self.create_publisher(ControllerValue, 'controller_value', 10)
        self.enable_motors_client = self.create_client(EnableMotors, 'enable_motors')
        self.enable_motors_future = None

        self.pose = Pose()
        self.get_logger().info(str(self.pose))
        self.motors_enabled = False

        self.MAX_POS_ERROR = 0.01
        self.MAX_ROT_ERROR = 0.01

        self.shouldStop = False

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing waypoint mission")

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
        return self.distanceTo(pose) < self.MAX_POS_ERROR and self.angularOffset(pose) < self.MAX_ROT_ERROR

    def calculateControllerValue(self, pose: Pose):
        dx = pose.x - self.pose.x
        dy = pose.y - self.pose.y
        drot = pose.rot - self.pose.rot
        dist = np.sqrt(dx**2 + dy**2)
        if dist > self.MAX_POS_ERROR:
            direction = to_polar(dx, dy)[0] - math.pi / 2 - self.pose.rot
            velocity = 1 if dist > 0.5 else 2 * dist
            if velocity < 0.1:
                velocity = 0.1
            rotation = 0
        else:
            direction = 0
            velocity = 0
            rotation = drot/abs(drot) if abs(drot) > 1 else drot
        return direction, velocity, rotation

    def distanceTo(self, pose: Pose):
        return np.sqrt((pose.x - self.pose.x)**2 + (pose.y - self.pose.y)**2)

    def angularOffset(self, pose: Pose):
        return abs(pose.rot - self.pose.rot) % (2 * math.pi)

    def sendControllerValue(self, direction, velocity, rotation):
        msg = ControllerValue()
        msg.direction = float(direction)
        msg.velocity = float(velocity)
        msg.rotation = float(rotation)
        self.publisher_.publish(msg)

    def checkServiceResponses(self):
        if self.enable_motors_future is not None and self.enable_motors_future.done():
            self.handle_enable_motors_response()


class PathExecutorPoseSubscriber(Node):

    def __init__(self, pathExecutor):
        super().__init__('path_executor_pose_subscriber')
        self.pose_subscription = self.create_subscription(Pose, 'omniwheel_pose', self.pose_callback, 10)
        self.pathExecutor = pathExecutor

    def pose_callback(self, msg):
        self.pathExecutor.pose = msg


def main(args=None):
    rclpy.init(args=args)

    path_executor = PathExecutor()
    executorPoseSubscriber = PathExecutorPoseSubscriber(path_executor)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(path_executor)
    executor.add_node(executorPoseSubscriber)
    try:
        executor.spin()
    except KeyboardInterrupt:
        path_executor.shouldStop = True


if __name__ == '__main__':
    main()