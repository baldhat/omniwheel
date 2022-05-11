import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from omniwheel.domain.pose import Pose2D

import time
import threading
import numpy as np
import math
import tf_transformations

from omniwheel.helper.helper import to_polar

from omniwheel_interfaces.action import Waypoints
from omniwheel_interfaces.msg import ControllerValue, Pose as PoseMsg
from omniwheel_interfaces.srv import EnableMotors
from nav_msgs.msg import Odometry


class PathExecutor(Node):
    """ The PathExecutor is a ROS ActionServer to execute waypoint missions.

        It works in union with the WatchDog node, which subscribes to the robot odometry and publishes the controller
        values.

        Service clients:
            - /enable_motors
        Action servers:
            - /waypoints
    """

    def __init__(self):
        super().__init__('path_executor')
        self._waypoint_action_server = ActionServer(self, Waypoints, '/waypoints',
                                                    execute_callback=self.execute_callback,
                                                    cancel_callback=self.cancel_callback,
                                                    goal_callback=self.goal_callback,
                                                    handle_accepted_callback=self.handle_accepted_callback,
                                                    callback_group=ReentrantCallbackGroup())
        self.enable_motors_client = self.create_client(EnableMotors, '/enable_motors')

        # The current pose of the robot. This gets updated in self.pose_callback
        self.pose = Pose2D(0, 0, 0)

        # Defines the distance in meters at which the executor assumes a waypoint to be reached.
        self.MAX_POS_ERROR = 0.02
        # Defines the angular distance in rad at which the executor assumes a waypoint to be reached
        self.MAX_ROT_ERROR = 0.02

        # If set to True, the executor stops the current action
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.rate = self.create_rate(20)

        self.is_executing_goal = False

        self.velocity, self.direction, self.rotation = 0.0, 0.0, 0.0

        self.get_logger().info("Ready...")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """ Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """ Executes a waypoint mission by iteratively driving to all waypoints and sending feedback after a
            waypoint has been reached.
        """
        self.start_execution()
        poses = [self.pose_msg_to_pose2d(pose_msg) for pose_msg in goal_handle.request.poses]

        for pose in poses:
            self.drive_to_pose(pose, goal_handle)
            if not goal_handle.is_active:
                self.stop_execution()
                return Waypoints.Result()

            if goal_handle.is_cancel_requested:
                self.stop_execution()
                goal_handle.canceled()
                return Waypoints.Result()

            self.send_feedback(goal_handle)
        return self.post_execute(goal_handle)

    def stop_execution(self):
        """
        When finishing a goal, disable motors and let the WatchDog know we stopped.
        """
        self.send_enable_motors(False)
        self.is_executing_goal = False

    def start_execution(self):
        """
        When beginning to execut a goal, enable motors and let the WatchDog know we started.
        """
        self.send_enable_motors(True)
        self.is_executing_goal = True

    def post_execute(self, goal_handle: ServerGoalHandle):
        """ Needs to be called when all waypoints have been reached.
            Disables motors and sends the successful result to the action client.
        """
        self.stop_execution()
        result = Waypoints.Result()
        result.final_pose = self.pose_to_msg(self.pose)
        goal_handle.succeed()
        self.get_logger().info("Finished goal")
        return result

    def drive_to_pose(self, pose: Pose2D, goal_handle):
        """ Drive towards a goal pose.
            Returns if the pose has been reached or the action should be cancelled.
            This method also calls the ROS executor to handle topics and requests.
            This method BLOCKS the thread to achieve a constant command rate. ~20 HZ
        """
        while not self.poseReached(pose) and not goal_handle.is_cancel_requested and goal_handle.is_active:
            self.calculate_controller_value(pose)
            self.rate.sleep()

    def poseReached(self, pose):
        """ Determine whether the pose has been reached by the robot.
            To reach a pose, the robot has to be close to the position and rotation of the waypoint.
        """
        return self.eucledian_distance(pose) <= self.MAX_POS_ERROR and abs(self.rotational_distance(pose)) <= self.MAX_ROT_ERROR

    def calculate_controller_value(self, pose: Pose2D):
        """ Returns the direction, velocity and rotational velocity to reach the goal pose.
            Direction is the relative angle in radians.
            Velocity is a float between 0 and 1. Rotation is a float between -1 and 1.
        """
        self.direction, drot, dist = self.calculate_distances(pose)
        self.velocity = self.calculate_velocity(dist, pose)
        self.rotation = self.calculate_rotation(drot, pose)

    def calculate_rotation(self, drot, pose):
        """ Return the relative rotational velocity [-1..1] to reach the goal pose.
            The velocity gets reduced, if the robot is close to the target rotation.
        """
        if self.rotational_distance(pose) > self.MAX_ROT_ERROR:
            rotation = - (drot / abs(drot) if abs(drot) > 0.5 else 2 * drot)
        else:
            rotation = 0
        return rotation

    def calculate_velocity(self, dist, pose):
        """ Return the relative translational velocity [-1..1] to reach the goal pose.
            The velocity gets reduced, if the robot is close to the target rotation.
        """
        if self.eucledian_distance(pose) > self.MAX_POS_ERROR:
            velocity = 1 if dist > 0.1 else 5 * dist
        else:
            velocity = 0
        return velocity

    def calculate_distances(self, pose):
        dx = pose.x - self.pose.x
        dy = pose.y - self.pose.y
        drot = self.rotational_distance(pose)
        dist = np.sqrt(dx ** 2 + dy ** 2)
        direction = to_polar(dx, dy)[0] - math.pi / 2 - self.pose.rot
        return direction, drot, dist

    def eucledian_distance(self, pose: Pose2D):
        """ Returns the eucledian distance (in meters) between the robots pose and the target pose.
        """
        return np.sqrt((pose.x - self.pose.x)**2 + (pose.y - self.pose.y)**2)

    def rotational_distance(self, pose):
        """ Returns the smallest rotation (in radians) towards the target pose.
        """
        drot = (pose.rot - self.pose.rot if pose.rot > self.pose.rot else self.pose.rot - pose.rot) % (2 * math.pi)
        return (2 * math.pi - drot) if drot > math.pi else drot

    def send_feedback(self, goal_handle):
        """ Send the feedback message to the action client.
        """
        feedback_msg = Waypoints.Feedback()
        feedback_msg.completed_pose.x = self.pose.x
        feedback_msg.completed_pose.y = self.pose.y
        feedback_msg.completed_pose.rot = self.pose.rot
        goal_handle.publish_feedback(feedback_msg)

    def send_enable_motors(self, value):
        """ Enables the motors by calling the corresponding service.
            This method blocks until the service is available and returned a response.
        """
        request = EnableMotors.Request()
        request.enable = value
        future = self.enable_motors_client.call_async(request)
        future.add_done_callback(lambda response: self.get_logger().info("Motors disabled"))

    def pose_msg_to_pose2d(self, pose_msg: PoseMsg):
        return Pose2D(pose_msg.x, pose_msg.y, pose_msg.rot)

    def pose_to_msg(self, pose: Pose2D):
        msg = PoseMsg()
        msg.x, msg.y, msg.rot = pose.x, pose.y, pose.rot
        return msg


class WatchDog(Node):
    """
    Handle subscriptions and publishing for the Action Server

    Publishers:
        - /controller_value
    Subscribers:
        - /odom
    """
    def __init__(self, action_server):
        super().__init__('path_executor')
        self.action_server = action_server

        self.publisher_ = self.create_publisher(ControllerValue, '/controller_value', 10)
        self.pose_subscription = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def pose_callback(self, msg):
        """
        Update the current pose of the robot in the action server object
        """
        x, y, z, w = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, \
                     msg.pose.pose.orientation.w
        euler = tf_transformations.euler_from_quaternion([x, y, z, w])
        self.action_server.pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])

    def timer_callback(self):
        """
        If the action server is currently executing a goal, publish the generated controller values to the topic
        """
        if self.action_server.is_executing_goal:
            self.publish_controller_values()

    def publish_controller_values(self):
        """ Publishes the direction, velocity and rotational_velocity to the controller_value topic.
        """
        msg = ControllerValue()
        msg.direction, msg.velocity, msg.rotation = float(self.action_server.direction), \
                                                    float(self.action_server.velocity),\
                                                    float(self.action_server.rotation)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    action_server = PathExecutor()
    watchdog = WatchDog(action_server)

    executor.add_node(action_server)
    executor.add_node(watchdog)

    try:
        rclpy.spin(action_server, executor=executor)
    finally:
        executor.shutdown()
        action_server.destroy_node()


if __name__ == '__main__':
    main()
