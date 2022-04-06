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
    """ The PathExecutor is primarily a ROS ActionServer to execute waypoint missions.

        It also subscribes to the omniwheel_pose topic to receive feedback on the current robot position.
        Before sending controller_values via the publisher, it sends the enable_motors service an enable command, and
        disables the motors afterwards.

        Publishers:
            - controller_value
        Subscribers:
            - wheel_odometry_pose
        Service clients:
            - enable_motors
        Action servers:
            - waypoints
    """

    def __init__(self):
        super().__init__('path_executor')
        self._waypoint_action_server = ActionServer(self, Waypoints, 'waypoints',
                                                    self.execute_callback, cancel_callback=self.cancel_callback)
        self.publisher_ = self.create_publisher(ControllerValue, 'controller_value', 10)
        self.enable_motors_client = self.create_client(EnableMotors, 'enable_motors')
        self.pose_subscription = self.create_subscription(Pose, 'wheel_odometry_pose', self.pose_callback, 10)

        # The current pose of the robot. This gets updated in self.pose_callback
        self.pose = Pose()

        # Defines the distance in meters at which the executor assumes a waypoint to be reached.
        self.MAX_POS_ERROR = 0.01
        # Defines the angular distance in rad at which the executor assumes a waypoint to be reached
        self.MAX_ROT_ERROR = 0.02

        # If set to high, the executor stops the current action
        self.cancel_action_execution = False

        self.get_logger().info("Ready...")

    def execute_callback(self, goal_handle):
        """ Executes a waypoint mission by iteratively driving to all waypoints and sending feedback after a
            waypoint has been reached.
        """
        self.pre_execute()

        for pose in goal_handle.request.poses:
            self.drive_to_pose(pose)
            if not self.cancel_action_execution:
                self.send_feedback(goal_handle)

        return self.post_execute(goal_handle)

    def pre_execute(self):
        """ Needs to be called before sending controller_values.
            Resets the cancel flag and enables motors.
        """
        self.cancel_action_execution = False
        self.send_enable_motors(True)

    def post_execute(self, goal_handle):
        """Needs to be called when all waypoints have been reached.
            Disables motors and sends the successful
            result to the action client.
        """
        self.send_enable_motors(False)
        goal_handle.succeed()
        result = Waypoints.Result()
        result.final_pose = self.pose
        return result

    def drive_to_pose(self, pose: Pose):
        """Drive towards a goal pose.
            Returns if the pose has been reached or the action should be cancelled.
            This method also calls the ROS executor to handle topics and requests.
            This method BLOCKS the thread to achieve a constant command rate. ~20 HZ
        """
        while not self.poseReached(pose) and not self.cancel_action_execution:
            direction, velocity, rotation = self.calculate_controller_value(pose)
            self.publish_controller_value(direction, velocity, rotation)
            self.executor.spin_once(timeout_sec=0.0)
            time.sleep(0.05)

    def poseReached(self, pose):
        """ Determine whether the pose has been reached by the robot.
            To reach a pose, the robot has to be close to the position and rotation of the waypoint.
        """
        return self.eucledian_distance(pose) <= self.MAX_POS_ERROR and abs(self.rotational_distance(pose)) <= self.MAX_ROT_ERROR

    def calculate_controller_value(self, pose: Pose):
        """ Returns the direction, velocity and rotational velocity to reach the goal pose.
            Direction is the relative angle in radians.
            Velocity is a float between 0 and 1. Rotation is a float between -1 and 1.
        """
        direction, drot, dist = self.calculate_distances(pose)
        velocity = self.calculate_velocity(dist, pose)
        rotation = self.calculate_rotation(drot, pose)
        return direction, velocity, rotation

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
            velocity = 1 if dist > 0.1 else 10 * dist
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

    def eucledian_distance(self, pose: Pose):
        """ Returns the eucledian distance (in meters) between the robots pose and the target pose.
        """
        return np.sqrt((pose.x - self.pose.x)**2 + (pose.y - self.pose.y)**2)

    def rotational_distance(self, pose):
        """ Returns the smallest rotation (in radians) towards the target pose.
        """
        drot = (pose.rot - self.pose.rot if pose.rot > self.pose.rot else self.pose.rot - pose.rot) % (2 * math.pi)
        return (2 * math.pi - drot) if drot > math.pi else drot

    def publish_controller_value(self, direction, velocity, rotation):
        """ Publishes the direction, velocity and rotational_velocity to the controller_value topic.
        """
        msg = ControllerValue()
        msg.direction, msg.velocity, msg.rotation = float(direction), float(velocity), float(rotation)
        self.publisher_.publish(msg)

    def cancel_callback(self, _):
        """ This method gets called to determine whether an action should be cancelled.
            Return code 1 represents NO_CANCEL.
            Return code 2 represents CANCEL.
            We use this method to also set the cancel_actino_execution flag to True
        """
        self.cancel_action_execution = True
        return 2

    def send_feedback(self, goal_handle):
        """ Send the feedback message to the action client.
        """
        feedback_msg = Waypoints.Feedback()
        feedback_msg.completed_pose = self.pose
        goal_handle.publish_feedback(feedback_msg)

    def send_enable_motors(self, value):
        """ Enables the motors by calling the corresponding service.
            This method blocks until the service is available and returned a response.
        """
        request = EnableMotors.Request()
        request.enable = value
        self.enable_motors_client.call(request)  # synchronous call, blocks if service is unavailable

    def pose_callback(self, msg):
        self.pose = msg


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    path_executor = PathExecutor()
    executor.add_node(path_executor)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        path_executor.destroy_node()


if __name__ == '__main__':
    main()
