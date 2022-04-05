from omniwheel.path_visualizer.domain.pose import Pose

from rclpy.action import ActionClient

from omniwheel_interfaces.msg import Pose as PoseMsg, MotorState
from omniwheel_interfaces.srv import EnableMotors, SetPose
from omniwheel_interfaces.action import Waypoints
from sensor_msgs.msg import BatteryState


class Robot:
    """
    This class represents the omniwheel robot towards the path_visualizer.

    Subscribers:
        - omniwheel_pose
        - motor_state
        - battery_state
    Service clients:
        - enable_motors
        - set_position
    Action servers:
        - waypoints
    """

    def __init__(self, node):
        self.node = node
        node.create_subscription(PoseMsg, 'omniwheel_pose', self.pose_update, 10)
        self.enable_motors_client = node.create_client(EnableMotors, 'enable_motors')
        self.position_client = node.create_client(SetPose, 'set_position')
        self.waypoint_client = ActionClient(node, Waypoints, 'waypoints')
        self.waypoint_goal_handle = None  # Used for action cancellation
        node.create_subscription(MotorState, 'motor_state', self.motor_state_callback, 10)
        node.create_subscription(BatteryState, 'battery_state', self.battery_state_callback, 10)

        self.pose = Pose(0, 0, 0)  # The current pose of the robot, updated by omniwheel_pose messages
        self.motors_enabled = False  # Motor state, updated by motor_state messages
        self.past_poses = [Pose(0, 0, 0)]  # List of previous poses of the robot, in order
        self.planned_poses: [Pose] = []  # List of planned poses/waypoints, gets send to the Waypoints-Action-Server
        self.battery_voltage = 0  # Current voltage of the robots batteries, updated by battery_state

    def set_pose(self, x, y, rot):
        """
        Updated the current pose of the robot. The previous pose gets added to the past_poses.
        """
        self.pose = Pose(x, y, rot)
        self.past_poses.append(self.pose)

    def pose_update(self, msg):
        """
        Callback for messages of the omniwheel_pose topic.
        """
        self.set_pose(msg.x, msg.y, msg.rot)

    def switch_motor_enabled(self):
        """
        Change enabled-state of the motors by calling the service. If enabled disable and vice versa.
        """
        request = EnableMotors.Request()
        request.enable = not self.motors_enabled
        enable_motors_future = self.enable_motors_client.call_async(request)
        enable_motors_future.add_done_callback(self.handle_enable_motors_response)

    def reset_position(self):
        """
        Request to reset the position of the robot to (0, 0) and the orientation to (0). This does NOT drive the
        robot, but rather resets the global coordinate system to have its center on the current robot position.
        """
        request = SetPose.Request()
        request.pose.x, request.pose.y, request.pose.rot = 0.0, 0.0, 0.0
        set_position_future = self.position_client.call_async(request)
        set_position_future.add_done_callback(self.handle_set_position_response)

    def add_waypoint(self, pos, send=False):
        """
        Add a new pose to the planned_poses. If the send-flag is set, immediately send them to the action server.
        """
        (x, y), orientation = pos, self.pose.rot
        self.planned_poses.append(Pose(x, y, orientation))
        if send:
            self.send_planned_waypoints()

    def send_planned_waypoints(self):
        """
        Send the queued waypoints in planned_poses to the action server and register the callback.
        """
        if len(self.planned_poses) <= 0:
            return
        goal = self.create_waypoint_goal()
        self.waypoint_client.wait_for_server()  # Blocks until the action server is available
        send_waypoints_future = self.waypoint_client.send_goal_async(
            goal, feedback_callback=self.waypoints_feedback_callback
        )
        send_waypoints_future.add_done_callback(self.waypoints_goal_response_callback)

    def create_waypoint_goal(self):
        """
        Create the action goal for a waypoint action and added the planed poses.
        """
        goal = Waypoints.Goal()
        goal.poses = []
        for pose in self.planned_poses:
            pose_msg = PoseMsg()
            pose_msg.x, pose_msg.y, pose_msg.rot = float(pose.x), float(pose.y), float(pose.rot)
            goal.poses.append(pose_msg)
        return goal

    def waypoints_goal_response_callback(self, future):
        """
        Callback for the waypoint request sent to the action server.
        If the action got rejected, clear the planned poses.
        If the action got accepted, register the result callback.
        """
        self.waypoint_goal_handle = future.result()
        if not self.waypoint_goal_handle.accepted:
            self.node.get_logger().debug('Goal rejected')
            self.planned_poses = []
            self.waypoint_goal_handle = None
            return

        self.node.get_logger().debug('Goal accepted')
        waypoints_result_future = self.waypoint_goal_handle.get_result_async()
        waypoints_result_future.add_done_callback(self.waypoints_result_callback)

    def waypoints_result_callback(self, future):
        """
        Callback for the result of a waypoint action. Clears the planned waypoints.
        """
        result = future.result().result
        self.node.get_logger().debug('Finished waypoint mission on pose: ' + str(result.final_pose))
        self.planned_poses = []
        self.waypoint_goal_handle = None

    def waypoints_feedback_callback(self, feedback):
        """
        Callback for the feedback on a waypoint action. Removes the first element of the planned poses, aka the reached
        waypoint
        """
        self.node.get_logger().debug('Reached waypoint: ' + str(feedback.feedback.completed_pose))
        self.planned_poses.pop(0)

    def cancel_waypoint_mission(self):
        """
        Cancels the current waypoint mission, if one is ongoing, by calling the cancel method on the action server.
        """
        if self.waypoint_goal_handle is None:
            return
        future = self.waypoint_goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _: self.node.get_logger().info("Cancelled"))
        self.planned_poses = []

    def motor_state_callback(self, msg):
        """
        Callback for the motor_state topic messages. Updates the corresponding variable.
        """
        self.motors_enabled = msg.enabled
        self.node.get_logger().info('Motors Enabled' if self.motors_enabled else 'Motors Disabled')

    def battery_state_callback(self, msg):
        """
        Callback for the battery_state topic messages. Updates the corresponding variable.
        """
        self.battery_voltage = msg.voltage

    def handle_enable_motors_response(self, future):
        """
        Callback for the EnableMotors service responses. Sets the corresponding variable to the return value.
        """
        try:
            response = future.result()
            self.motors_enabled = response.enabled
        except Exception as e:
            self.node.get_logger().info('Enable Motors Service call failed %r' % (e,))

    def handle_set_position_response(self, future):
        """
        Callback for the SetPosition service responses. Sets the pose of the robot and clears the past poses.
        """
        try:
            response = future.result()
            self.past_poses = []
            self.set_pose(response.pose.x, response.pose.y, response.pose.rot)
        except Exception as e:
            self.node.get_logger().error('Set Position Service call failed %r' % (e,))
        else:
            self.node.get_logger().debug('Reset Position')
