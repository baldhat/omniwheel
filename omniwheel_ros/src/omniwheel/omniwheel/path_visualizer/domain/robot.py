from omniwheel.path_visualizer.domain.pose import Pose

from rclpy.action import ActionClient

from omniwheel_interfaces.msg import Pose as PoseMsg, MotorState
from omniwheel_interfaces.srv import EnableMotors, SetPose
from omniwheel_interfaces.action import Waypoints


class Robot:
    def __init__(self, node):
        self.node = node
        node.create_subscription(PoseMsg, 'omniwheel_pose', self.pose_update, 10)
        self.enable_motors_client = node.create_client(EnableMotors, 'enable_motors')
        self.position_client = node.create_client(SetPose, 'set_position')
        self.waypoint_client = ActionClient(node, Waypoints, 'waypoints')
        node.create_subscription(MotorState, 'motor_state', self.motor_state_callback, 10)

        self.pose = Pose(0, 0, 0)
        self.motors_enabled = False
        self.past_poses = [Pose(0, 0, 0)]
        self.planned_poses: [Pose] = []

    def set_pose(self, x, y, rot):
        self.pose = Pose(x, y, rot)
        self.past_poses.append(self.pose)

    def pose_update(self, msg):
        self.set_pose(msg.x, msg.y, msg.rot)

    def switch_motor_enabled(self):
        request = EnableMotors.Request()
        request.enable = not self.motors_enabled
        enable_motors_future = self.enable_motors_client.call_async(request)
        enable_motors_future.add_done_callback(self.handle_enable_motors_response)

    def resetPosition(self):
        request = SetPose.Request()
        request.pose.x, request.pose.y, request.pose.rot = 0.0, 0.0, 0.0
        set_position_future = self.position_client.call_async(request)
        set_position_future.add_done_callback(self.handle_set_position_response)

    def add_waypoint(self, pos, send):
        x, y = pos
        orientation = self.pose.rot
        new_pose = PoseMsg()
        new_pose.x, new_pose.y, new_pose.rot = float(x), float(y), float(orientation)
        self.planned_poses.append(Pose(x, y, orientation))
        if send:
            self.send_planned_waypoints()

    def send_planned_waypoints(self):
        self.node.get_logger().info(str(self.planned_poses))
        if len(self.planned_poses) > 0:
            goal = Waypoints.Goal()
            goal.poses = []
            for pose in self.planned_poses:
                pose_msg = PoseMsg()
                pose_msg.x, pose_msg.y, pose_msg.rot = float(pose.x), float(pose.y), float(pose.rot)
                goal.poses.append(pose_msg)
            self.waypoint_client.wait_for_server()
            send_waypoints_future = self.waypoint_client.send_goal_async(goal,
                                                                         feedback_callback=self.waypoints_feedback_callback)
            send_waypoints_future.add_done_callback(self.waypoints_goal_response_callback)

    def waypoints_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected')
            self.planned_poses = []
            return

        self.node.get_logger().info('Goal accepted')
        waypoints_result_future = goal_handle.get_result_async()
        waypoints_result_future.add_done_callback(self.waypoints_result_callback)

    def waypoints_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info('Finished waypoint missiong on pose: ' + str(result.final_pose))
        self.planned_poses = []

    def waypoints_feedback_callback(self, feedback):
        self.node.get_logger().info('Reached waypoint: ' + str(feedback.feedback.completed_pose))
        self.planned_poses.pop(0)

    def motor_state_callback(self, msg):
        self.motors_enabled = msg.enabled
        self.node.get_logger().info('Motors Enabled' if self.motors_enabled else 'Motors Disabled')

    def handle_enable_motors_response(self, future):
        try:
            response = future.result()
            self.motors_enabled = response.enabled
        except Exception as e:
            self.node.get_logger().info('Enable Motors Service call failed %r' % (e,))
        else:
            self.node.get_logger().info('Motors Enabled' if response.enabled else 'Motors Disabled')

    def handle_set_position_response(self, future):
        try:
            response = future.result()
            self.past_poses = []
            self.set_pose(response.pose.x, response.pose.y, response.pose.rot)
        except Exception as e:
            self.node.get_logger().info('Set Position Service call failed %r' % (e,))
        else:
            self.node.get_logger().info('Reset Position')
