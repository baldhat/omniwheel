import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import math
import cmath
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from omniwheel_interfaces.msg import ControllerValue, Pose
from omniwheel_interfaces.srv import EnableMotors, SetPose
from omniwheel_interfaces.action import Waypoints


DARKGRAY = (100, 100, 100)


def to_polar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    if r > 1:
        r = 1
    t = cmath.polar(x + y * 1j)[1]
    return t, r


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.pose_subscription = self.create_subscription(Pose, 'omniwheel_pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(ControllerValue, 'controller_value', 10)
        self.enable_motors_client = self.create_client(EnableMotors, 'enable_motors')
        self.enable_motors_future = None
        self.position_client = self.create_client(SetPose, 'set_position')
        self.set_position_future = None
        self.waypoint_client = ActionClient(self, Waypoints, 'waypoints')
        self.send_waypoints_future = None
        self.waypoints_result_future = None

        pygame.init()
        pygame.display.init()
        self.WIDTH = 1400
        self.HEIGHT = 1000
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%i,%i' % (1920 - 1405, 80)
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Omniwheel")
        self.screen.fill(DARKGRAY)
        self.font = pygame.font.Font('freesansbold.ttf', 16)
        self.bigFont = pygame.font.Font('freesansbold.ttf', 20)
        self.clock = pygame.time.Clock()
        self.image = pygame.image.load("src/omniwheel/omniwheel/assets/omniwheel.png")

        self.motors_enabled = False

        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0

        self.position = np.zeros(2)
        self.orientation = 0

        self.MAP_SCALE = 100
        self.past_positions = [(0, 0)]

        self.planned_waypoints: [Pose] = []
        self.shift_down = False

        self.middle_mouse_down = False
        self.display_offset = np.array([self.WIDTH / 2.2, self.HEIGHT / 2])

    def pose_callback(self, msg):
        self.orientation = msg.rot
        self.position = (msg.x, msg.y)
        self.past_positions.append(self.position)

    def render(self):
        self.screen.fill(DARKGRAY)
        self.drawPath()
        self.drawWaypoints()
        self.renderRobot()
        pygame.display.flip()
        self.clock.tick(20)

    def renderRobot(self):
        self.blitRotateCenter(self.image, self.toPixelPos(self.position), self.orientation)

    def toPixelPos(self, position):
        return np.array([position[0], -position[1]]) * self.MAP_SCALE + self.display_offset

    def toRealPos(self, position):
        pos = (np.array([position[0], position[1]]) - self.display_offset) / self.MAP_SCALE
        return np.array((pos[0], -pos[1]))

    def blitRotateCenter(self, image, center, angle):
        rotated_image = pygame.transform.rotate(image, angle * 180 / math.pi)
        new_rect = rotated_image.get_rect(center=image.get_rect(center=center).center)
        self.screen.blit(rotated_image, new_rect)

    def drawPath(self):
        self.past_positions[0] = (0, 0)
        for i, value in enumerate(self.past_positions):
            if i < len(self.past_positions) - 1:
                start = self.toPixelPos(value)
                end = self.toPixelPos(self.past_positions[i + 1])
                pygame.draw.line(self.screen, (0, 0, 255), start, end)

    def drawWaypoints(self):
        for i, pose in enumerate(self.planned_waypoints):
            start = self.toPixelPos(self.position) if i == 0 else self.toPixelPos((self.planned_waypoints[i-1].x, self.planned_waypoints[i-1].y))
            end = self.toPixelPos((pose.x, pose.y))
            pygame.draw.line(self.screen, (0, 255, 0), start, end)

    def resetPosition(self):
        request = SetPose.Request()
        request.pose.x, request.pose.y, request.pose.rot = 0.0, 0.0, 0.0
        self.set_position_future = self.position_client.call_async(request)

    def tick(self):
        self.render()
        self.handle_events()
        self.checkServiceResponses()

    def handle_events(self):
        x, y, rot = self.last_x, self.last_y, self.last_rot
        for event in pygame.event.get():
            if event.type == pygame.KEYUP:
                self.handleKeyUp(event)
            if event.type == pygame.KEYDOWN:
                self.handleKeyDown(event)
            if event.type == pygame.MOUSEWHEEL:
                self.zoom(event.y)
            if event.type == pygame.MOUSEBUTTONDOWN:
                self.handleMouseDown(event)
            if event.type == pygame.MOUSEBUTTONUP:
                self.handleMouseUp(event)
            if event.type == pygame.MOUSEMOTION:
                self.handleMouseMotion(event)
        if self.hasValueChanged(rot, x, y):
            self.update()

    def hasValueChanged(self, rot, x, y):
        return self.last_x != x or self.last_y != y or self.last_rot != rot

    def handleKeyDown(self, event):
        if event.key == pygame.K_q:
            self.last_rot = 1
        if event.key == pygame.K_e:
            self.last_rot = -1
        if event.key == pygame.K_w:
            self.last_y = 1
        if event.key == pygame.K_a:
            self.last_x = -1
        if event.key == pygame.K_s:
            self.last_y = -1
        if event.key == pygame.K_d:
            self.last_x = 1
        if event.key == pygame.K_SPACE:
            self.send_enable_motors(not self.motors_enabled)
        if event.key == pygame.K_c:
            self.resetPosition()

    def handleKeyUp(self, event):
        if event.key == pygame.K_q:
            self.last_rot = 0
        if event.key == pygame.K_e:
            self.last_rot = 0
        if event.key == pygame.K_w:
            self.last_y = 0
        if event.key == pygame.K_a:
            self.last_x = 0
        if event.key == pygame.K_s:
            self.last_y = 0
        if event.key == pygame.K_d:
            self.last_x = 0
        if event.key == pygame.K_LSHIFT:
            self.send_planned_waypoints()

    def handleMouseDown(self, event):
        if event.button == 2: # middle mouse button
            self.middle_mouse_down = True
        if event.button == 1:
            self.handle_left_click(event)

    def handleMouseUp(self, event):
        if event.button == 2: # middle mouse button
            self.middle_mouse_down = False

    def handleMouseMotion(self, event):
        if self.middle_mouse_down:
            self.display_offset += np.array((event.rel[0], event.rel[1]))

    def zoom(self, y):
        self.MAP_SCALE *= 2 if y > 0 else 0.5

    def handle_left_click(self, event):
        x, y = self.toRealPos(event.pos)
        new_pose = Pose()
        new_pose.x, new_pose.y, new_pose.rot = float(x), float(y), float(self.orientation)
        self.planned_waypoints.append(new_pose)
        if not pygame.key.get_pressed()[pygame.K_LSHIFT]:
            self.send_planned_waypoints()

    def update(self):
        new_direction, rotation, velocity = self.calculateControllerValue()
        if self.motors_enabled:
            self.publishControllerValue(new_direction, rotation, velocity)

    def calculateControllerValue(self):
        new_direction, velocity = to_polar(self.last_x, self.last_y)
        rotation = self.last_rot
        new_direction = new_direction - math.pi / 2  # the robot has 0 degrees at the front
        return new_direction, rotation, velocity

    def publishControllerValue(self, new_direction, rotation, velocity):
        msg = ControllerValue()
        msg.direction = float(new_direction)
        msg.velocity = float(velocity)
        msg.rotation = float(rotation)
        self.publisher_.publish(msg)
        self.get_logger().debug('"%f %f %f"' % (msg.direction, msg.velocity, msg.rotation))

    def send_enable_motors(self, value):
        request = EnableMotors.Request()
        request.enable = value
        self.enable_motors_future = self.enable_motors_client.call_async(request)

    def send_planned_waypoints(self):
        if len(self.planned_waypoints) > 0:
            goal = Waypoints.Goal()
            goal.poses = self.planned_waypoints
            self.waypoint_client.wait_for_server()
            self.send_waypoints_future = self.waypoint_client.send_goal_async(goal, feedback_callback=self.waypoints_feedback_callback)
            self.send_waypoints_future.add_done_callback(self.waypoints_goal_response_callback)

    def waypoints_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.planned_waypoints = []
            return

        self.get_logger().info('Goal accepted')
        self.waypoints_result_future = goal_handle.get_result_async()
        self.waypoints_result_future.add_done_callback(self.waypoints_result_callback)

    def waypoints_result_callback(self, future):
        result = future.result()
        self.planned_waypoints = []

    def waypoints_feedback_callback(self, feedback):
        pose = feedback.feedback.completed_pose
        if round(self.planned_waypoints[0].x, 1) == round(pose.x, 1) \
                and round(self.planned_waypoints[0].y, 1) == round(pose.y, 1) \
                and round(self.planned_waypoints[0].rot, 1) == round(pose.rot, 1):
            self.planned_waypoints.pop(0)

    def handle_enable_motors_response(self):
        try:
            response = self.enable_motors_future.result()
            self.motors_enabled = response.enabled
        except Exception as e:
            self.get_logger().info(
                'Enable Motors Service call failed %r' % (e,))
        else:
            self.get_logger().info('Motors Enabled' if response.enabled else 'Motors Disabled')
        self.enable_motors_future = None

    def handle_set_position_response(self):
        try:
            response = self.set_position_future.result()
            self.position = (response.pose.x, response.pose.y)
            self.orientation = response.pose.rot
            self.past_positions = [self.position]
        except Exception as e:
            self.get_logger().info(
                'Set Position Service call failed %r' % (e,))
        else:
            self.get_logger().info('Reset Position')
        self.set_position_future = None



    def checkServiceResponses(self):
        if self.enable_motors_future is not None and self.enable_motors_future.done():
            self.handle_enable_motors_response()
        if self.set_position_future is not None and self.set_position_future.done():
            self.handle_set_position_response()


def main(args=None):
    rclpy.init(args=args)
    path_visualizer = PathVisualizer()

    try:
        while rclpy.ok():
            rclpy.spin_once(path_visualizer, timeout_sec=0)
            path_visualizer.tick()
    except KeyboardInterrupt:
        print('Bye')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
