import os

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import numpy as np
import math

from omniwheel.path_visualizer.domain.pose import Pose2D


class Renderer:
    """
    Pygame render class for the position, the path and data of a robot object.
    """

    def __init__(self, width, height):
        pygame.init()
        pygame.display.init()
        self.WIDTH = width
        self.HEIGHT = height
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%i,%i' % (1920 - 1405, 80)
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Omniwheel")
        self.screen.fill((100, 100, 100))
        self.font = pygame.font.Font('freesansbold.ttf', 16)
        self.big_font = pygame.font.Font('freesansbold.ttf', 20)
        self.image = pygame.image.load("src/omniwheel/omniwheel/assets/omniwheel.png")

        self.WAYPOINT_COLOR = (0, 255, 0)  # Color in which the waypoint lines are rendered
        self.PATH_COLOR = (0, 0, 255)  # Color in which the path is rendered

        self.MAP_SCALE = 100  # Pixels per meter
        self.display_offset = np.array([self.WIDTH / 2.2, self.HEIGHT / 2])  # Position of the coordinate origin

    def render(self, robot):
        """
        Render the data of the given robot object.
        Renders everything newly, not just the changes.
        """
        self.screen.fill((100, 100, 100))
        self.draw_path(robot.past_poses)
        self.draw_waypoints(robot.pose, robot.planned_poses)
        self.render_robot(robot.pose)
        self.render_stats(robot)
        pygame.display.flip()

    def reset_camera(self):
        """ Reset the visualization scale of the map and the coordinate origin. """
        self.MAP_SCALE = 100
        self.display_offset = np.array([self.WIDTH / 2.2, self.HEIGHT / 2])

    def draw_path(self, past_poses: [Pose2D]):
        """
        Draws a line between the past_poses in their order in the list.
        The first pose is always set as the origin.
        """
        past_poses[0] = Pose2D(0, 0, 0)
        for i, value in enumerate(past_poses):
            if i < len(past_poses) - 1:
                start = self.pose_to_pixel(value)
                end = self.pose_to_pixel(past_poses[i + 1])
                pygame.draw.line(self.screen, self.PATH_COLOR, start, end)

    def draw_waypoints(self, robot_pose: Pose2D, planned_poses: [Pose2D]):
        """
        Draws a line between the planned_poses in their order in the list. The line always starts at the robots
        current position.
        """
        for i, pose in enumerate(planned_poses):
            start = self.pose_to_pixel(robot_pose) if i == 0 else self.pose_to_pixel(planned_poses[i - 1])
            end = self.pose_to_pixel(pose)
            pygame.draw.line(self.screen, self.WAYPOINT_COLOR, start, end)

    def render_robot(self, pose: Pose2D):
        """ Draws the image representing the robot in the robots position."""
        self.blit_rotate_center(self.image, self.pose_to_pixel(pose), pose.rot)

    def render_stats(self, robot):
        """
        Render various stats about the robot.
        This area is visually separated by a line.
        """
        pygame.draw.line(self.screen, (255, 255, 255), (self.WIDTH * 0.8, 0), (self.WIDTH * 0.8, self.HEIGHT))
        self.render_motor_state(robot)
        self.render_position(robot)
        self.render_battery_voltage(robot)
        self.render_micro_steps(robot)
        self.render_max_wheel_velocity(robot)
        self.render_max_wheel_acceleration(robot)

    def render_motor_state(self, robot):
        self.draw_text('Motors Enabled' if robot.motors_enabled else 'Motors Disabled',
                       (0, 255, 255) if robot.motors_enabled else (255, 0, 0), (self.WIDTH * 0.85, self.HEIGHT * 0.01))

    def render_position(self, robot):
        self.draw_text('Position: ' + str(round(robot.pose.x, 2)) + "x " + str(round(robot.pose.y, 2)) + "y "
                       '(' + str(round(robot.pose.rot * 180 / math.pi, 1)) + "°)",
                       (255, 255, 255), (self.WIDTH * 0.81, self.HEIGHT * 0.05))

    def render_battery_voltage(self, robot):
        self.draw_text('Battery Voltage: ' + str(round(robot.battery_voltage, 2)),
                       (255, 255, 255), (self.WIDTH * 0.81, self.HEIGHT * 0.08))

    def render_micro_steps(self, robot):
        self.draw_text('Micro steps: ' + str(robot.micro_steps), (255, 255, 255),
                       (self.WIDTH * 0.81, self.HEIGHT * 0.11))

    def render_max_wheel_velocity(self, robot):
        self.draw_text('Max wheel velocity: ' + str(round(robot.max_wheel_velocity, 2)), (255, 255, 255),
                       (self.WIDTH * 0.81, self.HEIGHT * 0.14))

    def render_max_wheel_acceleration(self, robot):
        self.draw_text('Max wheel acceleration: ' + str(round(robot.max_wheel_acceleration, 2)), (255, 255, 255),
                       (self.WIDTH * 0.81, self.HEIGHT * 0.17))

    def render_current_velocities(self, robot):
        self.draw_text('Velocities: ' + str(round(robot.twist.x, 2)) + "x " + str(round(robot.twist.y, 2)) + "y "
                       '(' + str(round(robot.twist.rot * 180 / math.pi, 1)) + "°)",
                       (255, 255, 255), (self.WIDTH * 0.81, self.HEIGHT * 0.05))

    def draw_text(self, text, color, position):
        """
        Renders the given text on the screen at the given position and with the given text color.
        """
        text = self.font.render(text, True, color, (100, 100, 100))
        text_rect = text.get_rect()
        text_rect.x, text_rect.y = position
        self.screen.blit(text, text_rect)

    def blit_rotate_center(self, image, center, angle):
        """
        Rotate the image around its center and position it with its center at the given center position.
        Blit it onto the screen.
        """
        rotated_image = pygame.transform.rotate(image, angle * 180 / math.pi)
        new_rect = rotated_image.get_rect(center=image.get_rect(center=center).center)
        self.screen.blit(rotated_image, new_rect)

    def pose_to_pixel(self, pose: Pose2D):
        """ Converts a pose with real world coordinates to the corresponding pixel values"""
        return np.array([pose.x, -pose.y]) * self.MAP_SCALE + self.display_offset

    def pixel_to_pose(self, pixel_pos):
        """ Converts a pixel value to the corresponding real world coordinates. """
        pos = (np.array([pixel_pos[0], pixel_pos[1]]) - self.display_offset) / self.MAP_SCALE
        return np.array((pos[0], -pos[1]))

    def change_zoom(self, y):
        """ Change the zoom level by the sign of the given number. """
        self.MAP_SCALE *= 2 if y > 0 else 0.5
