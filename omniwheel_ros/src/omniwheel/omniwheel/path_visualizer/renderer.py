
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import numpy as np
import math

from omniwheel.path_visualizer.domain.pose import Pose

class Renderer:

    def __init__(self):
        pygame.init()
        pygame.display.init()
        self.WIDTH = 1400
        self.HEIGHT = 1000
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%i,%i' % (1920 - 1405, 80)
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Omniwheel")
        self.screen.fill((100, 100, 100))
        self.font = pygame.font.Font('freesansbold.ttf', 16)
        self.big_font = pygame.font.Font('freesansbold.ttf', 20)
        self.image = pygame.image.load("src/omniwheel/omniwheel/assets/omniwheel.png")

        self.WAYPOINT_COLOR = (0, 255, 0)
        self.PATH_COLOR = (0, 0, 255)

        self.MAP_SCALE = 100
        self.display_offset = np.array([self.WIDTH / 2.2, self.HEIGHT / 2])

    def render(self, robot):
        self.screen.fill((100, 100, 100))
        self.draw_path(robot.past_poses)
        self.draw_waypoints(robot.pose, robot.planned_poses)
        self.render_robot(robot.pose)
        self.render_stats(robot)
        pygame.display.flip()

    def draw_path(self, past_poses: [Pose]):
        past_poses[0] = Pose(0, 0, 0)
        for i, value in enumerate(past_poses):
            if i < len(past_poses) - 1:
                start = self.to_pixel_pos(value)
                end = self.to_pixel_pos(past_poses[i + 1])
                pygame.draw.line(self.screen, self.PATH_COLOR, start, end)

    def draw_waypoints(self, robot_pose: Pose, planned_poses: [Pose]):
        for i, pose in enumerate(planned_poses):
            start = self.to_pixel_pos(robot_pose) if i == 0 else self.to_pixel_pos(planned_poses[i - 1])
            end = self.to_pixel_pos(pose)
            pygame.draw.line(self.screen, self.WAYPOINT_COLOR, start, end)

    def render_robot(self, pose: Pose):
        self.blitRotateCenter(self.image, self.to_pixel_pos(pose), pose.rot)

    def render_stats(self, robot):
        self.drawMotorState(robot)
        self.drawPosition(robot)

    def drawPosition(self, robot):
        positionText = self.font.render('Position: ' + str(round(robot.pose.x, 2)) + "x  "
                                        + str(round(robot.pose.y, 2)) + "y "
                                        "(" + str(round(robot.pose.rot * 180 / math.pi, 1)) + "°)",
                                        True, (255, 255, 255), (100, 100, 100))
        positionTextRect = positionText.get_rect()
        positionTextRect.x, positionTextRect.y = (self.WIDTH * 0.81, self.HEIGHT * 0.05)
        self.screen.blit(positionText, positionTextRect)

    def drawMotorState(self, robot):
        pygame.draw.line(self.screen, (255, 255, 255), (self.WIDTH * 0.8, 0), (self.WIDTH * 0.8, self.HEIGHT))
        enabledText = self.big_font.render('Motors Enabled' if robot.motors_enabled else 'Motors Disabled', True,
                                          (0, 255, 255) if robot.motors_enabled else (255, 0, 0), (100, 100, 100))
        enabledTextRect = enabledText.get_rect()
        enabledTextRect.x, enabledTextRect.y = (self.WIDTH * 0.85, self.HEIGHT * 0.01)
        self.screen.blit(enabledText, enabledTextRect)

    def blitRotateCenter(self, image, center, angle):
        rotated_image = pygame.transform.rotate(image, angle * 180 / math.pi)
        new_rect = rotated_image.get_rect(center=image.get_rect(center=center).center)
        self.screen.blit(rotated_image, new_rect)

    def to_pixel_pos(self, pose: Pose):
        return np.array([pose.x, -pose.y]) * self.MAP_SCALE + self.display_offset

    def to_real_pos(self, pixel_pos):
        pos = (np.array([pixel_pos[0], pixel_pos[1]]) - self.display_offset) / self.MAP_SCALE
        return np.array((pos[0], -pos[1]))

    def zoom(self, y):
        self.MAP_SCALE *= 2 if y > 0 else 0.5