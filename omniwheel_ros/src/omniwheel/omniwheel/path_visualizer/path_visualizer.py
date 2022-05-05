import rclpy
from rclpy.node import Node
import pygame

from omniwheel.path_visualizer.renderer import Renderer
from omniwheel.path_visualizer.domain.robot import Robot
from omniwheel.path_visualizer.event_handler import EventHandler


class PathVisualizer(Node):
    """
    Visualizes the omniwheel robot with the help of a renderer and an event handler.
    """
    def __init__(self):
        super().__init__('path_visualizer')

        self.robot = Robot(self)
        self.renderer = Renderer(1400, 1000)
        self.event_handler = EventHandler(self, self.robot, self.renderer, self.stop)
        self.loop_timer = self.create_timer(0.05, self.timer_callback)

        self.clock = pygame.time.Clock()
        self.running = True

    def timer_callback(self):
        """
        As long as the stop handler hasn't been called, spin the ros node, render the robot data and position and
        check for events.
        The frame- and update-rate is locked to 20Hz.
        """
        if self.running:
            self.renderer.render(self.robot)
            self.event_handler.handle_events()

    def stop(self):
        """ Callback for the quit-event of the event handler. """
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    path_visualizer = PathVisualizer()
    rclpy.spin(path_visualizer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
