import time
import pygame
import math
import numpy as np

from omniwheel_interfaces.msg import ControllerValue
from omniwheel.helper.helper import to_polar


class EventHandler:
    """
    Responsible for listening to pygame events and acting to them.

    Publishers:
        - controller_value
    """

    def __init__(self, node, robot, renderer, on_exit):
        self.node = node
        self.robot = robot
        self.renderer = renderer
        self.on_exit = on_exit
        self.controller_publisher = self.node.create_publisher(ControllerValue, 'controller_value', 10)

        # These lists store tuples assigning handlers to their corresponding events
        self.key_event_handlers = []
        self.mouse_event_handlers = []
        self.mouse_button_event_handlers = []
        self.register_event_handlers()

        # The basis for the previously sent controller values.
        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0
        # The current basis for the controller values. These variables get set by key_presses and key_releases
        # and based on them the controller_value is calculated.
        self.x = 0
        self.y = 0
        self.rot = 0
        # Time of the last controller value update in seconds (float)
        self.last_update = 0.0

        # Different events have different attributes, but we can handle some the same way
        self.KEY_EVENT_TYPES = [pygame.KEYUP, pygame.KEYDOWN]
        self.MOUSE_EVENT_TYPES = [pygame.MOUSEWHEEL, pygame.MOUSEMOTION]
        self.MOUSE_BUTTON_EVENT_TYPES = [pygame.MOUSEBUTTONUP, pygame.MOUSEBUTTONDOWN]
        self.move_keys = [pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d, pygame.K_q, pygame.K_e]

        # Remember the state of some keys/buttons
        self.shift_down = False
        self.middle_mouse_down = False

    def register_event_handlers(self):
        """
        Register the handlers for the events.
        """
        self.register_key_event_handlers()
        self.register_mouse_event_handlers()
        self.register_mouse_button_event_handlers()

    def register_key_event_handlers(self):
        """
        Registers all handlers for the key up and key down events.
        """
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_q, lambda: self.set_rot(1)))
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_e, lambda: self.set_rot(-1)))
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_w, lambda: self.set_y(1)))
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_s, lambda: self.set_y(-1)))
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_a, lambda: self.set_x(-1)))
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_d, lambda: self.set_x(1)))
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_SPACE, self.robot.switch_motor_enabled))
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_c, self.robot.reset_position))
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_0, self.renderer.reset_camera))
        self.key_event_handlers.append((pygame.KEYDOWN, pygame.K_ESCAPE, self.robot.cancel_waypoint_mission))

        self.key_event_handlers.append((pygame.KEYUP, pygame.K_q, lambda: self.set_rot(0)))
        self.key_event_handlers.append((pygame.KEYUP, pygame.K_e, lambda: self.set_rot(0)))
        self.key_event_handlers.append((pygame.KEYUP, pygame.K_w, lambda: self.set_y(0)))
        self.key_event_handlers.append((pygame.KEYUP, pygame.K_s, lambda: self.set_y(0)))
        self.key_event_handlers.append((pygame.KEYUP, pygame.K_a, lambda: self.set_x(0)))
        self.key_event_handlers.append((pygame.KEYUP, pygame.K_d, lambda: self.set_x(0)))
        self.key_event_handlers.append((pygame.KEYUP, pygame.K_LSHIFT, self.robot.send_planned_waypoints))

    def register_mouse_event_handlers(self):
        """
        Register all handlers for mouse motion and mouse wheel events.
        """
        self.mouse_event_handlers.append((pygame.MOUSEMOTION, self.handle_mouse_motion))
        self.mouse_event_handlers.append((pygame.MOUSEWHEEL, self.handle_mouse_wheel))

    def register_mouse_button_event_handlers(self):
        """
        Register all handlers for mouse button events. (clicks)
        """
        self.mouse_button_event_handlers.append((
            pygame.MOUSEBUTTONUP, 2, lambda event: self.set_middle_mouse_down(False)))
        self.mouse_button_event_handlers.append((
            pygame.MOUSEBUTTONDOWN, 2, lambda event: self.set_middle_mouse_down(True)))
        self.mouse_button_event_handlers.append((
            pygame.MOUSEBUTTONDOWN, 1,
             lambda event: self.robot.add_waypoint(self.renderer.pixel_to_pose(event.pos),
                                                   not pygame.key.get_pressed()[pygame.K_LSHIFT])
        ))

    def should_update_controller_value(self):
        """
        Return whether the controller_value publisher should send an update.
        This can happen for two reasons:
        1. The controller inputs changed (aka a key got released or pressed)
        2. There hasn't been a value sent for the last 50ms. The teensy_node requires a command to be sent at least
           every 100ms, so we sent one every 50ms.
        """
        return self.has_controller_value_changed() or (
                time.time() - self.last_update > 0.05 and (self.rot != 0 or self.x != 0 or self.y != 0))

    def has_controller_value_changed(self):
        """ Determine whether an input has changed since the last sent command. """
        return self.last_x != self.x or self.last_y != self.y or self.last_rot != self.rot

    def handle_events(self):
        """
        Handle all the pygame events. Even if there were none, check if the controller_values need to be sent.
        """
        for event in pygame.event.get():
            self.handle_event_by_type(event)
        if self.should_update_controller_value():
            self.update()

    def handle_event_by_type(self, event):
        """
        Handle each event according to its event type.
        If it is a QUIT event, call the on_exit handler.
        """
        if event.type in self.KEY_EVENT_TYPES:
            self.handle_key_event(event)
        if event.type in self.MOUSE_EVENT_TYPES:
            self.handle_mouse_event(event)
        if event.type in self.MOUSE_BUTTON_EVENT_TYPES:
            self.handle_mouse_button_event(event)
        if event.type == pygame.QUIT:
            self.on_exit()

    def handle_key_event(self, event):
        """
        Handles key press or key release events by calling the corresponding handler.
        """
        for type_, key, handler in self.key_event_handlers:
            if key == event.key and type_ == event.type:
                handler()

    def handle_mouse_event(self, event):
        """
        Handles key mouse motion or mouse wheel events by calling the corresponding handler.
        """
        for type_, handler in self.mouse_event_handlers:
            if type_ == event.type:
                handler(event)

    def handle_mouse_button_event(self, event):
        """
        Handles mouse button click or release events by calling the corresponding handler.
        """
        for type_, button, handler in self.mouse_button_event_handlers:
            if type_ == event.type and button == event.button:
                handler(event)

    def handle_mouse_wheel(self, event):
        """
        Change the zoom level of the renderer according to the movement of the mouse wheel.
        """
        self.renderer.change_zoom(event.y)

    def handle_mouse_motion(self, event):
        """
        Handle the mouse motion event, by panning the map, if the middle_mouse_button is pressed.
        """
        if self.middle_mouse_down:
            self.set_display_offset(event)

    def set_rot(self, value):
        self.rot = value

    def set_x(self, value):
        self.x = value

    def set_y(self, value):
        self.y = value

    def set_middle_mouse_down(self, value):
        self.middle_mouse_down = value

    def set_display_offset(self, event):
        """
        Pan the map, by changing the renderers display_offset according to the motion of the mouse event.
        """
        self.renderer.display_offset += np.array((event.rel[0], event.rel[1]))

    def update(self):
        """
        Update the current controller_value by calculating it from the currently pressed keys.
        Then send the new controller value and update the last_-values.
        """
        new_direction, rotation, velocity = self.calculate_controller_value()
        self.publisher_controller_value(new_direction, rotation, velocity)
        self.last_update = time.time()
        self.set_last_values()

    def set_last_values(self):
        """
        Update the last_-values by setting the current values to be the last values.
        """
        self.last_x = self.x
        self.last_y = self.y
        self.last_rot = self.rot

    def calculate_controller_value(self):
        """
        Calculate the correct controller values from the keyboard inputs.
        last_x and last_y get converted to polar cooardinates. The velocity is the radius, the direction the angle
        (plus an offset of pi because ofthe way the coordinate frame of the robot is laid out)
        """
        new_direction, velocity = to_polar(self.last_x, self.last_y)
        rotation = self.last_rot
        new_direction = new_direction - math.pi / 2  # the robot has 0 degrees at the front
        return new_direction, rotation, velocity

    def publisher_controller_value(self, new_direction, rotation, velocity):
        """
        Create and send a controller_value message.
        """
        msg = ControllerValue()
        msg.direction, msg.max_velocity, msg.rotation = float(new_direction), float(velocity), float(rotation)
        self.controller_publisher.publish(msg)
