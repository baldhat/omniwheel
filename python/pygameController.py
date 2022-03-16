import pygame
import math
import cmath
import numpy as np

from serial import Serial

DARKGRAY = (100, 100, 100)

R = 0.15
MOTOR_REVS_PER_METER = 28

def send(ser: Serial, string):
    ser.write(string.encode())

def to_polar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    if r > 1:
        r = 1
    t = cmath.polar(x + y * 1j)[1]
    return t, r

class Controller():

    def __init__(self):
        pygame.init()
        pygame.display.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.WIDTH = 1400
        self.HEIGHT = 800
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Omniwheel")
        self.screen.fill(DARKGRAY)
        self.font = pygame.font.Font('freesansbold.ttf', 16)
        self.clock = pygame.time.Clock()
        self.image = pygame.image.load("assets/omniwheel.png")

        self.ser = Serial('/dev/ttyACM0', 4000000)

        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0
        self.position = np.zeros(2)
        self.orientation = 0

        self.motorsEnabled = False
        self.micro_step_options = [1, 2, 4, 8, 16, 32]

        self.micro_steps = self.fetchMicrosteps()
        self.wheel_velocity = self.fetchWheelVelocity()

        self.displayValues()

    def run(self):
        while True:
            self.handleEvents()
            self.handleSerial()
            self.render()

    def handleSerial(self):
        while self.ser.inWaiting():
            line = self.ser.readline().decode()
            if line.strip().startswith("{"):
                line = line[1:len(line) - 1]
                steps = line.split(";")
                self.updatePosition(steps)
            else:
                print(self.ser.readline().decode(), end="")

    def handleEvents(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.handleJoysticks(event)
            elif event.type == pygame.JOYBUTTONDOWN:
                self.handleButtons(event)

    def handleJoysticks(self, event):
        x = self.last_x
        y = self.last_y
        rot = self.last_rot
        if event.axis == 0:  # left joystick left right
            self.last_x = event.value if abs(event.value) > 0.2 else 0
        if event.axis == 1:  # left joystick up and down
            self.last_y = - event.value if abs(event.value) > 0.2 else 0
        if event.axis == 3:  # right joystick left right
            self.last_rot = - event.value if abs(event.value) > 0.2 else 0
        if event.axis == 4:  # right joystick up down
            pass
        if self.last_x != x or self.last_y != y or self.last_rot != rot:
            self.update()

    def handleButtons(self, event):
        if event.button == 0:
            self.enableMotors()
        elif event.button == 3:
            self.disableMotors()
        elif event.button == 4:
            self.decreaseVelocity()
        elif event.button == 5:
            self.increaseVelocity()
        elif event.button == 6:
            self.decreaseMicrosteps()
        elif event.button == 7:
            self.increaseMicrosteps()
        else:
            print(event.button)

    def update(self):
        new_direction, velocity = to_polar(self.last_x, self.last_y)
        rotation = self.last_rot
        new_direction = new_direction - math.pi / 2 # the robot has 0 degrees at the front

        if self.motorsEnabled:
            commandString = '{I;' + str(round(new_direction, 2)) + \
                            ';' + str(round(velocity, 2)) + \
                            ';' + str(round(rotation, 2)) + ';}'
            send(self.ser, commandString)

    def enableMotors(self):
        self.ser.write(b'{I}')
        self.motorsEnabled = True

    def disableMotors(self):
        self.ser.write(b'{E}')
        self.motorsEnabled = False

    def decreaseVelocity(self):
        if not self.motorsEnabled:
            self.wheel_velocity = round(self.wheel_velocity - 0.01, 2)
            self.ser.write(b'{S;' + str(self.wheel_velocity).encode() + b'}')

    def increaseVelocity(self):
        if not self.motorsEnabled:
            self.wheel_velocity = round(self.wheel_velocity + 0.01, 2)
            self.ser.write(b'{S;' + str(self.wheel_velocity).encode() + b'}')

    def decreaseMicrosteps(self):
        if not self.motorsEnabled:
            self.micro_steps = self.micro_step_options[
                (self.micro_step_options.index(self.micro_steps) - 1) % len(self.micro_step_options)]
            self.ser.write(b'{M;' + str(self.micro_steps).encode() + b'}')

    def increaseMicrosteps(self):
        if not self.motorsEnabled:
            self.micro_steps = self.micro_step_options[
                (self.micro_step_options.index(self.micro_steps) + 1) % len(self.micro_step_options)]
            self.ser.write(b'{M;' + str(self.micro_steps).encode() + b'}')

    def fetchMicrosteps(self):
        self.ser.write(b'{m}')
        while not self.ser.inWaiting():
            pass
        micro_steps = int(self.ser.readline())
        return micro_steps

    def fetchWheelVelocity(self):
        self.ser.write(b'{s}')
        while not self.ser.inWaiting():
            pass
        max_wheel_velocity = float(self.ser.readline())
        return max_wheel_velocity

    def updatePosition(self, steps):
        steps = [steps[0], steps[1], steps[2]]
        revs = np.array([int(step_strings.strip().replace("{", "").replace("}", "")) / 200 for step_strings in steps])
        dists = revs / MOTOR_REVS_PER_METER
        va, vb, vc = dists[0], dists[1], dists[2]
        omega = np.sum(dists) / (3 * R)
        alpha = (np.pi / 2 - np.arctan2((np.sqrt(3) * (va - vb)), (va + vb - 2*vc)))
        dist = np.sqrt(((va + vb - 2 * vc) / 3)**2 + (va - vb)**2 / 3)
        self.orientation += omega
        if not np.isnan(alpha):
            dx = dist * np.cos(alpha + np.pi / 2)
            dy = dist * np.sin(alpha + np.pi / 2)
            self.position += np.array([dx, dy])

    def displayValues(self):
        pygame.draw.line(self.screen, (255, 255, 255), (self.WIDTH * 0.8, 0), (self.WIDTH * 0.8, self.HEIGHT))

        microStepsText = self.font.render('MicroSteps: ' + str(self.micro_steps), True, (255, 255, 255), DARKGRAY)
        microStepsTextRect = microStepsText.get_rect()
        microStepsTextRect.x, microStepsTextRect.y = (self.WIDTH * 0.81, self.HEIGHT * 0.05)
        self.screen.blit(microStepsText, microStepsTextRect)

        velocityText = self.font.render('Max Wheel Velocity: ' + str(self.wheel_velocity),
                                        True, (255, 255, 255), DARKGRAY)
        velocityTextRect = microStepsText.get_rect()
        velocityTextRect.x, velocityTextRect.y = (self.WIDTH * 0.81, self.HEIGHT * 0.1)
        self.screen.blit(velocityText, velocityTextRect)

    def render(self):
        self.screen.fill(DARKGRAY)
        self.displayValues()
        self.renderRobot()
        pygame.display.flip()
        self.clock.tick(10)

    def renderRobot(self):
        self.blitRotateCenter(self.image, self.toPixelPos(self.position), self.orientation)

    def toPixelPos(self, position):
        return position * 100 + np.array([self.WIDTH / 2, self.HEIGHT / 2])

    def blitRotateCenter(self, image, center, angle):
        rotated_image = pygame.transform.rotate(image, angle * 180 / math.pi)
        new_rect = rotated_image.get_rect(center=image.get_rect(center=center).center)
        self.screen.blit(rotated_image, new_rect)


if __name__ == '__main__':
    controller = Controller()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\nBye!")