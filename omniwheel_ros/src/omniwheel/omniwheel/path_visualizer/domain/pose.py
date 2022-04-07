

class Pose2D:
    """
    Represents the 2D pose of a robot.
    """
    def __init__(self, x, y, rot):
        self.x = x
        self.y = y
        self.rot = rot

    def __eq__(self, pose):
        """ Overwrite the equality method to depend only on the values, not the object. """
        return round(self.x, 5) == round(pose.x, 5) \
                and round(self.y, 5) == round(pose.y, 5) \
                and round(self.rot, 5) == round(pose.rot, 5)

    def __str__(self):
        return "Pose(x={0}, y={1}, rot={2})".format(self.x, self.y, self.rot)
