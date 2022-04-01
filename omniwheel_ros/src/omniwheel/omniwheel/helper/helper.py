import math
import cmath


def to_polar(x, y):
    """ Return the angle and radius (less than or equal to 1) of the polar representation of the cartesian coordinates
        x and y.
    """
    r = math.sqrt(x ** 2 + y ** 2)
    if r > 1:
        r = 1
    t = cmath.polar(x + y * 1j)[1]
    return t, r
