import math
from math import pi


class Vector2D:
    """Class to represent a vector in 2D space

    Args:
        x (float): X coordinate of the vector.
        y (float): Y coordinate of the vector.
    """
    __slots__ = ('x', 'y')

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Vector({self.x:.2f}, {self.y:.2f})"

    def __format__(self, format_spec) -> str:
        return f"Vector({self.x:{format_spec}} {self.y:{format_spec}})"

    def __add__(self, other) -> "Vector2D":
        if isinstance(other, Vector2D):
            return type(self)(self.x + other.x, self.y + other.y)
        elif isinstance(other, (int, float)):
            return type(self)(self.x + other, self.y + other)
        else:
            return NotImplemented

    def __radd__(self, other) -> "Vector2D":
        return self + other

    def __sub__(self, other) -> "Vector2D":
        return self + (-other)

    def __rsub__(self, other) -> "Vector2D":
        return -self + other

    def __mul__(self, other) -> "Vector2D":
        if isinstance(other, Vector2D):
            return type(self)(self.x * other.x, self.y * other.y)
        elif isinstance(other, (int, float)):
            return type(self)(self.x * other, self.y * other)
        else:
            return NotImplemented

    def __rmul__(self, other) -> "Vector2D":
        return self * other

    def __truediv__(self, other) -> "Vector2D":
        if isinstance(other, Vector2D):
            return type(self)(self.x / other.x, self.y / other.y)
        elif isinstance(other, (int, float)):
            return type(self)(self.x / other, self.y / other)
        else:
            return NotImplemented

    def __matmul__(self, other) -> "Vector2D":
        return self.dot(other)

    def __neg__(self) -> "Vector2D":
        return type(self)(-self.x, -self.y)

    def __abs__(self) -> "Vector2D":
        return self.length

    def __iter__(self) -> "Vector2D":
        return iter((self.x, self.y))

    def __gt__(self, other) -> "Vector2D":
        return self.length > other.length

    @property
    def length(self) -> float:
        return (self.x ** 2 + self.y ** 2) ** 0.5

    @property
    def normalized(self) -> "Vector2D":
        return self / self.length

    @property
    def angle(self) -> float:
        return math.atan2(self.y, self.x)

    def dot(self, other) -> float:
        return self.x * other.x + self.y * other.y

    def rotate(self, angle: float) -> "Vector2D":
        """Rotate vector by angle

        Args:
            angle (float): Angle to rotate by.
        """
        x = self.x * math.cos(angle) - self.y * math.sin(angle)
        y = self.x * math.sin(angle) + self.y * math.cos(angle)
        return type(self)(x, y)

    def reflect(self, origin: "Vector2D") -> "Vector2D":
        """Reflect vector around origin

        Args:
            origin (Vector2D): Origin to reflect around.

        Returns:
            Vector2D: Reflected vector.
        """
        return origin * 2 - self

    def towards(self, other: "Vector2D", distance: float = 1) -> "Vector2D":
        """Get vector pointing towards other vector

        Args:
            other (Vector2D): Vector to point towards.
            distance (float): Distance to travel.

        Returns:
            Vector2D: Vector pointing towards other vector.
        """
        return (other - self).normalized * distance


def to_world_pos(pos: "Vector2D", angle: float):
    return pos.rotate(angle)


def normalize_angle(radian: float) -> float:
    """normalize angle to [-pi, pi]"""
    return (radian + pi) % (2 * pi) - pi


def sign(x):
    return 1 if x >= 0 else -1


def atan_smooth(x, sharpness=5):
    return math.atan(x * sharpness) / pi * 2


def clip(value, min_val=-10, max_value=10):
    return max(min_val, min(value, max_value))
