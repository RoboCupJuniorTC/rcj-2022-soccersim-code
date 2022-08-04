import math

class Vector2:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def Magnitude(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y)

    def Normalized(self) -> 'Vector2':
        magnitude: float = self.Magnitude()
        return self / magnitude

    def GetAngle(self) -> float:
        # Get angle in rad between -pi and pi
        rad = math.atan2(self.y, self.x)

        # Calculate angle between 0 and 2pi
        if rad < 0:
            rad += 2 * math.pi

        # Change from anticlockwise to clockwise
        rad = 2 * math.pi - rad

        return rad

    @staticmethod
    def Distance(firstVector2: 'Vector2', secondVector2: 'Vector2') -> float:
        difference: Vector2 = firstVector2 - secondVector2
        return difference.Magnitude()

    def __add__(self, other: 'Vector2'):
        x = self.x + other.x
        y = self.y + other.y
        return Vector2(x, y)

    def __sub__(self, other: 'Vector2'):
        x = self.x - other.x
        y = self.y - other.y
        return Vector2(x, y)

    def __mul__(self, other: 'Vector2'):
        x = self.x * other.x
        y = self.y * other.y
        return Vector2(x, y)

    def __truediv__(self, other: 'Vector2'):
        x = self.x / other.x
        y = self.y / other.y
        return Vector2(x, y)

    def __truediv__(self, other: float):
        x = self.x / other
        y = self.y / other
        return Vector2(x, y)

    def __str__(self):
        return "({0}, {1})".format(self.x, self.y)

    def __iter__(self):
        return iter([self.x, self.y])

Vector2.zero = Vector2(0, 0)
Vector2.north = Vector2(1, 0)
