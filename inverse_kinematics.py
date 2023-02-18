import math
import numpy as np


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_angle(self):
        return math.degrees(math.atan2(self.y, self.x))


def calculate_lower_arm(point: Point):
        if point.y < 0.00000001:
            point.y = 0.0000001

        r1 = 0.876
        r2 = 0.844
        deltaR = r1 * r1 - r2 * r2
        p = deltaR + math.pow(point.x, 2) + math.pow(point.y, 2);

        a = 4 * math.pow(point.y, 2) + 4 * math.pow(point.x, 2)
        b = -4 * point.x * p
        c = p * p - 4 * math.pow(point.y, 2) * r1 * r1

        det = math.sqrt(math.pow(b, 2) - 4 * a * c)
        if det == 0:
            upperJointXs = [-b / (2*a)]
        else:
            upperJointXs = [(-b + det) / (2 * a), (-b - det) / (2 * a)]

        if len(upperJointXs) == 0:
            return None

        if len(upperJointXs) == 1:
            upperJointX = upperJointXs[0]
            upperJointY = (p - 2 * point.x * upperJointX) / (2 * point.y)
        else:
            x1 = upperJointXs[0]
            x2 = upperJointXs[1]
            y1 = (p - 2 * point.x * x1) / (2 * point.y)
            y2 = (p - 2 * point.x * x2) / (2 * point.y)

            if y1 > y2:
                upperJointX = x1
                upperJointY = y1
            else:
                upperJointX = x2
                upperJointY = y2

        return Point(upperJointX, upperJointY)


def calculate_upper_arm(point: Point):
    lower = calculate_lower_arm(point)

    return Point(point.x - lower.x, point.y - lower.y)


def point_to_angles(point: Point):
    upper = calculate_upper_arm(point)
    lower = calculate_lower_arm(point)
    print(upper.x, upper.y)
    upper_angle = upper.get_angle()
    # if point.x < 0:
    #     upper_angle = upper_angle - 360
    return Point(lower.get_angle(), upper_angle)


if __name__ == '__main__':
    p = Point(-0.8, 0.2)
    angles = point_to_angles(p)
    print(f"theta0: {angles.x}, theta1: {angles.y}")