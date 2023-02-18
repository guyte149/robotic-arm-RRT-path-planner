import math

import matplotlib.pyplot as plt
import numpy as np
from random import uniform

from matplotlib.patches import Circle

from inverse_kinematics import Point, point_to_angles


def convert_point_to_angles(x, y):
    L1 = 0.876
    L2 = 0.844 + 0.09
    theta1 = math.atan2(y, x)
    theta2 = math.acos((x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2))
    return np.array([math.degrees(theta1), math.degrees(theta2)])


def randomize_range(lower_x, upper_x, lower_y, upper_y, color="blue"):
    theta0_list = list()
    theta1_list = list()
    for i in range(1000):
        try:
            rand_x = uniform(lower_x, upper_x)
            rand_y = uniform(lower_y, upper_y)
            angles = point_to_angles(Point(rand_x, rand_y))
            theta0_list.append(angles.x)
            theta1_list.append(angles.y)
        except ValueError:
            continue

    plt.scatter(theta0_list, theta1_list, color=color)


def main():
    # Set up the figure and axis
    fig, ax = plt.subplots()

    # Set the limits of the x and y axis
    ax.set_xlim(0, 180)
    ax.set_ylim(-300, 80)
    plt.axis("equal")

    # Draw the rectangle frame
    rect = plt.Rectangle((40, 40), 100, -260, linewidth=1, edgecolor='r', facecolor='none')
    ax.add_patch(rect)

    # angles = convert_point_to_angles(-0.3, 0.2)

    randomize_range(0.39, 0.7, -0.15, 0.06, "blue")
    randomize_range(-0.84, -0.37, -0.15, 0.25, "green")
    randomize_range(-1.5, 1.5, -1, -0.15, "orange")

    patches = list()
    patches.append(Circle((135, -91), 31, fill=False, edgecolor='purple'))
    patches.append(Circle((45, -91), 31, fill=False, edgecolor='purple'))
    patches.append(Circle((70, -70), 9, fill=False, edgecolor='purple'))
    patches.append(Circle((75, -73), 7, fill=False, edgecolor='purple'))
    patches.append(Circle((105, -107), 8, fill=False, edgecolor='purple'))
    patches.append(Circle((100, -114), 8, fill=False, edgecolor='purple'))
    patches.append(Circle((110, -119), 11, fill=False, edgecolor='purple'))
    patches.append(Circle((87, -128), 8, fill=False, edgecolor='purple'))
    patches.append(Circle((94, -121), 8, fill=False, edgecolor='purple'))
    patches.append(Circle((79, -132), 5, fill=False, edgecolor='purple'))
    patches.append(Circle((74, -79), 3, fill=False, edgecolor='purple'))
    patches.append(Circle((150, -116), 26, fill=False, edgecolor='purple'))
    patches.append(Circle((150, -65), 26, fill=False, edgecolor='purple'))
    patches.append(Circle((34, -67), 26, fill=False, edgecolor='purple'))
    patches.append(Circle((34, -116), 26, fill=False, edgecolor='purple'))
    patches.append(plt.Rectangle((90, -115), 20, -20, linewidth=1, edgecolor='purple', facecolor='none'))


    # Add the circle to the plot
    for patch in patches:
        ax.add_patch(patch)

    # Show the plot
    plt.show()


if __name__ == '__main__':
    main()

