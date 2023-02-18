"""
RRT_2D
"""

import math
import time

import numpy as np

import env
import plotting
import utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):
                    self.new_state(node_new, self.s_goal)
                    return self.extract_path(node_new)

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def convert_angles_path_to_field_path(angle_path):
    field_path = list()
    for point in angle_path:
        lower_vec = np.array([0.87 * math.cos(math.radians(point[0])), 0.87 * math.sin(math.radians(point[0]))])
        upper_vec = np.array([0.83 * math.cos(math.radians(point[1])), 0.83 * math.sin(math.radians(point[1]))])
        field_path.append(lower_vec + upper_vec)
    return field_path


def smooth_path(rrt_solution, max_iter=100, tolerance=0.01):
    """
    Smooth an RRT solution using a simple gradient descent algorithm.

    :param rrt_solution: The RRT solution to be smoothed, represented as a list of 2D points.
    :param max_iter: The maximum number of iterations to perform. Defaults to 100.
    :param tolerance: The minimum distance between two successive iterations at which to stop. Defaults to 0.01.
    :return: The smoothed RRT solution, represented as a list of 2D points.
    """
    smoothed_path = rrt_solution.copy()
    iter_count = 0
    while iter_count < max_iter:
        last_point = smoothed_path[0]
        for i in range(1, len(smoothed_path) - 1):
            current_point = smoothed_path[i]
            next_point = smoothed_path[i+1]
            new_point = current_point + 0.5 * (last_point + next_point - 2 * current_point)
            smoothed_path[i] = new_point
            last_point = current_point
        iter_count += 1
        if np.linalg.norm(smoothed_path[-1] - rrt_solution[-1]) < tolerance:
            break
    return smoothed_path


def main():
    x_start = (90, -90)  # Starting node
    x_goal = (90, -210)  # Goal node

    t1 = time.time()
    rrt = Rrt(x_start, x_goal, 1, 0.1, 10000)
    path = rrt.planning()
    print(f"rrt calculation time: {time.time() - t1}")

    path = np.array([np.array(xi) for xi in path])
    t1 = time.time()
    smoothed_path = smooth_path(path)
    print(f"smoothing calculation time: {time.time() - t1}")

    field_path = convert_angles_path_to_field_path(smoothed_path)

    if path is not None:
        rrt.plotting.animation(rrt.vertex, [path, smoothed_path], "RRT", True)
    else:
        print("No Path Found!")

    rrt.plotting.plot_path(smoothed_path, equal_axis=True)
    rrt.plotting.plot_path(field_path, equal_axis=True)

    # traj = Trajectory(path)
    # traj.calculate()
    # calculate_trajectory(path, MAX_V, a_max)


if __name__ == '__main__':
    main()
