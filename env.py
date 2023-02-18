"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = (40, 140)
        self.y_range = (-220, 40)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        # 140 - 40
        # 40 - -220
        obs_boundary = [
            [40, 40, 100, 0],
            [140, 40, 0, -260],
            [140, -220, -100, 0],
            [40, -220, 0, 260],
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [90, -115, 20, -20]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [135, -91, 31],
            [45, -91, 31],
            [70, -70, 9],
            [75, -73, 7],
            [105, -107, 8],
            [100, -114, 8],
            [110, -119, 11],
            [87, -128, 8],
            [94, -121, 8],
            [79, -132, 5],
            [74, -79, 3],
            [150, -116, 26],
            [150, -65, 26],
            [34, -67, 26],
            [34, -116, 26]
        ]

        return obs_cir
