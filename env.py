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
            # [14, 12, 8, 2],
            # [18, 22, 8, 3],
            # [26, 7, 2, 12],
            # [32, 14, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            # [90, -90, 20],
            # [46, 20, 2],
            # [15, 5, 2],
            # [37, 7, 3],
            # [37, 23, 3]
        ]

        return obs_cir
