# %% importing
import numpy as np
import matplotlib.pyplot as plt


MAX_A = 5
MAX_V = 30


class Trajectory:

    def __init__(self, path):
        self.path = path
        self.path_cumsum = self._get_path_cumsum()
        self.t_vectors = []
        self.s_vectors = []
        self.v_vectors = []
        self.a_vectors = []

    def calculate(self):
        path_length = self._get_path_cumulative_length()
        if path_length > MAX_V ** 2 / MAX_A:  ## this is a trapezoid profile
            self._calculate_trapezoid_profile(path_length)
        else:
            self._calculate_triangle_profile(path_length)
        self._plot_path()

        return self.time_array, self.theta_array_mod, self.v_array, self.a_array

    def _plot_path(self):
        plt.figure()
        for t_vec, s_vec in zip(self.t_vectors, self.s_vectors):
            plt.plot(t_vec, s_vec, '.-')
        plt.xlabel('time')
        plt.ylabel('path')

        plt.figure()
        for t_vec, v_vec in zip(self.t_vectors, self.v_vectors):
            plt.plot(t_vec, v_vec, '.-')
        plt.xlabel('time')
        plt.ylabel('velocity')

        plt.figure()
        for t_vec, a_vec in zip(self.t_vectors, self.a_vectors):
            plt.plot(t_vec, a_vec, '.-')
        plt.xlabel('time')
        plt.ylabel('acceleration')
        plt.show()

    def _calculate_trapezoid_profile(self, path_length):
        ## time DURATIONs of the ramp (=t1) and the plateau (=t2)
        t1 = MAX_V / MAX_A
        t2 = path_length / MAX_V - t1

        ## the theoretic path transitions
        s_way_points = np.cumsum(np.array([t1 * MAX_V / 2, t2 * MAX_V, t1 / 2 * MAX_V]))

        ## path points at each velocity segment (ramp up, plateau, ramp down)
        self.s_vectors.append(self.path_cumsum[self.path_cumsum <= s_way_points[0]])
        self.s_vectors.append(self.path_cumsum[(self.path_cumsum > s_way_points[0]) & (self.path_cumsum <= s_way_points[1])])
        self.s_vectors.append(self.path_cumsum[(self.path_cumsum > s_way_points[1]) & (self.path_cumsum <= s_way_points[2])])
        self.theta_array_mod = np.concatenate(self.s_vectors)

        ## time stamps for each path segment
        self.t_vectors.append(np.sqrt(2 * self.s_vectors[0] / MAX_A))
        self.t_vectors.append((self.s_vectors[1] - s_way_points[0]) / MAX_V + t1)
        self.t_vectors.append(t1 - np.sqrt(4 * t1 ** 2 - 8 * (self.s_vectors[2] - s_way_points[1]) / MAX_A) / 2 + (t1 + t2))
        self.time_array = np.concatenate(self.t_vectors)

        ## velocity stamps for each path segment
        self.v_vectors.append(self.t_vectors[0] * MAX_A)
        self.v_vectors.append(np.repeat(MAX_V, self.t_vectors[1].shape))
        self.v_vectors.append(MAX_V - (self.t_vectors[2] - (t1 + t2)) * MAX_A)
        self.v_array = np.concatenate(self.v_vectors)

        ## acceleration stamps ofr each path semgent
        self.a_vectors.append(np.repeat(MAX_A, self.t_vectors[0].shape))
        self.a_vectors.append(np.zeros_like(self.t_vectors[1]))
        self.a_vectors.append(np.repeat(-MAX_A, self.t_vectors[2].shape))
        self.a_array = np.concatenate(self.a_vectors)

    def _calculate_triangle_profile(self, path_length):
        t1 = np.sqrt(path_length / MAX_A)
        s_way_points = np.cumsum(np.array([t1 ** 2 * MAX_A / 2, t1 ** 2 * MAX_A / 2]))

        ## path points at each velocity segment (ramp up, plateau, ramp down)
        self.s_vectors.append(self.path_cumsum[self.path_cumsum <= s_way_points[0]])
        self.s_vectors.append(self.path_cumsum[(self.path_cumsum > s_way_points[0])])
        self.theta_array_mod = np.concatenate(self.s_vectors)

        ## time stamps for each path segment
        self.t_vectors.append(np.sqrt(2 * self.s_vectors[0] / MAX_A))
        # t2_vec=t1-np.sqrt(2*s2_vec/MAX_A)
        self.t_vectors.append(t1 - np.sqrt(4 * t1 ** 2 - 8 * (self.s_vectors[1] - s_way_points[0]) / MAX_A) / 2 + t1)

        self.time_array = np.concatenate(self.t_vectors)

        ## velocity stamps for each path segment
        self.v_vectors.append(self.t_vectors[0] * MAX_A)
        self.v_vectors.append(self.v_vectors[0][-1] - (self.t_vectors[1] - t1) * MAX_A)
        self.v_array = np.concatenate(self.v_vectors)

        ## acceleration stamps ofr each path semgent
        self.a_vectors.append(np.repeat(MAX_A, self.t_vectors[0].shape))
        self.a_vectors.append(np.repeat(-MAX_A, self.t_vectors[1].shape))
        self.a_array = np.concatenate(self.a_vectors)

    def _get_path_cumsum(self):
        return np.concatenate((np.zeros(1), np.cumsum((np.linalg.norm(np.diff(self.path, axis=0), axis=1)))))

    def _get_path_cumulative_length(self):
        return self.path_cumsum[-1]


if __name__ == "__main__":
    # theta_array=np.random.random((200,2))
    # ind_sort=np.argsort(theta_array[:,0])
    # theta_array2=theta_array[ind_sort,]
    MAX_A = 5  # in rad/sec^2
    MAX_V = 30  # in rad/sec


    a1 = np.linspace(0, 200, 200)
    a2 = a1 * 2
    theta_array = np.column_stack((a1, a2))
    traj = Trajectory(theta_array)
    traj.calculate()
