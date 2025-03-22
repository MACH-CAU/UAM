import numpy as np
from scipy import interpolate
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from .convention import LAT, LON, ALT, YAW


class Spline2dHandler:
    def __init__(self, points_waypoint, step, kind='linear'):
        lat, lon  = points_waypoint[:, LAT], points_waypoint[:, LON]
        print(lat, lon)
        self.points_waypoint = points_waypoint

        self.ds = np.hypot(np.diff(lat), np.diff(lon))
        self.step = step
        self.s = self.__calculate_s()
        if kind == 'cubic':
            self.sx = CubicSpline(self.s, lat, bc_type='natural')
            self.sy = CubicSpline(self.s, lon, bc_type='natural')
        else:
            self.sx = interpolate.interp1d(self.s, lat, kind=kind)
            self.sy = interpolate.interp1d(self.s, lon, kind=kind)

    def calculate(self):
        path = np.empty((0, 4))  # Updated to handle (lat, lon, yaw)
        s_array = np.arange(0, self.s[-1], self.step)
        i = 0
        for cur_s in s_array:
            cur_lat, cur_lon = self.__calculate_position(cur_s)
            cur_path = np.vstack((cur_lat, cur_lon, self.points_waypoint[i, ALT], self.points_waypoint[i, YAW]))
            path = np.vstack([path, cur_path.T])
            if i < len(self.points_waypoint)                     \
                and int(cur_lat) == self.points_waypoint[i, LAT] \
                and int(cur_lon) == self.points_waypoint[i, LON]:
                i += 1
        path = np.vstack((path, self.points_waypoint[-1]))

        return path

    def __calculate_s(self):
        s = [0.0]
        s.extend(np.cumsum(self.ds))
        return s

    def __calculate_position(self, cur_s):
        return self.sx(cur_s), self.sy(cur_s)

if __name__ == '__main__':
    waypoints = np.array([
        [37.5478915, 127.1194249, 20.0, float('nan')],
        [37.5474712, 127.1186771, 20.0, float('nan')],
        [37.5468944, 127.1186654, 20.0, float('nan')],
    ])
    print(waypoints)
    path1 = Spline2dHandler(waypoints, 0.0001, 'linear').calculate()
    path2 = Spline2dHandler(waypoints, 0.0001, 'quadratic').calculate()
    path3 = Spline2dHandler(waypoints, 0.0001, 'cubic').calculate()
    print(path3)

    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='way-points')
    plt.plot(path1[:, 0], path1[:, 1], 'yx-', label='linear')
    plt.plot(path2[:, 0], path2[:, 1], 'g*-', label='quadratic')
    plt.plot(path3[:, 0], path3[:, 1], 'b^-', label='cubic')
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("lat[m]")
    plt.ylabel("lon[m]")
    plt.show()




