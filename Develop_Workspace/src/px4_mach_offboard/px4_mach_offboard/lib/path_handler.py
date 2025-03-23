import numpy as np
""" CONST """

from numpy.typing import NDArray
from .convention import Waypoint, YAW, LAT, LON, ALT
from .dummy_planner import DummyPlanner
from .middle_point_planner import MiddlePointPlanner

import numpy as np


class PathHandler:

    '''
    @param path: list of Waypoint
           => 현재 위치를 포함해야 함
    @param PathPlanner: type of PathPlanner     
    @return path: NDArray[Waypoint]
    '''
    def __init__(self, path: NDArray[Waypoint], PathPlanner: type):
        self.path = path
        self.new_path = path[0] # np.empty((0, 4))
        if len(path) < 2:
            raise ValueError("path should have more 2 waypoints")
        self.PathPlanner = PathPlanner



    def print_path(self):
        for i, wp in enumerate(self.path):
            print(f"Waypoint {i}: {wp[LAT]=}, {wp[LON]=}, {wp[ALT]=} {wp[YAW]=}")

    def make_path(self):
        # self.path_x_list, self.path_y_list, self.path_yaw_list = [], [], []
        # self.path_x, self.path_y, self.path_yaw = np.empty((0,)), np.empty((0,)), np.empty((0,))   # 크기가 0인 객체 배열, [], []
        for i in range(len(self.path) - 1):
            path_planner = self.PathPlanner(self.path[i], self.path[i + 1])
            # TODO 현재 경로 포함해서 줘야 함 다음 경로 ㄴㄴ
            cal_path = path_planner.plan() 
            self.new_path = np.vstack([self.new_path, cal_path])

        return self.new_path


if __name__ == '__main__':

    waypoints = np.array([
        [37.5478915, 127.1194249, 20.0, float('nan')],
        [37.5474712, 127.1186771, 20.0, float('nan')],
        [37.5468944, 127.1186654, 20.0, float('nan')],
    ])

    
    # path_handldler:PathHanlder(waypoints, DummyPlanner)
    path_handler = PathHanlder(waypoints, MiddlePointPlanner)
    path_handler.print_path()
    new_path = path_handler.make_path()


    import matplotlib.pyplot as plt

    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ko', markersize=15, label='way-points')
    # plt.plot(new_path[:, 0], new_path[:, 1], 'ro-', label='Planned Path(DummyPlanner)')
    plt.plot(new_path[:, 0], new_path[:, 1], 'bo-', label='Planned Path(MiddlePointPlanner)')
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.xlabel("lat[m]")
    plt.ylabel("lon[m]")
    plt.show()
    print('new path: ', new_path)
    print('waypoints: ', waypoints)
    print('path_handler: ', path_handler)
    print('path_handler.new_path: ', path_handler.new_path)

