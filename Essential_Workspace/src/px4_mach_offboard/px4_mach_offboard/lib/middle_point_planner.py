
import numpy as np



class MiddlePointPlanner:
    def __init__(self, start_waypoint, end_waypoint):
        if len(start_waypoint) != 4 or len(end_waypoint) != 4:
            raise ValueError("waypoint should have more 4 waypoints")
        self.start_waypoint = start_waypoint
        self.end_waypoint = end_waypoint
        self.new_path = np.empty((0, 4))


    def plan(self):

        # 계산된 경로 추가
        self.__calculate()

        # 마지막 경로 추가
        self.new_path = np.vstack([self.new_path, self.end_waypoint])
        return self.new_path


    def __calculate(self):
        insert_path = np.empty((0, 4))

        insert_path = np.vstack([
            insert_path, 
            np.median([self.start_waypoint, self.end_waypoint], axis=0)
        ])

        self.new_path = np.vstack([self.new_path, insert_path])

