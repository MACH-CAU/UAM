import numpy as np
from numpy.typing import NDArray

LAT, LON, ALT, YAW = 0, 1, 2, 3 
Waypoint = NDArray[np.float64]

DICT_NAVIGATION_STATE = {
        0: "MANUAL",
        1: "ALTCTL",
        2: "POSCTL",
        3: "AUTO_MISSION",
        4: "AUTO_LOITER",
        5: "AUTO_RTL",
        6: "POSITION_SLOW",
        # 7: "FREE5",
        # 8: "FREE4",
        # 9: "FREE3",
        10: "ACRO",
        11: "FREE2",
        12: "DESCEND",
        13: "TERMINATION",
        14: "OFFBOARD",
        # 15: "STAB",
        # 16: "FREE1",
        17: "AUTO_TAKEOFF",
        18: "AUTO_LAND",
        19: "AUTO_FOLLOW_TARGET",
        20: "AUTO_PRECLAND",
        21: "ORBIT",
        22: "AUTO_VTOL_TAKEOFF",
        # 23: "EXTERNAL1",
        # 24: "EXTERNAL2",
        # 25: "EXTERNAL3",
        # 26: "EXTERNAL4",
        # 27: "EXTERNAL5",
        # 28: "EXTERNAL6",
        # 29: "EXTERNAL7",
        # 30: "EXTERNAL8",
        # 31: "MAX",
}


DICT_VEHICLE_TYPE = {
        0: "UNKNOWN",
        1: "ROTARY_WING",
        2: "FIXED_WING",
}

