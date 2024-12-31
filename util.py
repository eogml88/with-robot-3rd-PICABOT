# Copyright 2024 @With-Robot 3.5
#
# Licensed under the MIT License;
#     https://opensource.org/license/mit

import numpy as np

from dataclasses import dataclass
from enum import Enum


#
# An Enum class for defining the state of a robot
#
class State(Enum):
    StandBy = 0
    MoveToPick = 1
    FindTarget = 2
    ApproachToTarget = 3
    PickTarget = 4
    MoveToPlace = 5
    PlaceTarget = 6
    MoveToBase = 7


#
# A class defining the settings required for robot operation
#
@dataclass(frozen=True)
class Config:
    map_size: tuple = (100, 100)
    map_cell: float = 0.1
    lidar_offset: float = 0.2751  # distance from youBot_ref to lidar
    lidar_pcd: int = 342 * 2  # Point Cloud Density


#
# A class defining the mission for robot
#
@dataclass(frozen=True)
class Mission:
    pass


#
# A class defining the context for robot operation
#
@dataclass
class Context:
    map: np.array = None


#
# A class defining readable datas of robot
#
@dataclass
class ReadData:
    localization: np.array = None
    joints: np.array = None
    scan_flg: bool = False
    scan: np.array = None
    scan_position: tuple = None
    img_flag: bool = False
    img: np.array = None


#
# A class defining control datas of robot
#
@dataclass
class ControlData:
    wheels_position: np.array = None
