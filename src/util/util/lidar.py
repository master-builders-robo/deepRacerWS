from sensor_msgs.msg import LaserScan

import math
import numpy as np
import enum

class LidarAngles(float, enum.Enum):
    FRONT_E: float = 0
    BACK_E: float  = 180.0
    LEFT_E: float  = 90.0
    RIGHT_E: float = -90.0

    FRONT_R: float = np.deg2rad(FRONT_E)
    BACK_R: float  = np.deg2rad(BACK_E)
    LEFT_R: float  = np.deg2rad(LEFT_E)
    RIGHT_R: float = np.deg2rad(RIGHT_E)

class Lidar:
    """
        Class that disregards infinite values from lidar, allows indexing via angle.
    """

    def __init__(self):
        """
            Initializes Lidar Class
        """
        # Init Variables
        self.initialized: bool = False
        self.ranges: np.ndarray[float] = math.nan
        self.angles: np.ndarray[float] = math.nan
        self.angle_min: float          = math.nan
        self.angle_max: float          = math.nan
        self.angle_increment: float    = math.nan

    def update(self, msg: LaserScan):
        """
            Updates Lidar Scan dists, if lidar scan dists have not yet been initialized then initialize.
            Args:
                msg: LaserScan message from lidar subscriber.
        """
        # Init Variables
        if not self.initialized:
            self.initialized = True

            # Init Consts
            self.angle_min = msg.angle_min
            self.angle_max = msg.angle_max
            self.angle_increment = msg.angle_increment

            # Init Ranges
            self.ranges = np.array(msg.ranges)
            self.ranges[self.ranges == np.inf] = math.nan

            # Create angles
            diff = self.angle_max - self.angle_min
            num = int(diff / self.angle_increment)
            self.angles = np.linspace(start=self.angle_min, stop=self.angle_max, num=num)
            return
        
        # Update ranges
        for i, msg_range in enumerate(msg.ranges):
            if math.isfinite(msg_range):
                self.ranges[i] = msg_range

    def __getitem__(self, index: int) -> tuple:
        """
            Allows indexing by index directly on the class.
            Args:
                index
            Return:
                (dist, angle)
        """
        return (self.ranges[index], self.angles[index])
    
    def __len__(self) -> int:
        """
            Gets length of internal ranges class.
        """
        return len(self.ranges)
    
    def get_ray(self, angle: float, euler_angle: bool = True) -> tuple:
        """
            Gets (dist, angle) of raycast at specified angle
            Args:
                angle: Angle at which to search
                euler_angle: Whether or not angle is an euler angle.
            Return:
                (dist, angle) if initialized, otherwise math.nan
        """
        if not self.initialized:
            return math.nan
        index = self._angle_to_index(float(angle), euler_angle=euler_angle)
        return (self.ranges[index], self.angles[index])
    
    def get_dist(self, angle: float, euler_angle: bool = True) -> tuple:
        """
            Gets dist of raycast at specified angle
            Args:
                angle: Angle at which to search
                euler_angle: Whether or not angle is an euler angle.
            Return:
                dist if initialized, otherwise math.nan
        """
        if not self.initialized:
            return math.nan
        index = self._angle_to_index(float(angle), euler_angle=euler_angle)
        return self.ranges[index]

    def _angle_to_index(self, angle: float, euler_angle: bool = True) -> int:
        """
            Gets index of raycast at specified angle
            Args:
                angle: Angle at which to find index for
                euler_angle: Whether or not angle is an euler angle.
            Return:
                index of angle.
        """
        if not self.initialized:
            return math.nan

        new_angle = angle
        # Convert angle to radians
        if euler_angle:
            new_angle = angle * 180.0 / np.pi
        
        # Clamp Angle
        new_angle += np.pi
        new_angle = new_angle % (2.0 * np.pi)
        index = int(new_angle / self.angle_increment)
        return index % len(self.ranges)