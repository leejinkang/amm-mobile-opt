from abc import ABC, abstractmethod
import numpy as np
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import ColorRGBA
from tf_transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import TransformStamped, Quaternion, Pose


class SteeringWheel:
    def __init__(self):
        self.wheel_names = ["fr", "br", "bl", "fl"]
        self.wheel_steering_angles = [0.0, 0.0, 0.0, 0.0] # fr, br, bl, fl [rad]
        self.wheel_steering_speed = [0.0, 0.0, 0.0, 0.0] # fr, br, bl, fl [rad/s]

class DrivingWheel:
    def __init__(self):
        self.wheel_names = ["fr", "br", "bl", "fl"]
        self.wheel_driving_speed = [0.0, 0.0, 0.0, 0.0] # fr, br, bl, fl [rad/s]
        self.wheel_driving_velocity = [0.0, 0.0, 0.0, 0.0] # fr, br, bl, fl [m/s] --> each wheel velocity

class FourWheelSwerveModel:
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._name = name

        # ===========================
        # TODO: Load from external file
        # Config
        self.body_x_length = 0.9  # forward
        self.body_y_length = 0.6
        self.wheel_radius = 0.12
        # Order: fr, br, bl, fl
        self.fr_xy = (0.35, -0.25)
        self.br_xy = (-0.35, -0.25)
        self.bl_xy = (-0.35, 0.25)
        self.fl_xy = (0.35, 0.25)
        self.wheel_names = ["fr", "br", "bl", "fl"]
        self.wheel_frame_ids = [f"{self.name}_wheel_{w}" for w in self.wheel_names]
        self.wheel_x = [self.fr_xy[0], self.br_xy[0], self.bl_xy[0], self.fl_xy[0]]
        self.wheel_y = [self.fr_xy[1], self.br_xy[1], self.bl_xy[1], self.fl_xy[1]]




