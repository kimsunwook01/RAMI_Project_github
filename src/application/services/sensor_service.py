from src.infrastructure.sensors.vision_processor import VisionProcessor
from src.infrastructure.sensors.ultrasonic_processor import UltrasonicProcessor
from src.infrastructure.sensors.imu_processor import ImuProcessor
from typing import Dict

class SensorService:
    def __init__(self, model, data):
        self.vision = VisionProcessor(model, data)
        self.ultrasonic = UltrasonicProcessor(model, data)
        self.imu = ImuProcessor(model, data)

    def get_sensor_state(self) -> Dict:
        """
        현재 시스템의 모든 센서 상태를 모아서 반환합니다.
        """
        return {
            "imu": self.imu.get_imu_data(),
            "ultrasonic_min_dist": self.ultrasonic.get_minimum_distance(),
            "ultrasonic_all": self.ultrasonic.get_all_distances(),
            "gripper_camera": self.vision.get_camera_info("gripper_camera"),
            "head_camera": self.vision.get_camera_info("head_camera")
        }
