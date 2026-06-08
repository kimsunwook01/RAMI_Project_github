import mujoco
from src.infrastructure.sensors.vision_processor import VisionProcessor
from src.infrastructure.sensors.ultrasonic_processor import UltrasonicProcessor
from src.infrastructure.sensors.imu_processor import ImuProcessor

class SensorService:
    """
    모든 하위 센서 프로세서(Vision, Ultrasonic, IMU)를 래핑하여
    상위 계층(애플리케이션 및 도메인)에 통합된 센서 상태를 단일 딕셔너리로 제공하는 파사드(Facade) 클래스입니다.
    """
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        
        # 프로세서 초기화
        self.vision = VisionProcessor(model)
        self.ultrasonic = UltrasonicProcessor(model)
        self.imu = ImuProcessor(model)
        
    def get_sensor_state(self, data: mujoco.MjData):
        """
        현재 스텝의 모든 센서 정보를 모아서 반환합니다.
        """
        # 1. Vision (YOLO v8 Object Detection)
        head_detections = self.vision.process_camera(data, "head_camera")
        gripper_detections = self.vision.process_camera(data, "gripper_camera")
        
        # 2. Ultrasonic
        us_data = self.ultrasonic.process(data)
        
        # 3. IMU (Odometry)
        imu_data = self.imu.process(data)
        
        return {
            "vision": {
                "head_camera_detections": head_detections,
                "gripper_camera_detections": gripper_detections
            },
            "ultrasonic": us_data,
            "imu": imu_data
        }
