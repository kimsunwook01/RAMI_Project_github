import numpy as np
import mujoco

class ImuProcessor:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        
        try:
            self.accel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")
            self.gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")
            self.accel_adr = model.sensor_adr[self.accel_id]
            self.gyro_adr = model.sensor_adr[self.gyro_id]
        except ValueError:
            self.accel_adr = -1
            self.gyro_adr = -1

        # Odometry (추측 항법) 상태 변수
        self.last_time = 0.0
        self.yaw_heading = 0.0  # 상대적 Z축 회전각(Radian)
        
    def process(self, data: mujoco.MjData):
        if self.accel_adr == -1 or self.gyro_adr == -1:
            return {"accel": [0,0,0], "gyro": [0,0,0], "yaw_heading": 0.0}
            
        accel = data.sensordata[self.accel_adr:self.accel_adr+3].copy()
        gyro = data.sensordata[self.gyro_adr:self.gyro_adr+3].copy()
        
        # 간단한 자이로스코프 Z축 적분을 통한 상대 Yaw 방위각 계산 (Dead Reckoning)
        current_time = data.time
        dt = current_time - self.last_time
        if dt > 0:
            # gyro[2]는 로컬 Z축 회전 각속도 (rad/s)
            self.yaw_heading += gyro[2] * dt
            
        self.last_time = current_time
        
        return {
            "accel": accel.tolist(),
            "gyro": gyro.tolist(),
            "yaw_heading": self.yaw_heading
        }
