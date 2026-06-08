import mujoco
from typing import Dict

class ImuProcessor:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.accel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")
        self.gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")

    def get_imu_data(self) -> Dict[str, list]:
        data_dict = {"accel": [0.0, 0.0, 0.0], "gyro": [0.0, 0.0, 0.0]}
        
        if self.accel_id != -1:
            adr = self.model.sensor_adr[self.accel_id]
            data_dict["accel"] = [float(self.data.sensordata[adr+i]) for i in range(3)]
            
        if self.gyro_id != -1:
            adr = self.model.sensor_adr[self.gyro_id]
            data_dict["gyro"] = [float(self.data.sensordata[adr+i]) for i in range(3)]
            
        return data_dict
