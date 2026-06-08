import mujoco

class UltrasonicProcessor:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.sensor_count = 8
        self.sensor_adrs = []
        
        # 8개의 초음파 센서 adr(주소) 미리 캐싱
        for i in range(1, self.sensor_count + 1):
            sensor_name = f"ultrasonic_sensor_{i}"
            try:
                sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
                self.sensor_adrs.append(model.sensor_adr[sensor_id])
            except ValueError:
                self.sensor_adrs.append(-1) # 센서가 없는 경우

    def process(self, data: mujoco.MjData):
        """
        data.sensordata에서 8채널 거리 측정값을 읽어옵니다.
        """
        readings = {}
        min_dist = float('inf')
        
        for i, adr in enumerate(self.sensor_adrs):
            if adr != -1:
                # rangefinder 센서는 스칼라 값 1개를 가짐
                dist = data.sensordata[adr]
                readings[f"ch{i+1}"] = dist
                if dist > 0 and dist < min_dist:
                    min_dist = dist
            else:
                readings[f"ch{i+1}"] = -1.0
                
        readings["min_dist"] = min_dist if min_dist != float('inf') else -1.0
        return readings
