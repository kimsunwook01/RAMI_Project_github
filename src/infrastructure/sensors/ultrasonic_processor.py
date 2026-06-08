import mujoco
from typing import List

class UltrasonicProcessor:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.sensor_names = [f"us_{i}_sensor" for i in range(1, 9)]
        self.sensor_ids = []
        for name in self.sensor_names:
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid != -1:
                self.sensor_ids.append(sid)
            else:
                print(f"[Warning] Sensor {name} not found in model.")

    def get_all_distances(self) -> List[float]:
        distances = []
        for sid in self.sensor_ids:
            adr = self.model.sensor_adr[sid]
            dist = self.data.sensordata[adr]
            # MuJoCo rangefinder returns -1 if no geom is hit within its max range
            if dist < 0:
                dist = 5.0 # Max range
            distances.append(float(dist))
        return distances

    def get_minimum_distance(self) -> float:
        dists = self.get_all_distances()
        if not dists:
            return 5.0
        return min(dists)
