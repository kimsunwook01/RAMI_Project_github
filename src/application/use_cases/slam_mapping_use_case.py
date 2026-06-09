import mujoco
import numpy as np
from src.domain.mapping.occupancy_grid import OccupancyGridMap
from src.application.services.sensor_service import SensorService

class SlamMappingUseCase:
    def __init__(self, model: mujoco.MjModel, map_width=20.0, map_height=20.0, resolution=0.05):
        self.model = model
        self.grid_map = OccupancyGridMap(width_m=map_width, height_m=map_height, resolution=resolution)
        
        # 초음파 센서 사이트 ID 캐싱
        self.sensor_count = 8
        self.site_ids = []
        for i in range(1, self.sensor_count + 1):
            try:
                site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"us_site_{i}")
                self.site_ids.append(site_id)
            except ValueError:
                self.site_ids.append(-1)

    def execute(self, data: mujoco.MjData, sensor_service: SensorService):
        """
        한 스텝의 센서 데이터를 읽어 그리드 맵을 업데이트합니다.
        """
        # 1. 센서 데이터 가져오기 (IMU에서 완벽한 오도메트리 획득, Ultrasonic에서 평균 거리 획득)
        sensor_state = sensor_service.get_sensor_state(data)
        us_data = sensor_state["ultrasonic"]
        
        # 2. 각 초음파 센서의 측정값을 맵에 반영
        for i, site_id in enumerate(self.site_ids):
            if site_id == -1:
                continue
                
            dist = us_data.get(f"ch{i+1}", -1.0)
            
            # 사이트의 글로벌 위치 (start_x, start_y)
            start_x = data.site_xpos[site_id][0]
            start_y = data.site_xpos[site_id][1]
            
            # 사이트의 글로벌 전방 방향 (mat[:, 2]는 로컬 Z축이 글로벌로 변환된 벡터)
            mat = data.site_xmat[site_id].reshape(3, 3)
            forward_vec = mat[:, 2]
            
            # 2D 평면에서의 방향 벡터 정규화
            dir_x = forward_vec[0]
            dir_y = forward_vec[1]
            norm = math.hypot(dir_x, dir_y)
            if norm < 1e-5:
                continue
            dir_x /= norm
            dir_y /= norm
            
            is_hit = False
            if dist > 0 and dist < 3.0:
                # 장애물에 닿은 경우
                end_x = start_x + dir_x * dist
                end_y = start_y + dir_y * dist
                is_hit = True
            else:
                # 닿지 않은 경우 최대 감지 거리(3.0m)까지 빈 공간으로 마킹
                end_x = start_x + dir_x * 3.0
                end_y = start_y + dir_y * 3.0
                is_hit = False
                
            self.grid_map.update_ray(start_x, start_y, end_x, end_y, is_hit)

    def get_map(self):
        return self.grid_map.get_probability_map()

import math
