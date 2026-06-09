import mujoco
import numpy as np

class UltrasonicProcessor:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.sensor_count = 8
        self.site_ids = []
        
        # 8개의 초음파 센서 사이트(site) ID를 미리 캐싱
        for i in range(1, self.sensor_count + 1):
            site_name = f"us_site_{i}"
            try:
                site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
                self.site_ids.append(site_id)
            except ValueError:
                self.site_ids.append(-1) # 사이트가 없는 경우

        # 부채꼴(Cone) 형태의 5개 레이 각도 설정 (-15, -7.5, 0, 7.5, 15 도)
        self.cone_angles = np.radians([-15.0, -7.5, 0.0, 7.5, 15.0])
        self.max_range = 3.0 # 최대 감지 거리 3m

    def process(self, data: mujoco.MjData):
        """
        각 초음파 센서 사이트에서 5개의 Ray를 쏘아 거리를 측정하고 평균값을 반환합니다.
        """
        readings = {}
        min_dist_overall = float('inf')
        
        geomgroup = np.array([1, 1, 1, 0, 1, 1], dtype=np.uint8) # group 3 (천장) 제외
        geomid_out = np.zeros(1, dtype=np.int32)
        
        # 로봇 자기 자신(base_link)은 레이캐스트에서 제외
        bodyexclude = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if bodyexclude == -1:
            bodyexclude = 0
            
        for i, site_id in enumerate(self.site_ids):
            if site_id != -1:
                # 사이트의 글로벌 위치 및 회전 행렬
                pnt = data.site_xpos[site_id]
                mat = data.site_xmat[site_id].reshape(3, 3)
                
                # 사이트의 기본 Z축(로컬 기준)은 앞쪽을 향한다고 가정 (urdf 설정에 따라 다를 수 있음)
                # 앞서 us_site의 zaxis가 전방을 가리키도록 설정되었음.
                forward_vec = mat[:, 2] # 로컬 Z축 (글로벌 좌표계로 변환된 벡터)
                up_vec = mat[:, 1] # 로컬 Y축을 위쪽 방향으로 가정 (또는 로컬 X축)
                # 레이캐스트용 부채꼴 회전 축은 글로벌 Z축 (0,0,1) 평면 상에서 회전
                rot_axis = np.array([0.0, 0.0, 1.0])
                
                valid_dists = []
                for angle in self.cone_angles:
                    # Rodrigues' 회전 공식을 이용하여 forward_vec를 rot_axis 기준으로 angle만큼 회전
                    cos_theta = np.cos(angle)
                    sin_theta = np.sin(angle)
                    ray_dir = forward_vec * cos_theta + np.cross(rot_axis, forward_vec) * sin_theta + rot_axis * np.dot(rot_axis, forward_vec) * (1 - cos_theta)
                    ray_dir = ray_dir / np.linalg.norm(ray_dir)
                    
                    dist = mujoco.mj_ray(self.model, data, pnt, ray_dir, geomgroup, True, bodyexclude, geomid_out)
                    
                    # 거리가 0 이하거나 최대 측정 거리를 초과하면 무시
                    if 0 < dist < self.max_range:
                        valid_dists.append(dist)
                
                if valid_dists:
                    avg_dist = float(np.mean(valid_dists))
                    readings[f"ch{i+1}"] = avg_dist
                    if avg_dist < min_dist_overall:
                        min_dist_overall = avg_dist
                else:
                    readings[f"ch{i+1}"] = -1.0
            else:
                readings[f"ch{i+1}"] = -1.0
                
        readings["min_dist"] = min_dist_overall if min_dist_overall != float('inf') else -1.0
        return readings
