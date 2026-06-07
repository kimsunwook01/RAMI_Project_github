import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    print("=== [Step 2] 로봇암 및 리프트 조인트 초기 자세 유지 테스트 ===")
    print("리프트와 7개의 회전 관절이 0.0 값을 인가받아 위치를 단단히 유지합니다.")
    print("중력에 의해 팔이 쳐지지 않고 중앙에서 앞으로 뻗은 기본 자세를 지켜야 합니다.")
    print("뷰어 창을 닫으면 종료됩니다.")
    
    # 리프트와 7개 암 조인트 (총 8개)의 목표 위치를 0.0으로 고정
    target_positions = [0.0] * 8
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            # 동체 및 휠 정지
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            
            # 암 조인트 위치 제어 유지
            adapter.control_arm_joints(target_positions)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
