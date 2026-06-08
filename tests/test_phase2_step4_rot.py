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
    
    print("=== [Step 4] rotation_joint 피드백 기반 자동 왕복 테스트 ===")
    print("rotation_joint 가 -3.14 ~ 3.14 범위를 왕복 회전합니다.")
    print("실제 각도가 목표에 도달하면 방향을 바꿉니다.")
    
    target_positions = [0.0] * 8
    
    # 목표값 및 인덱스 설정
    joint_idx = 1
    limit_max = 3.14
    limit_min = -3.14
    current_target = limit_max
    target_positions[joint_idx] = current_target
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            # 실제 현재 각도 읽기
            actual_positions = adapter.read_arm_joints()
            actual_angle = actual_positions[joint_idx]
            
            # 목표 각도 도달 체크 (오차 0.05 라디안 이내)
            if abs(actual_angle - current_target) < 0.05:
                current_target = limit_min if current_target > 0 else limit_max
                target_positions[joint_idx] = current_target
            
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            adapter.control_arm_joints(target_positions)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0: time.sleep(time_until_next)

if __name__ == "__main__": main()
