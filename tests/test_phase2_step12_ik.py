import os
import sys
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter
from src.application.services.manipulator_service import ManipulatorService
from src.domain.controllers.kinematics import KinematicsSolver

# 기본 타겟: 정면으로 약간 뻗은 초기 자세 근처
target_pos = [0.0, 0.4, 0.5]
target_rpy = [0.0, 0.0, 0.0]

def terminal_input_thread():
    global target_pos, target_rpy
    print("\n=== [Step 12] DLS IK 및 궤적 제어 수동 검증 ===")
    print("사용법: X Y Z R P Y 형식으로 6개의 숫자를 입력하세요. (단위: m, rad)")
    print("  예시: 0.0 0.5 0.5 0 0 0")
    print("※ X:좌우(음수=우), Y:앞뒤(양수=앞), Z:상하")
    print("※ 반드시 터미널 창을 클릭한 뒤 입력하세요.")
    print("=============================================\n")
    
    while True:
        try:
            val = input("목표 좌표(X Y Z R P Y)> ").strip()
            if not val: continue
            
            parts = val.split()
            if len(parts) != 6:
                print("입력 형식이 잘못되었습니다. 6개의 숫자를 공백으로 구분해 입력하세요.")
                continue
                
            x, y, z, r, p, yw = map(float, parts)
            target_pos = [x, y, z]
            target_rpy = [r, p, yw]
            print(f"✅ 목표 좌표 업데이트: POS({x}, {y}, {z}), RPY({r}, {p}, {yw})")
        except ValueError:
            print("입력 오류: 모두 숫자여야 합니다.")
        except Exception as e:
            print(f"알 수 없는 오류: {e}")

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    # 서비스 계층 초기화
    service = ManipulatorService(hardware=adapter, dt=client.model.opt.timestep)
    
    t = threading.Thread(target=terminal_input_thread, daemon=True)
    t.start()
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            
            target_rot_mat = KinematicsSolver.euler_to_rotation_matrix(*target_rpy)
            
            # IK 및 Trajectory 1스텝 구동
            error = service.step_ik_towards_pose(target_pos, target_rot_mat)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
