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

# 굽힌 자세(초기 자세)
bent_pose = [0.0, 0.0, 0.0, -0.785398, 1.570796, -0.785398, 0.0, 0.0]

# 현재의 타겟 위치 (루프에서 추적)
target_pos = [0.0, 0.4, 0.5]
target_rpy = [0.0, 0.0, 0.0]

def terminal_input_thread(service: ManipulatorService):
    global target_pos
    print("\n=== [Phase 2-3] Step 15: Cartesian Waypoint Interpolation 최종 데모 ===")
    print("사용법: X Y Z 형식으로 목표 좌표를 입력하세요. (단위: m)")
    print("특징: 타겟 마커(빨간 구체)가 이동하고, 로봇 팔이 완벽한 직선 궤적으로 타겟을 쫓아갑니다.")
    print("※ X: 좌/우 (음수=오른쪽, 양수=왼쪽)")
    print("※ Y: 앞/뒤 (★★ 음수(-)가 로봇 정면입니다! 예: -0.5 ★★)")
    print("※ Z: 상/하 (양수=위)")
    print("※ 'reset' 입력 시 굽힌 자세로 즉시 복귀")
    print("========================================================================\n")
    
    while True:
        try:
            val = input("목표 좌표(X Y Z) 또는 'reset'> ").strip().lower()
            if not val: continue
            
            if val == 'reset':
                service.cmd_joints = bent_pose.copy()
                service.trajectory.reset()
                print("🔄 자세 초기화!")
                continue
                
            parts = val.split()
            if len(parts) != 3:
                print("입력 형식이 잘못되었습니다. 3개의 숫자를 공백으로 구분해 입력하세요.")
                continue
                
            x, y, z = map(float, parts)
            target_pos = [x, y, z]
            print(f"✅ 타겟 변경: POS({x}, {y}, {z}) -> 로봇이 직선 이동을 시작합니다!")
        except ValueError:
            print("입력 오류: 모두 숫자여야 합니다.")
        except Exception as e:
            print(f"알 수 없는 오류: {e}")

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    service = ManipulatorService(hardware=adapter, dt=client.model.opt.timestep)
    
    # target_marker (mocap) 인덱스 찾기
    mocap_id = mujoco.mj_name2id(client.model, mujoco.mjtObj.mjOBJ_BODY, "target_marker")
    if mocap_id != -1:
        mocap_idx = client.model.body_mocapid[mocap_id]
    else:
        mocap_idx = -1
        
    t = threading.Thread(target=terminal_input_thread, args=(service,), daemon=True)
    t.start()
    
    service.cmd_joints = bent_pose.copy()
    target_rot_mat = KinematicsSolver.euler_to_rotation_matrix(*target_rpy)
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            
            # 시각적 마커 위치 갱신
            if mocap_idx != -1:
                client.data.mocap_pos[mocap_idx] = target_pos
                
            # Cartesian Waypoint Interpolation (직선 궤적 이동)
            # 기존 0.15 에서 3배 빠른 0.45로 상향 조정
            service.step_cartesian_target(target_pos, target_rot_mat, max_cartesian_vel=0.45)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
