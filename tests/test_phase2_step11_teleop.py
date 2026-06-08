import os
import sys
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

# 글로벌 목표 위치 배열: [lift, rot, arm1, arm2, arm3, arm4, arm5, arm6]
target_positions = [0.0] * 8

def terminal_input_thread():
    global target_positions
    print("\n=== [Step 11] 7축 통합 터미널 수동 제어 테스트 ===")
    print("조인트 이동 가능 범위 (단위: 미터 또는 라디안):")
    print("  LIFT: -0.62 ~ 0.62")
    print("  ROT: -3.14 ~ 3.14")
    print("  ARM1~4: -1.57 ~ 1.57")
    print("  ARM5: -3.14 ~ 3.14")
    print("  ARM6: 0.0 ~ 1.57")
    print("사용법: [조인트이름] [목표값] 을 입력하고 [엔터]를 누르세요.")
    print("  예: LIFT 0.5")
    print("  예: ROT 1.57")
    print("  예: ARM1 -0.5")
    print("※ 반드시 이 '터미널 창'을 클릭한 뒤 입력하셔야 합니다.")
    print("========================================\n")
    
    limits = {
        'LIFT': (-0.62, 0.62, 0),
        'ROT': (-3.14, 3.14, 1),
        'ARM1': (-1.57, 1.57, 2),
        'ARM2': (-1.57, 1.57, 3),
        'ARM3': (-1.57, 1.57, 4),
        'ARM4': (-1.57, 1.57, 5),
        'ARM5': (-3.14, 3.14, 6),
        'ARM6': (0.0, 1.57, 7),
    }
    
    while True:
        try:
            val = input("명령> ").strip().upper()
            if not val: continue
            
            parts = val.split()
            if len(parts) != 2:
                print("입력 형식이 잘못되었습니다. 예: ARM1 1.0")
                continue
                
            joint_name = parts[0]
            target_val = float(parts[1])
            
            if joint_name not in limits:
                print(f"알 수 없는 조인트 이름입니다: {joint_name} (LIFT, ROT, ARM1~6 중 하나여야 함)")
                continue
                
            min_val, max_val, idx = limits[joint_name]
            
            if target_val < min_val or target_val > max_val:
                print(f"경고: {joint_name}의 허용 범위를 초과했습니다! ({min_val} ~ {max_val} 이내로 입력하세요)")
                continue
                
            target_positions[idx] = target_val
            print(f"✅ {joint_name} 조인트의 목표 위치가 {target_val} 로 변경되었습니다.")
        except ValueError:
            print("입력 오류: 목표값은 숫자여야 합니다. (예: LIFT 0.5)")
        except Exception as e:
            print(f"알 수 없는 오류: {e}")

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    t = threading.Thread(target=terminal_input_thread, daemon=True)
    t.start()
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            adapter.control_arm_joints(target_positions)
            
            client.step()
            viewer.sync()
            
            # 피드백 출력을 원하는 경우 주석 해제
            # actual = adapter.read_arm_joints()
            # print(f"Actual LIFT: {actual[0]:.2f}, ARM1: {actual[2]:.2f}", end='\r')
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
