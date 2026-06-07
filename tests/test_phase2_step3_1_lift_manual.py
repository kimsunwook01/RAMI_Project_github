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
    print("\n=== [Step 3-1] 리프트 수동 입력 제어 ===")
    print("리프트 이동 가능 범위: -0.62 ~ 0.62 (단위: 미터)")
    print("사용법: 원하는 리프트의 높이 값만 입력하고 [엔터]를 누르세요. (예: 0.5)")
    print("※ 반드시 이 '터미널 창'을 클릭한 뒤 입력하셔야 합니다.")
    print("========================================\n")
    
    while True:
        try:
            val = input("목표 높이> ").strip()
            if not val: continue
            
            height = float(val)
            if height < -0.62 or height > 0.62:
                print("경고: 허용 범위를 초과했습니다! (-0.62 ~ 0.62 이내로 입력하세요)")
                continue
                
            target_positions[0] = height
            print(f"✅ 리프트 목표 높이가 {height}m 로 변경되었습니다.")
        except ValueError:
            print("입력 오류: 숫자를 입력해주세요. (예: -0.2)")
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
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
