import os
import sys
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

# 글로벌 속도 변수 [LF, LB, RF, RB]
cmd_wheels = [0.0, 0.0, 0.0, 0.0]

def terminal_input_thread():
    global cmd_wheels
    print("\n=== [Step 2-1] 휠 개별 컨트롤 모드 ===")
    print("사용법: [바퀴이름] [속도] 를 입력하고 엔터를 누르세요.")
    print("  바퀴이름: LF (좌전), LB (좌후), RF (우전), RB (우후), ALL (전체)")
    print("  예시: LF 20   (좌측 앞바퀴를 20 속도로 회전)")
    print("  예시: ALL 0   (모든 바퀴 정지)")
    print("========================================\n")
    
    while True:
        try:
            val = input("명령> ")
            parts = val.strip().split()
            if len(parts) == 2:
                target = parts[0].upper()
                speed = float(parts[1])
                
                if target == 'LF': cmd_wheels[0] = speed
                elif target == 'LB': cmd_wheels[1] = speed
                elif target == 'RF': cmd_wheels[2] = speed
                elif target == 'RB': cmd_wheels[3] = speed
                elif target == 'ALL': cmd_wheels = [speed, speed, speed, speed]
                else:
                    print("알 수 없는 바퀴 이름입니다. (LF, LB, RF, RB, ALL 중 택일)")
                    continue
                
                print(f"✅ [{target}] 바퀴의 목표 속도를 {speed} rad/s로 설정했습니다.")
            else:
                print("입력 형식이 잘못되었습니다. 예: LF 20")
        except ValueError:
            print("속도는 숫자여야 합니다. 예: LF 20")
        except Exception as e:
            print(f"입력 오류: {e}")

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    # 터미널 입력 스레드 시작
    t = threading.Thread(target=terminal_input_thread, daemon=True)
    t.start()
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            # 본체는 고정
            adapter.move_base(0.0, 0.0, 0.0)
            
            # 입력받은 개별 바퀴 속도 인가
            adapter.control_wheels(cmd_wheels[0], cmd_wheels[1], cmd_wheels[2], cmd_wheels[3])
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
