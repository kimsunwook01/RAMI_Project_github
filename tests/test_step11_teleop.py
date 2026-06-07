import os
import sys
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter
from src.domain.controllers.mecanum_controller import MecanumController

# 목표 속도 [vx, vy, wz]
cmd_vel = [0.0, 0.0, 0.0]

def terminal_input_thread():
    global cmd_vel
    print("\n=== [Step 11] 통합 텔레오퍼레이션 뷰어 ===")
    print("사용법: 아래 키를 입력하고 [엔터]를 누르세요.")
    print("  W: 전진   |  S: 정지   |  X: 후진")
    print("  A: 좌이동 |            |  D: 우이동")
    print("  Q: 좌상단 |            |  E: 우상단")
    print("  Z: 좌하단 |            |  C: 우하단")
    print("  R: 반시계회전 | T: 시계회전")
    print("※ 반드시 이 '터미널 창'을 클릭한 뒤 입력하셔야 합니다.")
    print("========================================\n")
    
    speed = 0.3 # 이동 속도 m/s
    rot_speed = 1.0 # 회전 속도 rad/s
    
    while True:
        try:
            val = input("명령키> ").strip().upper()
            if not val: continue
            
            # W, A, S, D, Q, E, Z, C, R, T
            if val == 'W': cmd_vel = [speed, 0.0, 0.0]
            elif val == 'X': cmd_vel = [-speed, 0.0, 0.0]
            elif val == 'A': cmd_vel = [0.0, speed, 0.0]
            elif val == 'D': cmd_vel = [0.0, -speed, 0.0]
            elif val == 'Q': cmd_vel = [speed, speed, 0.0]
            elif val == 'E': cmd_vel = [speed, -speed, 0.0]
            elif val == 'Z': cmd_vel = [-speed, speed, 0.0]
            elif val == 'C': cmd_vel = [-speed, -speed, 0.0]
            elif val == 'R': cmd_vel = [0.0, 0.0, rot_speed]
            elif val == 'T': cmd_vel = [0.0, 0.0, -rot_speed]
            elif val == 'S': cmd_vel = [0.0, 0.0, 0.0]
            else:
                print("알 수 없는 키입니다. (W,A,S,D,X,Q,E,Z,C,R,T 중 택일)")
                continue
            
            print(f"✅ 주행 명령 변경: vx={cmd_vel[0]}, vy={cmd_vel[1]}, wz={cmd_vel[2]}")
        except Exception as e:
            print(f"입력 오류: {e}")

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    mecanum = MecanumController()
    
    t = threading.Thread(target=terminal_input_thread, daemon=True)
    t.start()
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            vx, vy, wz = cmd_vel
            w_lf, w_lb, w_rf, w_rb = mecanum.compute_wheel_velocities(vx, vy, wz)
            
            adapter.move_base(vx, vy, wz)
            adapter.control_wheels(w_lf, w_lb, w_rf, w_rb)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
