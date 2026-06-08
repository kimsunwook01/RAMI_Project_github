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

# [vx, vy, vz, wx, wy, wz]
current_v_cartesian = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
speed = 0.2  # 0.2 m/s

# 굽힌 자세(초기 자세) 설정: Lift, Rot, Arm1, Arm2(-45), Arm3(90), Arm4(-45), Arm5, Arm6
bent_pose = [0.0, 0.0, 0.0, -0.785398, 1.570796, -0.785398, 0.0, 0.0]

def terminal_input_thread(service: ManipulatorService):
    global current_v_cartesian
    print("\n=== [Phase 2-3] Step 13: 직교 공간 속도 제어(Jacobian) 검증 ===")
    print("사용법: 다음 명령어를 입력하세요.")
    print("  x+, x- : X축 전진/후진 (로봇 기준 좌/우)")
    print("  y+, y- : Y축 전진/후진 (로봇 기준 앞/뒤)")
    print("  z+, z- : Z축 전진/후진 (상/하)")
    print("  stop   : 모든 동작 정지")
    print("  reset  : 초기 자세로 초기화")
    print("============================================================\n")
    
    while True:
        try:
            val = input("명령 입력> ").strip().lower()
            if not val: continue
            
            v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            if val == "x+": v[0] = speed
            elif val == "x-": v[0] = -speed
            elif val == "y+": v[1] = speed
            elif val == "y-": v[1] = -speed
            elif val == "z+": v[2] = speed
            elif val == "z-": v[2] = -speed
            elif val == "stop":
                print("🛑 정지!")
            elif val == "reset":
                print("🔄 굽힌 초기 자세로 복귀 중...")
                service.cmd_joints = bent_pose.copy()
                service.trajectory.reset()
                v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else:
                print("알 수 없는 명령입니다.")
                continue
                
            current_v_cartesian = v
            if val != "stop":
                print(f"✅ 속도 적용됨: {val}")
                
        except Exception as e:
            print(f"알 수 없는 오류: {e}")

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    service = ManipulatorService(hardware=adapter, dt=client.model.opt.timestep)
    
    t = threading.Thread(target=terminal_input_thread, args=(service,), daemon=True)
    t.start()
    
    # 초기에 약간 굽힌 자세를 cmd_joints 로 지정 (특이점 회피)
    service.cmd_joints = bent_pose.copy()
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            
            # 카테시안 속도 제어 실행
            service.step_velocity_control(current_v_cartesian)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
