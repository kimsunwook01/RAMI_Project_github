import os
import sys
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

# 글로벌 타겟 (사용자 입력)
target_pos = [0.0, 0.4, 0.5]

def terminal_input_thread():
    global target_pos
    print("\n=== [Phase 2-3] Step 12: FK 및 타겟 마커 검증 ===")
    print("사용법: X Y Z 형식으로 3개의 숫자를 입력하세요. (단위: m)")
    print("목표: 빨간 반투명 마커가 입력한 위치로 이동하는지 확인합니다.")
    print("※ X:좌우(음수=우), Y:앞뒤(양수=앞), Z:상하")
    print("※ 반드시 터미널 창을 클릭한 뒤 입력하세요.")
    print("=============================================\n")
    
    while True:
        try:
            val = input("목표 좌표(X Y Z)> ").strip()
            if not val: continue
            
            parts = val.split()
            if len(parts) != 3:
                print("입력 형식이 잘못되었습니다. 3개의 숫자를 공백으로 구분해 입력하세요.")
                continue
                
            x, y, z = map(float, parts)
            target_pos = [x, y, z]
            print(f"✅ 타겟 마커 업데이트: POS({x}, {y}, {z})")
        except ValueError:
            print("입력 오류: 모두 숫자여야 합니다.")
        except Exception as e:
            print(f"알 수 없는 오류: {e}")

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    t = threading.Thread(target=terminal_input_thread, daemon=True)
    t.start()
    
    # target_marker 인덱스 찾기 (mocap)
    mocap_id = mujoco.mj_name2id(client.model, mujoco.mjtObj.mjOBJ_BODY, "target_marker")
    if mocap_id == -1:
        print("에러: target_marker를 rami_world.xml에서 찾을 수 없습니다.")
        return
    
    # 해당 바디의 mocap id 추출
    mocap_idx = client.model.body_mocapid[mocap_id]

    # 출력을 너무 자주 하지 않기 위한 타이머
    last_print_time = time.time()
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            # 베이스 및 휠 정지 유지
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            
            # 리프트와 암은 현재 초기 자세(0) 유지
            adapter.control_arm_joints([0.0]*8)
            
            # 시각적 마커(target_marker) 좌표 업데이트 (mocap 바디 제어)
            client.data.mocap_pos[mocap_idx] = target_pos
            
            client.step()
            viewer.sync()
            
            # 현재 End-effector 의 순운동학 좌표를 1초마다 출력
            curr_time = time.time()
            if curr_time - last_print_time > 1.0:
                ee_pos, _ = adapter.get_end_effector_pose()
                print(f"🔄 현재 그리퍼 좌표(FK): X={ee_pos[0]:.3f}, Y={ee_pos[1]:.3f}, Z={ee_pos[2]:.3f}")
                last_print_time = curr_time
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
