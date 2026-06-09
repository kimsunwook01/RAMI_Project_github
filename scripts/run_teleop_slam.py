import os
import sys
import time
import threading
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter
from src.domain.controllers.mecanum_controller import MecanumController
from src.application.services.sensor_service import SensorService
from src.application.use_cases.slam_mapping_use_case import SlamMappingUseCase

# 목표 속도 [vx, vy, wz]
cmd_vel = [0.0, 0.0, 0.0]

def terminal_input_thread():
    global cmd_vel
    print("\n=== [Phase 4] 실내 SLAM 텔레오퍼레이션 ===")
    print("사용법: 아래 키를 입력하고 [엔터]를 누르세요.")
    print("  W: 전진   |  S: 정지   |  X: 후진")
    print("  A: 좌이동 |            |  D: 우이동")
    print("  Q: 좌상단 |            |  E: 우상단")
    print("  Z: 좌하단 |            |  C: 우하단")
    print("  R: 반시계회전 | T: 시계회전")
    print("※ 반드시 이 '터미널 창'을 클릭한 뒤 입력하셔야 합니다.")
    print("========================================\n")
    
    speed = 0.5 # 이동 속도 m/s
    rot_speed = 1.0 # 회전 속도 rad/s
    
    while True:
        try:
            val = input("명령키> ").strip().upper()
            if not val: continue
            
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
                print("알 수 없는 키입니다.")
                continue
        except Exception as e:
            print(f"입력 오류: {e}")

def main():
    xml_path = "config/rami_description/rami_phase4_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    mecanum = MecanumController()
    sensor_service = SensorService(client.model)
    slam_uc = SlamMappingUseCase(client.model, map_width=30.0, map_height=30.0, resolution=0.1)
    
    # 거실 전등 1 조인트 좌표 (11.0, 16.0)를 기준으로 스폰 위치 설정 (Ground Truth)
    try:
        root_x_id = mujoco.mj_name2id(client.model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
        root_y_id = mujoco.mj_name2id(client.model, mujoco.mjtObj.mjOBJ_JOINT, "root_y")
        qpos_x_adr = client.model.jnt_qposadr[root_x_id]
        qpos_y_adr = client.model.jnt_qposadr[root_y_id]
        client.data.qpos[qpos_x_adr] = 11.0
        client.data.qpos[qpos_y_adr] = 16.0
        
        # 팔이 몸통을 뚫지 않도록 기본 접은 자세로 초기화 및 홀드 (ctrl 설정)
        # lift, rot, arm1, arm2, arm3, arm4, arm5, arm6
        safe_arm_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 1. 시뮬레이션 시작 시 물리적 충돌 방지를 위해 qpos 직접 초기화
        arm_joints = [
            "lift_slide_joint", "rotation_joint",
            "arm_joint_1", "arm_joint_2", "arm_joint_3", 
            "arm_joint_4", "arm_joint_5", "arm_joint_6"
        ]
        for j_name, angle in zip(arm_joints, safe_arm_pose):
            j_id = mujoco.mj_name2id(client.model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
            if j_id != -1:
                q_adr = client.model.jnt_qposadr[j_id]
                client.data.qpos[q_adr] = angle
                
        # 2. 중력에 의해 떨어지지 않도록 Actuator ctrl 값 설정 (Position Control)
        adapter.control_arm_joints(safe_arm_pose)
                
        mujoco.mj_forward(client.model, client.data)
        print("로봇이 거실 전등 좌표(11.0, 16.0)에 성공적으로 배치되었습니다.")
    except Exception as e:
        print("초기화 설정 오류:", e)
    
    t = threading.Thread(target=terminal_input_thread, daemon=True)
    t.start()
    
    # 시각화 설정 (Matplotlib non-blocking)
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    im = ax.imshow(np.zeros((300, 300)), cmap='gray', origin='lower', vmin=0.0, vmax=1.0)
    plt.title("RAMI SLAM Map (Occupancy Grid)")
    
    step_count = 0
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            vx, vy, wz = cmd_vel
            w_lf, w_lb, w_rf, w_rb = mecanum.compute_wheel_velocities(vx, vy, wz)
            
            adapter.move_base(vx, vy, wz)
            adapter.control_wheels(w_lf, w_lb, w_rf, w_rb)
            
            client.step()
            
            # 10 스텝마다 SLAM 업데이트 실행 (성능 최적화)
            step_count += 1
            if step_count % 10 == 0:
                slam_uc.execute(client.data, sensor_service)
                
            # 100 스텝마다 맵 시각화 업데이트
            if step_count % 100 == 0:
                prob_map = slam_uc.get_map()
                im.set_data(prob_map)
                fig.canvas.draw_idle()
                plt.pause(0.001)
                
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

    # 프로그램 종료 시 최종 맵 저장
    prob_map = slam_uc.get_map()
    plt.imsave("map_output.png", prob_map, cmap='gray', origin='lower')
    print("지도 저장이 완료되었습니다: map_output.png")

if __name__ == "__main__":
    main()
