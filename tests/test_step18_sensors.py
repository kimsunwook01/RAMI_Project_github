import os
import sys
import time
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.application.services.sensor_service import SensorService

def print_sensor_loop(service: SensorService, stop_event: threading.Event):
    while not stop_event.is_set():
        state = service.get_sensor_state()
        
        # 윈도우/리눅스 터미널 클리어
        os.system('cls' if os.name == 'nt' else 'clear')
        
        print("=== [Phase 3] Step 18: 센서 연동 통합 테스트 ===")
        print("로봇 관절을 마우스로 드래그하며 센서값이 변하는지 확인하세요.\n")
        
        print(f"[초음파 센서 (Ultrasonic)]")
        print(f" - 최단 장애물 거리: {state['ultrasonic_min_dist']:.2f} m")
        print(f" - 개별 8채널 거리: {[round(d, 2) for d in state['ultrasonic_all']]}\n")
        
        imu = state["imu"]
        print(f"[관성 센서 (IMU)]")
        print(f" - 가속도(Accel): X={imu['accel'][0]:.2f}, Y={imu['accel'][1]:.2f}, Z={imu['accel'][2]:.2f}")
        print(f" - 자이로(Gyro) : X={imu['gyro'][0]:.2f}, Y={imu['gyro'][1]:.2f}, Z={imu['gyro'][2]:.2f}\n")
        
        cam = state["gripper_camera"]
        if "error" not in cam:
            pos = cam["pos"]
            fwd = cam["forward_vector"]
            print(f"[그리퍼 카메라 (gripper_camera)]")
            print(f" - 전역 좌표: X={pos[0]:.2f}, Y={pos[1]:.2f}, Z={pos[2]:.2f}")
            print(f" - 시선 벡터: X={fwd[0]:.2f}, Y={fwd[1]:.2f}, Z={fwd[2]:.2f}")
            
        time.sleep(0.1)

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    service = SensorService(client.model, client.data)
    
    stop_event = threading.Event()
    t = threading.Thread(target=print_sensor_loop, args=(service, stop_event), daemon=True)
    t.start()
    
    print("뷰어를 실행합니다...")
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            client.step()
            viewer.sync()
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)
                
    stop_event.set()
    t.join()

if __name__ == "__main__":
    main()
