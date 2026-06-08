import time
import os
import sys
import numpy as np
import mujoco
import mujoco.viewer

# 프로젝트 루트 디렉토리를 sys.path에 추가하여 src 모듈을 임포트할 수 있도록 합니다.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.application.services.sensor_service import SensorService

def main():
    print("=== [Step 18] 센서 연동 통합 테스트 및 스트리밍 검증 ===")
    
    # 1. 시뮬레이터(XML) 로드
    xml_path = os.path.join(os.path.dirname(__file__), "..", "config", "rami_description", "rami_world.xml")
    xml_path = os.path.abspath(xml_path)
    client = MujocoClient(xml_path)
    
    # 2. SensorService 초기화
    print("\n[시스템] YOLOv8 모델 다운로드 및 렌더러 파이프라인 초기화 중...")
    print("[시스템] 네트워크 상태에 따라 최초 로딩 시 몇 초가 소요될 수 있습니다.")
    sensor_service = SensorService(client.model)
    print("[시스템] 파사드 초기화 완료! 뷰어를 실행합니다.")
    
    # 3. 메인 시뮬레이션 루프
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        # 시각화 지오메트리 그룹(FOV 도형들) 렌더링 활성화
        viewer.opt.geomgroup[5] = 1
        
        last_print_time = time.time()
        
        while viewer.is_running():
            # 물리 엔진 스텝 전진
            mujoco.mj_step(client.model, client.data)
            viewer.sync()
            
            # 1초 주기로 센서 상태를 터미널에 깔끔하게 출력
            current_time = time.time()
            if current_time - last_print_time >= 1.0:
                # 파사드에서 센서 통합 상태 한번에 가져오기
                state = sensor_service.get_sensor_state(client.data)
                
                print("\n" + "="*60)
                print(f"[시뮬레이션 시간] {client.data.time:.2f} s")
                
                print("\n--- [ IMU (Odometry) ] ---")
                print(f"상대 회전각(Yaw) : {np.rad2deg(state['imu']['yaw_heading']):.2f} 도")
                print(f"가속도(X, Y, Z)  : {[round(x, 2) for x in state['imu']['accel']]}")
                print(f"각속도(X, Y, Z)  : {[round(x, 2) for x in state['imu']['gyro']]}")
                
                print("\n--- [ Ultrasonic ] ---")
                print(f"가장 가까운 장애물: {state['ultrasonic']['min_dist']:.2f} m")
                
                print("\n--- [ Vision (YOLOv8) ] ---")
                head_det = state['vision']['head_camera_detections']
                if head_det:
                    print(f"Head Camera  : {len(head_det)}개 객체 포착됨")
                    for d in head_det:
                        print(f"  └─ {d['class']} (신뢰도: {d['confidence']*100:.1f}%)")
                else:
                    print("Head Camera  : 포착된 객체 없음")
                    
                gripper_det = state['vision']['gripper_camera_detections']
                if gripper_det:
                    print(f"Gripper Cam  : {len(gripper_det)}개 객체 포착됨")
                    for d in gripper_det:
                        print(f"  └─ {d['class']} (신뢰도: {d['confidence']*100:.1f}%)")
                else:
                    print("Gripper Cam  : 포착된 객체 없음")
                    
                print("="*60)
                
                last_print_time = current_time

if __name__ == "__main__":
    main()
