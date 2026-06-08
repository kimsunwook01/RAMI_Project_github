import time
import os
import sys
import numpy as np
import mujoco
import mujoco.viewer

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.application.services.sensor_service import SensorService

def main():
    print("=== [Step 18-1] 전방 5m 임의 객체 탐지 검증 테스트 ===")
    
    xml_path = os.path.join(os.path.dirname(__file__), "..", "config", "rami_description", "rami_world.xml")
    xml_path = os.path.abspath(xml_path)
    client = MujocoClient(xml_path)
    
    sensor_service = SensorService(client.model)
    print("[시스템] 뷰어를 실행합니다. 로봇이 정면 5m 앞의 붉은 원기둥과 푸른 상자를 응시합니다.")
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        viewer.opt.geomgroup[5] = 1  # 시야각(FOV) 시각화 도형 활성화
        
        last_print_time = time.time()
        
        while viewer.is_running():
            mujoco.mj_step(client.model, client.data)
            viewer.sync()
            
            current_time = time.time()
            if current_time - last_print_time >= 1.0:
                state = sensor_service.get_sensor_state(client.data)
                
                print("\n" + "="*60)
                print(f"[시뮬레이션 시간] {client.data.time:.2f} s")
                
                print("\n--- [ Vision (YOLOv8) ] ---")
                head_det = state['vision']['head_camera_detections']
                if head_det:
                    print(f"✅ Head Camera  : {len(head_det)}개 객체 포착됨!")
                    for d in head_det:
                        print(f"  └─ 인식 결과: {d['class']} (신뢰도: {d['confidence']*100:.1f}%)")
                else:
                    print("❌ Head Camera  : 포착된 객체 없음 (객체가 시야를 벗어났거나 YOLO 인식 실패)")
                    
                print("="*60)
                
                last_print_time = current_time

if __name__ == "__main__":
    main()
