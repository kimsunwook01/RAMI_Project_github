import os
import sys
import time

# 루트 디렉토리를 sys.path에 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    
    print("=== [Step 16] 센서 부착 및 FOV 시각화 검증 테스트 ===")
    print("뷰어를 확인하여 다음 사항을 점검하세요:")
    print("1. 로봇 머리와 그리퍼 부근에 노란색 사각뿔(카메라 FOV)이 렌더링되는지")
    print("2. 로봇 베이스 주변 8방향으로 하늘색 원뿔(초음파 감지 범위)이 렌더링되는지")
    print("3. 시각화 도형들이 그림자를 발생시키거나 자체 음영(Shading)을 가지지 않는지")
    print("종료하려면 뷰어 창을 닫으세요.")
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        # 선임 에이전트의 조언에 따라 분리된 비전 전용 그룹(group="5")을 뷰어에서 활성화하여 보이게 만듭니다.
        viewer.opt.geomgroup[5] = 1
        
        while viewer.is_running():
            step_start = time.time()
            client.step()
            viewer.sync()
            
            # 실시간 동기화
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
