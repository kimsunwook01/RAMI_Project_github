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
    
    print("=== [Step 1] 지면 밀착 렌더링 테스트 ===")
    print("뷰어를 확인하여 바퀴가 공중에 뜨거나 바닥에 파묻히지 않고 완벽히 닿아 있는지 점검하세요.")
    print("종료하려면 뷰어 창을 닫으세요.")
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
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
