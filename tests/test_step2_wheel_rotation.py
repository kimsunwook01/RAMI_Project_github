import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    print("=== [Step 2] 바퀴 단독 회전 테스트 ===")
    print("로봇 본체는 정지한 상태에서, 4개의 바퀴가 끊김 없이 부드럽게 전진 방향으로 회전합니다.")
    print("종료하려면 뷰어 창을 닫으세요.")
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            # 본체는 고정
            adapter.move_base(0.0, 0.0, 0.0)
            
            # 4개 바퀴 모두 동일한 속도(예: 10 rad/s)로 회전
            # 축 방향에 따라 부호가 다를 수 있지만, 일단 + 방향으로 동일하게 인가
            adapter.control_wheels(10.0, 10.0, 10.0, 10.0)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
