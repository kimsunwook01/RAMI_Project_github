import os
import sys
import time
import math

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    print("=== [Step 3] 리프트 자동 상하 왕복 이동 테스트 ===")
    print("팔을 앞으로 뻗은 자세(0.0)를 유지한 채로 리프트가 상하(-0.62 ~ 0.62)로 자동 왕복 이동합니다.")
    print("뷰어 창을 닫으면 종료됩니다.")
    
    target_positions = [0.0] * 8
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        start_time = time.time()
        while viewer.is_running():
            step_start = time.time()
            
            elapsed = time.time() - start_time
            # 리프트 높이를 사인파를 사용하여 -0.62 ~ 0.62 사이로 부드럽게 변경
            lift_z = math.sin(elapsed) * 0.6
            target_positions[0] = lift_z
            
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            adapter.control_arm_joints(target_positions)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
