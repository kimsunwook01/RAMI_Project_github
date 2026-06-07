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
    
    print("=== [Step 9] arm_joint_5 자동 왕복 테스트 ===")
    print("arm_joint_5 가 -180도(-3.14) ~ 180도(3.14) 범위를 왕복 회전합니다.")
    
    target_positions = [0.0] * 8
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        start_time = time.time()
        while viewer.is_running():
            step_start = time.time()
            
            elapsed = time.time() - start_time
            target_positions[6] = math.sin(elapsed) * 3.14
            
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            adapter.control_arm_joints(target_positions)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0: time.sleep(time_until_next)

if __name__ == "__main__": main()
