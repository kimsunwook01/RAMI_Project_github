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
    
    print("=== [Step 5] arm_joint_1 피드백 기반 자동 왕복 테스트 ===")
    
    target_positions = [0.0] * 8
    
    joint_idx = 2
    limit_max = 1.57
    limit_min = -1.57
    current_target = limit_max
    target_positions[joint_idx] = current_target
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            actual_positions = adapter.read_arm_joints()
            actual_angle = actual_positions[joint_idx]
            
            if abs(actual_angle - current_target) < 0.05:
                current_target = limit_min if current_target > 0 else limit_max
                target_positions[joint_idx] = current_target
            
            adapter.move_base(0.0, 0.0, 0.0)
            adapter.control_wheels(0.0, 0.0, 0.0, 0.0)
            adapter.control_arm_joints(target_positions)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0: time.sleep(time_until_next)

if __name__ == "__main__": main()
