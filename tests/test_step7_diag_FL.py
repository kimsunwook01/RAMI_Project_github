import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter
from src.domain.controllers.mecanum_controller import MecanumController

def main():
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    mecanum = MecanumController()
    
    print("=== [Step 7] 좌측 전진 대각 이동 ===")
    print("우하단(RB)과 좌상단(LF) 바퀴만 회전하여 대각선으로 이동합니다.")
    
    vx, vy, wz = 0.3, 0.3, 0.0
    w_lf, w_lb, w_rf, w_rb = mecanum.compute_wheel_velocities(vx, vy, wz)
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            adapter.move_base(vx, vy, wz)
            adapter.control_wheels(w_lf, w_lb, w_rf, w_rb)
            client.step()
            viewer.sync()
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0: time.sleep(time_until_next)

if __name__ == "__main__": main()
