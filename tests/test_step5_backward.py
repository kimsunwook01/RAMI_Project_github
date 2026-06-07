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
    mecanum = MecanumController(wheel_radius=0.05, wheel_base=0.2, track_width=0.3)
    
    print("=== [Step 5] 단방향 후진 구동 동기화 테스트 ===")
    print("로봇이 후방(-0.3 m/s)으로 이동하며 바퀴가 뒤로 회전합니다.")
    print("종료하려면 뷰어 창을 닫으세요.")
    
    # 후방 이동이므로 전진(vx)에 음수 입력
    vx, vy, wz = -0.3, 0.0, 0.0
    w_lf, w_lb, w_rf, w_rb = mecanum.compute_wheel_velocities(vx, vy, wz)
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            adapter.move_base(vx, vy, wz)
            adapter.control_wheels(w_lf, w_lb, w_rf, w_rb)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
