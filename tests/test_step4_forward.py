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
    
    # 메카넘 휠 역기구학 제어기 초기화
    mecanum = MecanumController(wheel_radius=0.05, wheel_base=0.2, track_width=0.3)
    
    print("=== [Step 4] 전진 주행 동기화 테스트 ===")
    print("가상 관절의 물리적 전진(0.3 m/s)과 메카넘 제어기가 계산한")
    print("4개 바퀴의 전진 회전이 완벽하게 동기화되어 주행합니다.")
    print("종료하려면 뷰어 창을 닫으세요.")
    
    # 목표 속도: 전진 0.3 m/s
    vx, vy, wz = 0.3, 0.0, 0.0
    
    # 역기구학 연산
    w_lf, w_lb, w_rf, w_rb = mecanum.compute_wheel_velocities(vx, vy, wz)
    
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            # 동체 가상 관절 이동 명령
            adapter.move_base(vx, vy, wz)
            # 바퀴 회전 속도 인가
            adapter.control_wheels(w_lf, w_lb, w_rf, w_rb)
            
            client.step()
            viewer.sync()
            
            time_until_next = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()
