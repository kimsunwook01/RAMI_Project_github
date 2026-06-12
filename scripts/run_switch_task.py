import mujoco
import mujoco.viewer
import sys
import os
import cv2
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter
from src.application.services.manipulator_service import ManipulatorService
from src.infrastructure.sensors.vision_processor import VisionProcessor
from src.domain.robot.switch_task_controller import SwitchTaskController

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    print("Loading simulation environment...")
    model = mujoco.MjModel.from_xml_path(os.path.join(PROJECT_ROOT, "config/rami_description/rami_indoor_world.xml"))
    data = mujoco.MjData(model)
    
    # Spawn the robot at a good location near a switch.
    # Look at rami_indoor_world.xml for qr_switch_case locations:
    # qr_switch_case_1 is at 11.6, 10.0, 1.42
    # So we spawn at X=12.5, Y=10.0 facing the switch (which means facing -X)
    root_x_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
    root_y_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_y")
    root_rot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_z_rot")
    
    if root_x_id != -1:
        data.qpos[model.jnt_qposadr[root_x_id]] = 12.5
        data.qpos[model.jnt_qposadr[root_y_id]] = 10.0
        data.qpos[model.jnt_qposadr[root_rot_id]] = 0.0 # Facing -X or +X? Actually we will let it SEARCH by turning.

    hardware = RamiMujocoAdapter(model, data)
    manipulator = ManipulatorService(hardware, dt=model.opt.timestep)
    vision = VisionProcessor(model, data)
    
    controller = SwitchTaskController(hardware, manipulator, vision)
    
    mujoco.mj_forward(model, data)
    
    # Safe arm pose initially
    hardware.control_arm_joints([0]*8)
    
    cv2.namedWindow("Robot Vision", cv2.WINDOW_NORMAL)
    
    print("Starting Main Loop...")
    step_count = 0
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                # 1. 제어기 스텝 실행 (비전 처리, 스테이트 머신, 속도 명령 계산)
                bgr_img, qr_det = controller.step()
                
                # 2. 물리 엔진 스텝 진행 (여러 번 쪼개서 시뮬레이션 안정성 확보)
                for _ in range(10):
                    mujoco.mj_step(model, data)
                    
                # 뷰어 동기화 (3D 창 업데이트)
                viewer.sync()
                
                # 3. OpenCV 시각화 (Bounding Box 그리기)
                if bgr_img is not None:
                    if qr_det:
                        box = qr_det['bbox']
                        cv2.rectangle(bgr_img, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
                        cv2.circle(bgr_img, tuple(qr_det['center']), 5, (0, 0, 255), -1)
                        
                    # 화면 중앙선 표시
                    h, w = bgr_img.shape[:2]
                    cv2.line(bgr_img, (w//2, 0), (w//2, h), (255, 0, 0), 1)
                    
                    # State 텍스트 표시
                    cv2.putText(bgr_img, f"State: {controller.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    
                    cv2.imshow("Robot Vision", bgr_img)
                    key = cv2.waitKey(1)
                    if key == 27: # ESC
                        break
                        
                step_count += 1
                if controller.state == "DONE":
                    print("Task successfully completed!")
                    break
                    
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
