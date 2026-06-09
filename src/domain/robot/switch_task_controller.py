import math
import numpy as np
import mujoco
from src.domain.controllers.kinematics import KinematicsSolver

class SwitchTaskController:
    def __init__(self, hardware_io, manipulator_service, vision_processor):
        self.hw = hardware_io
        self.manipulator = manipulator_service
        self.vision = vision_processor
        
        self.state = "SEARCH"
        self.target_qr_data = None
        self.target_3d_pos = None
        
        self.screen_w = self.vision.width
        self.screen_h = self.vision.height
        
        # 튜닝 가능한 파라미터
        self.turn_speed = 0.5
        self.forward_speed = 0.5
        self.center_tol = 40  # 픽셀 오차 허용 범위
        self.target_bbox_area = 15000  # 접근 완료로 판단할 QR 코드 넓이 (가까워질수록 커짐)
        self.wait_steps = 0
        
    def step(self, data, model):
        # 1. 비전 센서 업데이트
        detections, bgr_img = self.vision.process_camera(data, "head_camera", detect_yolo=False, detect_qr=True)
        
        qr_det = None
        if detections:
            # 임의의 QR 코드 하나를 타겟으로 잡음
            qr_det = detections[0]
            
        # 2. 로봇의 현재 회전각 (theta) 
        root_rot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_z_rot")
        theta = data.qpos[model.jnt_qposadr[root_rot_id]] if root_rot_id != -1 else 0.0
        
        vx, vy, wz = 0.0, 0.0, 0.0
        
        # 3. State Machine
        if self.state == "SEARCH":
            if qr_det is None:
                # 못 찾았으면 계속 좌회전
                wz = self.turn_speed
            else:
                print(f"[SEARCH] QR Code Found! Center: {qr_det['center']}")
                self.state = "APPROACH"
                
        elif self.state == "APPROACH":
            if qr_det is None:
                # 시야에서 놓침
                print("[APPROACH] Lost QR Code. Reverting to SEARCH.")
                self.state = "SEARCH"
            else:
                cx, cy = qr_det['center']
                bbox = qr_det['bbox']
                area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                
                err_x = cx - (self.screen_w / 2)
                
                if abs(err_x) > self.center_tol:
                    # 화면 중앙에 오도록 회전
                    wz = -self.turn_speed if err_x > 0 else self.turn_speed
                else:
                    # 화면 중앙에 얼추 맞으면 전진
                    if area < self.target_bbox_area:
                        # 로컬 전진 방향은 X축 (1, 0) 이라고 가정하고 글로벌 회전
                        # 헤드 카메라가 -X 방향을 보고 있으므로, -X가 전진 방향일 수 있음.
                        # rami_world.xml: <camera name="head_camera" xyaxes="-1 0 0 0 0 1" /> -> Z축이 0 0 1, X축이 -1 0 0 
                        # 따라서 로컬 -X가 카메라 정면임. 전진은 로컬 -X.
                        local_fwd_x = -1.0
                        local_fwd_y = 0.0
                        
                        vx = (local_fwd_x * math.cos(theta) - local_fwd_y * math.sin(theta)) * self.forward_speed
                        vy = (local_fwd_x * math.sin(theta) + local_fwd_y * math.cos(theta)) * self.forward_speed
                    else:
                        print(f"[APPROACH] Arrived at target! Area: {area}")
                        self.state = "ALIGN_ARM"
                        self.wait_steps = 50 # 대기 후 팔 움직임 시작
                        
        elif self.state == "ALIGN_ARM":
            # 베이스 정지
            self.wait_steps -= 1
            if self.wait_steps <= 0:
                print("[ALIGN_ARM] Starting Manipulation!")
                # 현재 위치에서 로봇 앞쪽(-X 방향)으로 약간 높이를 올려서 팔을 뻗음
                # 실제로는 그리퍼 카메라나 뎁스를 써야하지만, 데모용으로는 로봇 좌표계 기준 고정 오프셋 타격
                # 글로벌 좌표계 변환
                offset_x, offset_y, offset_z = -0.7, 0.0, 1.4 # 로컬 -0.7m 전방, 1.4m 높이
                
                root_x_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
                root_y_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_y")
                rx = data.qpos[model.jnt_qposadr[root_x_id]]
                ry = data.qpos[model.jnt_qposadr[root_y_id]]
                
                target_x = rx + (offset_x * math.cos(theta) - offset_y * math.sin(theta))
                target_y = ry + (offset_x * math.sin(theta) + offset_y * math.cos(theta))
                target_z = offset_z
                
                # 타겟 방향은 로봇이 바라보는 방향의 반대(-X)
                # Roll=0, Pitch=0, Yaw=theta + pi
                target_rot = KinematicsSolver.euler_to_rotation_matrix(0, 0, theta + math.pi)
                
                self.manipulator.solve_global_ik([target_x, target_y, target_z], target_rot)
                self.state = "PRESS"
                
        elif self.state == "PRESS":
            # solve_global_ik가 설정한 global_target_joints로 이동 (step_cartesian_target 등을 활용하거나 기본 PID 의존)
            # 여기서는 단순히 안전 자세에서 목표 자세로 IK 1스텝씩 접근
            if hasattr(self.manipulator, 'global_target_joints'):
                self.manipulator.hardware.control_arm_joints(self.manipulator.global_target_joints)
                
            # 팔이 움직일 충분한 시간을 기다린 후 DONE
            self.wait_steps += 1
            if self.wait_steps > 200:
                print("[PRESS] Task Done!")
                self.state = "DONE"
                
        elif self.state == "DONE":
            pass # 모든 작업 완료
            
        # 베이스 속도 명령 하달
        if self.state in ["SEARCH", "APPROACH"]:
            self.hw.move_base(vx, vy, wz)
        else:
            self.hw.move_base(0, 0, 0)
            
        return bgr_img, qr_det
