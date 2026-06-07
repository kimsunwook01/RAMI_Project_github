from src.application.interfaces.robot_hardware_io import RobotHardwareIO
import mujoco

class RamiMujocoAdapter(RobotHardwareIO):
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        
        # Actuator ID들을 미리 캐싱하여 검색 오버헤드를 줄입니다.
        self.vx_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_vx")
        self.vy_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_vy")
        self.wz_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_wz")
        
        self.w_lf_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "weel_lf")
        self.w_lb_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "weel_lb")
        self.w_rf_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "weel_rf")
        self.w_rb_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "weel_rb")
        
        # Arm & Lift Actuators
        self.arm_actuator_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_lift"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_rot"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_arm1"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_arm2"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_arm3"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_arm4"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_arm5"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_arm6")
        ]

    def move_base(self, vx: float, vy: float, wz: float) -> None:
        """
        MJCF에 정의된 velocity 액추에이터를 통해 동체의 가상 관절을 제어합니다.
        로봇 표준 좌표계 (vx: 전진, vy: 좌측, wz: 반시계 회전)를 하드웨어 축에 맞게 변환합니다.
        - RAMI 모델 기준: +X축은 좌측, +Y축은 후방을 향합니다.
        """
        # root_x 액추에이터 (base_vx): 좌우 이동 (+X가 좌측)
        if self.vx_id != -1: self.data.ctrl[self.vx_id] = vy
        # root_y 액추에이터 (base_vy): 전후 이동 (+Y가 후방이므로 전진 vx는 -로 인가)
        if self.vy_id != -1: self.data.ctrl[self.vy_id] = -vx
        # root_z_rot 액추에이터 (base_wz): 회전 (+Z가 반시계 방향)
        if self.wz_id != -1: self.data.ctrl[self.wz_id] = wz
        
    def control_wheels(self, w_lf: float, w_lb: float, w_rf: float, w_rb: float) -> None:
        """
        각 바퀴 액추에이터에 직접 회전 속도를 인가합니다.
        입력값은 로봇 전진 시 바퀴가 앞으로 굴러가는 방향을 양수(+)로 가정합니다.
        우측 바퀴들은 축 방향(axis="-1 0 0")이 반대이므로, 양수 인가 시 뒤로 굴러갑니다.
        따라서 우측 바퀴의 경우 값을 반전(-)하여 물리 엔진에 전달합니다.
        """
        if self.w_lf_id != -1: self.data.ctrl[self.w_lf_id] = w_lf
        if self.w_lb_id != -1: self.data.ctrl[self.w_lb_id] = w_lb
        if self.w_rf_id != -1: self.data.ctrl[self.w_rf_id] = -w_rf
        if self.w_rb_id != -1: self.data.ctrl[self.w_rb_id] = -w_rb

    def control_arm_joints(self, target_positions: list[float]) -> None:
        """
        리프트 및 7개 암 조인트(총 8개)의 Position Actuator 제어.
        target_positions 배열 길이는 반드시 8이어야 함.
        """
        if len(target_positions) != len(self.arm_actuator_ids):
            return
            
        for idx, act_id in enumerate(self.arm_actuator_ids):
            if act_id != -1:
                self.data.ctrl[act_id] = target_positions[idx]
