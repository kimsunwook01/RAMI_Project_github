import numpy as np
import mujoco

class ImuProcessor:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        
        # 실제 로봇의 위치(Ground Truth)를 읽어오기 위한 조인트 ID
        try:
            self.root_x_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
            self.root_y_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_y")
            self.root_z_rot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_z_rot")
            
            self.qpos_x_adr = model.jnt_qposadr[self.root_x_id]
            self.qpos_y_adr = model.jnt_qposadr[self.root_y_id]
            self.qpos_theta_adr = model.jnt_qposadr[self.root_z_rot_id]
            
            self.dof_x_adr = model.jnt_dofadr[self.root_x_id]
            self.dof_y_adr = model.jnt_dofadr[self.root_y_id]
            self.dof_theta_adr = model.jnt_dofadr[self.root_z_rot_id]
        except ValueError:
            self.qpos_x_adr = -1

    def process(self, data: mujoco.MjData):
        if self.qpos_x_adr == -1:
            return {"x": 0.0, "y": 0.0, "yaw_heading": 0.0, "vx": 0.0, "vy": 0.0, "omega": 0.0}
            
        # 오차와 왜곡이 없는 완벽한 Ground Truth 절대 좌표/속도 반환 (사용자 피드백 반영)
        x = data.qpos[self.qpos_x_adr]
        y = data.qpos[self.qpos_y_adr]
        theta = data.qpos[self.qpos_theta_adr]
        
        vx = data.qvel[self.dof_x_adr]
        vy = data.qvel[self.dof_y_adr]
        omega = data.qvel[self.dof_theta_adr]
        
        return {
            "x": float(x),
            "y": float(y),
            "yaw_heading": float(theta),
            "vx": float(vx),
            "vy": float(vy),
            "omega": float(omega),
            "accel": [0,0,0], # 레거시 인터페이스 호환용
            "gyro": [0, 0, float(omega)]
        }
