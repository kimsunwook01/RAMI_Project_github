import math
import numpy as np
from typing import List, Tuple

class KinematicsSolver:
    """
    매니퓰레이터의 자코비안 행렬과 Damped Least Squares (DLS) 기법을 이용하여 
    역기구학(IK)을 계산하는 순수 도메인 모듈입니다.
    """
    
    def __init__(self, damping: float = 0.1, step_size: float = 0.5):
        self.damping = damping
        self.step_size = step_size
        
        # 각 조인트(리프트 포함 8개)의 구동 한계. 
        # (최소, 최대) 라디안 또는 미터
        self.limits = [
            (-0.62, 0.62),      # Lift
            (-3.14159, 3.14159),# Rot
            (-1.5708, 1.5708),  # Arm 1
            (-1.5708, 1.5708),  # Arm 2
            (-1.5708, 1.5708),  # Arm 3
            (-1.5708, 1.5708),  # Arm 4
            (-3.14159, 3.14159),# Arm 5
            (0.0, 1.5708),      # Arm 6
        ]

    def solve_ik_dls(
        self,
        target_pos: List[float],
        target_rot_mat: List[List[float]],
        current_pos: List[float],
        current_rot_mat: List[List[float]],
        current_joints: List[float],
        jacp: List[List[float]],
        jacr: List[List[float]],
        dof_indices: List[int]
    ) -> List[float]:
        """
        주어진 목표 위치/자세와 현재 자코비안을 바탕으로 DLS 역기구학을 1회 수행합니다.
        
        Returns:
            List[float]: 1 step 이동해야 할 다음 조인트 각도 배열 (길이 8)
        """
        # 1. 자코비안 슬라이싱 (우리가 제어하는 8개 DOF만 추출)
        # jacp: (3, NV), jacr: (3, NV)
        jp = np.array(jacp)
        jr = np.array(jacr)
        
        J = np.zeros((6, len(dof_indices)))
        for i, idx in enumerate(dof_indices):
            if idx != -1:
                J[0:3, i] = jp[:, idx]
                J[3:6, i] = jr[:, idx]
                
        # 2. 위치 및 자세(Orientation) 오차 계산
        R_curr = np.array(current_rot_mat)
        R_targ = np.array(target_rot_mat)
        
        # 회전 벡터(axis-angle) 추출: e_ori = 1/2 * sum( col_curr x col_targ )
        e_ori = 0.5 * (np.cross(R_curr[:, 0], R_targ[:, 0]) +
                       np.cross(R_curr[:, 1], R_targ[:, 1]) +
                       np.cross(R_curr[:, 2], R_targ[:, 2]))
                       
        e = np.concatenate((np.array(target_pos) - np.array(current_pos), e_ori))
        
        # 3. Damped Least Squares 계산
        # Delta q = (J^T * J + lambda^2 * I)^-1 * J^T * e
        J_T = J.T
        I = np.eye(len(dof_indices))
        
        # 행렬 연산
        matrix_to_inv = J_T @ J + (self.damping ** 2) * I
        delta_q = np.linalg.inv(matrix_to_inv) @ J_T @ e
        
        # 5. 현재 각도에 업데이트 반영
        next_joints = np.array(current_joints) + self.step_size * delta_q
        
        # 6. 조인트 한계(Limit) 클램핑
        clamped_joints = []
        for i in range(len(next_joints)):
            val = next_joints[i]
            min_val, max_val = self.limits[i]
            val = max(min_val, min(max_val, val))
            clamped_joints.append(float(val))
            
        return clamped_joints

    def compute_velocity(self, v_cartesian: List[float], jacp: List[List[float]], jacr: List[List[float]], dof_indices: List[int]) -> List[float]:
        """
        직교좌표계 작업공간 속도(Cartesian Velocity)를 조인트 속도로 변환합니다.
        
        Args:
            v_cartesian: [vx, vy, vz, wx, wy, wz] 형태의 6자유도 속도 벡터
            jacp, jacr: 시뮬레이터에서 획득한 위치/회전 자코비안
            dof_indices: 추출할 조인트 인덱스
            
        Returns:
            List[float]: 조인트 속도 벡터 (길이 8)
        """
        jp = np.array(jacp)
        jr = np.array(jacr)
        
        J = np.zeros((6, len(dof_indices)))
        for i, idx in enumerate(dof_indices):
            if idx != -1:
                J[0:3, i] = jp[:, idx]
                J[3:6, i] = jr[:, idx]
                
        # DLS Pseudo-inverse: J_pseudo = (J^T J + lambda^2 I)^-1 J^T
        J_T = J.T
        I = np.eye(len(dof_indices))
        matrix_to_inv = J_T @ J + (self.damping ** 2) * I
        J_pseudo = np.linalg.inv(matrix_to_inv) @ J_T
        
        delta_q = J_pseudo @ np.array(v_cartesian)
        return delta_q.tolist()

    @staticmethod
    def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
        """ ZYX Euler angles to 3x3 Rotation Matrix """
        cx, sx = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw), math.sin(yaw)
        
        R_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        R_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        R_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        
        R = R_z @ R_y @ R_x
        return R.tolist()

    @staticmethod
    def compute_error(target_pos: List[float], target_rot_mat: List[List[float]], current_pos: List[float], current_rot_mat: List[List[float]]) -> List[float]:
        """
        위치 오차와 방향 오차를 결합하여 6자유도 에러 벡터를 반환합니다.
        """
        e_pos = np.array(target_pos) - np.array(current_pos)
        
        R_curr = np.array(current_rot_mat)
        R_targ = np.array(target_rot_mat)
        e_ori = 0.5 * (np.cross(R_curr[:, 0], R_targ[:, 0]) +
                       np.cross(R_curr[:, 1], R_targ[:, 1]) +
                       np.cross(R_curr[:, 2], R_targ[:, 2]))
                       
        return np.concatenate((e_pos, e_ori)).tolist()

    def limit_joints(self, joints: List[float]) -> List[float]:
        """ 조인트 제한 범위를 적용합니다. """
        clamped = []
        for i, val in enumerate(joints):
            min_val, max_val = self.limits[i]
            clamped.append(max(min_val, min(max_val, val)))
        return clamped
