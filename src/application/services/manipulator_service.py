import time
import math
from typing import Callable, List, Tuple

from src.application.interfaces.robot_hardware_io import RobotHardwareIO
from src.domain.controllers.kinematics import KinematicsSolver
from src.domain.controllers.trajectory import TrajectoryGenerator

class ManipulatorService:
    """
    사용자의 고수준 명령(목표 3D 좌표 및 자세)을 받아,
    IK 솔버와 궤적 생성기를 거쳐 하드웨어(RobotHardwareIO)로 전달하는 애플리케이션 서비스
    """
    def __init__(self, hardware: RobotHardwareIO, dt: float = 0.01):
        self.hardware = hardware
        self.dt = dt
        self.ik_solver = KinematicsSolver(damping=0.1, step_size=0.1)
        self.trajectory = TrajectoryGenerator(num_joints=8, max_vel=1.5, max_acc=3.0, dt=dt)
        self.cmd_joints = None
        
    def reset(self):
        self.trajectory.reset()
        self.cmd_joints = None
        
    def step_ik_towards_pose(self, target_pos: List[float], target_rot_mat: List[List[float]]) -> float:
        """
        논블로킹(Non-blocking) 방식으로 IK를 1스텝 연산하고 하드웨어에 제어 명령을 내립니다.
        메인 시뮬레이션 루프 안에서 매 프레임 호출하기 적합합니다.
        
        Returns:
            float: 현재 엔드이펙터와 목표 위치 간의 거리 오차(Error norm)
        """
        # 1. 하드웨어 피드백 획득 (위치/자세/자코비안은 실제 센서값 사용)
        current_actual = self.hardware.read_arm_joints()
        if self.cmd_joints is None:
            self.cmd_joints = current_actual.copy()
            
        curr_pos, curr_rot_mat = self.hardware.get_end_effector_pose()
        jacp, jacr = self.hardware.get_jacobian()
        dof_indices = self.hardware.get_arm_dof_indices()
        
        # 2. DLS IK 솔버로 목표 관절 각도 도출 
        # (실제 센서값이 아닌 현재 명령값 cmd_joints를 기준으로 해야 중력 처짐으로 인한 떨림을 방지)
        ik_target_joints = self.ik_solver.solve_ik_dls(
            target_pos=target_pos,
            target_rot_mat=target_rot_mat,
            current_pos=curr_pos,
            current_rot_mat=curr_rot_mat,
            current_joints=self.cmd_joints,
            jacp=jacp,
            jacr=jacr,
            dof_indices=dof_indices
        )
        
        # 3. 궤적 생성기(Trajectory Generator)로 가속도 제한
        smooth_next_joints = self.trajectory.step(self.cmd_joints, ik_target_joints)
        self.cmd_joints = smooth_next_joints
        
        # 4. 하드웨어 명령 하달
        self.hardware.control_arm_joints(self.cmd_joints)
        
        # 오차(거리) 계산
        dx = target_pos[0] - curr_pos[0]
        dy = target_pos[1] - curr_pos[1]
        dz = target_pos[2] - curr_pos[2]
        error_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return error_dist

    def step_velocity_control(self, v_cartesian: List[float]):
        """
        직교 공간 속도 제어 (Cartesian Velocity Control)
        텔레오퍼레이션 검증용으로 주어진 속도 벡터에 맞게 조인트를 이동시킵니다.
        """
        current_actual = self.hardware.read_arm_joints()
        if self.cmd_joints is None:
            self.cmd_joints = current_actual.copy()
            
        jacp, jacr = self.hardware.get_jacobian()
        dof_indices = self.hardware.get_arm_dof_indices()
        
        # 속도를 관절 델타로 변환
        delta_q = self.ik_solver.compute_velocity(v_cartesian, jacp, jacr, dof_indices)
        
        # dt만큼 적분하여 이상적인 다음 명령 계산
        ideal_next_joints = []
        for i in range(len(self.cmd_joints)):
            ideal_next_joints.append(self.cmd_joints[i] + delta_q[i] * self.dt)
            
        # 궤적 생성기 통과 (가속도/속도 제한)
        smooth_next_joints = self.trajectory.step(self.cmd_joints, ideal_next_joints)
        self.cmd_joints = smooth_next_joints
        
        self.hardware.control_arm_joints(self.cmd_joints)

    def solve_global_ik(self, target_pos: List[float], target_rot_mat: List[List[float]], max_iter: int = 100, tol: float = 0.001) -> bool:
        """
        내부 루프를 돌며 글로벌 타겟에 대한 이상적인 목표 조인트 각도를 완벽히 연산합니다.
        시뮬레이션 시간(물리엔진)을 흐르게 하지 않고, 가상으로 FK/자코비안을 업데이트하며 계산합니다.
        
        Returns:
            bool: 연산 성공(수렴) 여부
        """
        if self.cmd_joints is None:
            self.cmd_joints = self.hardware.read_arm_joints()
            
        q_virtual = self.cmd_joints.copy()
        
        # 내부 루프 파라미터 (KinematicsSolver의 step_size 대신 루프용 별도 크기 사용)
        loop_step_size = 0.5
        dof_indices = self.hardware.get_arm_dof_indices()
        
        success = False
        for _ in range(max_iter):
            # 1. 가상 조인트에 대한 FK 및 자코비안 획득
            curr_pos, curr_rot_mat, jacp, jacr = self.hardware.compute_virtual_fk_and_jacobian(q_virtual)
            
            # 2. 오차 계산
            e = self.ik_solver.compute_error(target_pos, target_rot_mat, curr_pos, curr_rot_mat)
            
            # 3. 수렴 확인 (거리 오차 및 방향 오차가 충분히 작은지)
            e_norm = math.sqrt(sum(x*x for x in e[:3])) # 위치 오차만 우선 체크
            if e_norm < tol:
                success = True
                break
                
            # 4. 속도(delta_q) 도출 (Pseudo-inverse)
            delta_q = self.ik_solver.compute_velocity(e, jacp, jacr, dof_indices)
            
            # 5. 업데이트 및 리밋 클램핑
            for i in range(len(q_virtual)):
                q_virtual[i] += delta_q[i] * loop_step_size
            q_virtual = self.ik_solver.limit_joints(q_virtual)
            
        if success:
            # 최종 연산된 이상적인 목표를 향해 궤적 생성기가 부드럽게 쫓아가도록 타겟 업데이트
            # 여기서는 직접 하드웨어로 보내지 않고, 외부의 step_towards_target 같은 곳에서
            # trajectory_generator 를 이용해 q_virtual 로 서서히 다가가도록 설계할 수 있음.
            # 하지만 단순화를 위해 현재 cmd_joints의 타겟 지점으로 지정
            self.global_target_joints = q_virtual
            
        return success

    def step_cartesian_target(self, target_pos: List[float], target_rot_mat: List[List[float]], max_cartesian_vel: float = 0.2):
        """
        비동기 시뮬레이션 루프 내에서 매 틱(tick)마다 호출되어,
        말단이 목표 직교 좌표계 지점까지 완벽한 '직선'을 그리며 이동하게 하는 폐루프 제어.
        Cartesian Waypoint Interpolation을 실시간으로 수행합니다.
        """
        if self.cmd_joints is None:
            self.cmd_joints = self.hardware.read_arm_joints()
            
        # 1. 현재 cmd_joints에 대한 FK 연산
        curr_pos, curr_rot_mat, jacp, jacr = self.hardware.compute_virtual_fk_and_jacobian(self.cmd_joints)
        
        # 2. 직교 공간 오차 계산
        e = self.ik_solver.compute_error(target_pos, target_rot_mat, curr_pos, curr_rot_mat)
        
        # 3. 최대 Cartesian 속도 클램핑 (직선 궤적 유지의 핵심)
        e_pos_norm = math.sqrt(sum(x*x for x in e[:3]))
        max_step = max_cartesian_vel * self.dt
        
        # 오차가 크면 최대 속도 벡터로 제한하여 한 걸음(waypoint) 전진
        if e_pos_norm > max_step:
            scale = max_step / e_pos_norm
            e_clamped = [x * scale for x in e]
        else:
            e_clamped = e # 도달했을 때는 남은 오차만큼만 이동
            
        # 4. 직교 속도를 조인트 속도로 변환
        dof_indices = self.hardware.get_arm_dof_indices()
        delta_q = self.ik_solver.compute_velocity(e_clamped, jacp, jacr, dof_indices)
        
        # 5. 다음 명령 타겟 (IK 1-step 적용)
        ideal_next_joints = []
        for i in range(len(self.cmd_joints)):
            ideal_next_joints.append(self.cmd_joints[i] + delta_q[i])
            
        # 6. 관절 궤적 스무딩 (부드러운 가감속) 및 적용
        smooth_next_joints = self.trajectory.step(self.cmd_joints, ideal_next_joints)
        self.cmd_joints = smooth_next_joints
        
        self.hardware.control_arm_joints(self.cmd_joints)

    def move_to_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, 
                     tick_callback: Callable = None, tol: float = 0.05, max_timeout: float = 5.0):
        """
        블로킹(Blocking) 방식으로 오차가 허용 범위 내로 들어올 때까지 루프를 돕니다.
        
        Args:
            tick_callback: 1스텝 연산 후 시뮬레이터 진행 등을 위해 호출할 콜백 함수 (예: client.step)
            tol: 허용 위치 오차 (미터)
            max_timeout: 최대 대기 시간 (초)
        """
        target_pos = [x, y, z]
        target_rot_mat = KinematicsSolver.euler_to_rotation_matrix(roll, pitch, yaw)
        
        start_time = time.time()
        
        while True:
            # 타임아웃 체크
            if time.time() - start_time > max_timeout:
                print("[ManipulatorService] 타임아웃: 목표 좌표 도달 실패")
                break
                
            error = self.step_ik_towards_pose(target_pos, target_rot_mat)
            
            if tick_callback:
                tick_callback()
                
            if error < tol:
                print(f"[ManipulatorService] 목표 도달 성공 (오차: {error:.4f}m)")
                break
