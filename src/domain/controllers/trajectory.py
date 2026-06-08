from typing import List

class TrajectoryGenerator:
    """
    급격한 모터 구동으로 인한 동체의 관성 흔들림(밀림)을 방지하기 위해
    목표 조인트 각도로 이동할 때 최대 가속도와 속도를 제한하여 부드러운 궤적을 생성합니다.
    """
    
    def __init__(self, num_joints: int = 8, max_vel: float = 1.0, max_acc: float = 2.0, dt: float = 0.01):
        """
        Args:
            num_joints: 제어할 조인트 수
            max_vel: 조인트의 최대 속도 한계 (rad/s 또는 m/s)
            max_acc: 조인트의 최대 가속도 한계 (rad/s^2 또는 m/s^2)
            dt: 제어 주기 (초)
        """
        self.num_joints = num_joints
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.dt = dt
        
        # 이전 스텝의 속도 상태를 저장
        self.current_vel = [0.0] * num_joints
        
    def reset(self):
        """ 초기화 (정지 상태) """
        self.current_vel = [0.0] * self.num_joints
        
    def step(self, current_cmd_joints: List[float], ideal_next_joints: List[float]) -> List[float]:
        """
        현재 명령 조인트에서 이상적인 다음 조인트로 이동하기 위한 가감속 궤적을 계산합니다.
        
        Args:
            current_cmd_joints: 내부적으로 관리되는 현재 제어 명령 각도
            ideal_next_joints: IK가 계산한 이상적인 다음 목표 각도
            
        Returns:
            List[float]: 가속도/속도 제한이 적용된 실제 하달될 다음 스텝 명령
        """
        next_cmd = []
        for i in range(self.num_joints):
            # 1. 이상적 목표에 도달하기 위한 요구 속도 계산 (P Gain 적용)
            v_des = (ideal_next_joints[i] - current_cmd_joints[i]) / self.dt
            
            # 2. 가속도 제한
            a_req = (v_des - self.current_vel[i]) / self.dt
            a_req = max(-self.max_acc, min(self.max_acc, a_req))
            
            # 3. 속도 업데이트 및 제한
            self.current_vel[i] += a_req * self.dt
            self.current_vel[i] = max(-self.max_vel, min(self.max_vel, self.current_vel[i]))
            
            # 4. 다음 위치 적분 (현재 명령값 기반)
            next_pos = current_cmd_joints[i] + self.current_vel[i] * self.dt
            next_cmd.append(next_pos)
            
        return next_cmd
