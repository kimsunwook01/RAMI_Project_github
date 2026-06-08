from typing import Protocol, List

class RobotHardwareIO(Protocol):
    """
    MuJoCo 시뮬레이터나 실제 로봇 하드웨어 컨트롤러가 반드시 구현해야 할 추상 인터페이스입니다.
    순수 제어 로직(Domain/Application)은 이 인터페이스만을 통해 로봇과 소통합니다.
    """
    
    def move_base(self, vx: float, vy: float, wz: float) -> None:
        """
        동체의 이동 속도를 직접 제어합니다.
        
        Args:
            vx (float): 전후 이동 속도 (m/s)
            vy (float): 좌우 평행 이동 속도 (m/s)
            wz (float): Z축 기준 회전 각속도 (rad/s)
        """
        pass
        
    def control_wheels(self, w_lf: float, w_lb: float, w_rf: float, w_rb: float) -> None:
        """
        4개 바퀴의 독립적인 각속도를 개별 제어합니다. (디버깅 및 메카넘 역기구학용)
        
        Args:
            w_lf (float): 좌측 전륜 각속도 (rad/s)
            w_lb (float): 좌측 후륜 각속도 (rad/s)
            w_rf (float): 우측 전륜 각속도 (rad/s)
            w_rb (float): 우측 후륜 각속도 (rad/s)
        """
        pass
        
    def control_arm_joints(self, target_positions: List[float]) -> None:
        """
        리프트를 포함한 로봇암 7개 조인트의 목표 위치/각도를 제어합니다.
        
        Args:
            target_positions (List[float]): [리프트(m), 회전관절(rad), 암1(rad), 암2(rad), 암3(rad), 암4(rad), 암5(rad), 암6(rad)]
                단, 리프트 포함 총 8개의 액추에이터가 존재함.
        """
        pass
        
    def read_arm_joints(self) -> List[float]:
        """
        리프트를 포함한 로봇암 7개 조인트의 실제 위치/각도를 읽어옵니다.
        
        Returns:
            List[float]: [리프트(m), 회전관절(rad), 암1(rad), 암2(rad), 암3(rad), 암4(rad), 암5(rad), 암6(rad)]
        """
        pass
        
    def read_wheel_joints(self) -> List[float]:
        """
        4개 바퀴 조인트의 실제 회전 각도(Position)를 읽어옵니다. (Odometry 및 폐루프 제어용)
        
        Returns:
            List[float]: [w_lf(rad), w_lb(rad), w_rf(rad), w_rb(rad)]
        """
        pass
        
    def read_base_pose(self) -> tuple[float, float, float]:
        """
        로봇 동체의 실제 위치 및 자세를 읽어옵니다. (관성에 의한 밀림 보상용)
        
        Returns:
            tuple[float, float, float]: (x(m), y(m), theta(rad))
        """
        pass
        
    def get_end_effector_pose(self) -> tuple[List[float], List[float]]:
        """
        로봇암 말단(End-effector, 예: gripper_camera_1)의 현재 전역(또는 베이스 기준) 좌표 및 방향을 반환합니다.
        
        Returns:
            tuple[List[float], List[float]]: (position[x,y,z], orientation_matrix[3x3])
        """
        pass
        
    def get_jacobian(self) -> tuple[List[List[float]], List[List[float]]]:
        """
        현재 관절 상태에서의 매니퓰레이터 자코비안(Jacobian) 행렬을 반환합니다.
        
        Returns:
            tuple[jacp, jacr]: 
                jacp: 3 x N 위치 자코비안 (N은 조인트 수)
                jacr: 3 x N 회전 자코비안
        """
        pass
        
    def get_arm_dof_indices(self) -> List[int]:
        """
        리프트를 포함한 8개 매니퓰레이터 조인트의 속도(qvel/dof) 인덱스를 반환합니다.
        자코비안 행렬에서 해당 열(Column)을 추출하기 위해 사용됩니다.
        
        Returns:
            List[int]: [리프트, 회전, 암1, 암2, 암3, 암4, 암5, 암6]
        """
        pass
        
    def compute_virtual_fk_and_jacobian(self, virtual_joints: List[float]) -> tuple[List[float], List[List[float]], List[List[float]], List[List[float]]]:
        """
        주어진 가상의 조인트 각도로 물리엔진 상태를 업데이트한 후(시뮬레이션 시간 진행 없음),
        순운동학(말단 좌표 및 방향)과 자코비안 행렬을 계산해 반환합니다.
        계산 후 내부적으로 기존 상태(원래 관절 각도)로 원상 복구해야 합니다.
        
        Returns:
            tuple: (pos, rot_mat, jacp, jacr)
        """
        pass
