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
