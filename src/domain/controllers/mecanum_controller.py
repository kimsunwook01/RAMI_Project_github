class MecanumController:
    """
    메카넘 휠 구동 역기구학(Inverse Kinematics) 연산을 수행하는 순수 도메인 모듈입니다.
    로봇의 이동 속도 명령을 받아 각 바퀴의 각속도 명령으로 변환합니다.
    """
    def __init__(self, wheel_radius: float = 0.05, wheel_base: float = 0.2, track_width: float = 0.3):
        """
        Args:
            wheel_radius (float): 바퀴의 반지름 (단위: m)
            wheel_base (float): 앞바퀴와 뒷바퀴 사이의 거리 (단위: m) -> 2 * Lx
            track_width (float): 좌측 바퀴와 우측 바퀴 사이의 거리 (단위: m) -> 2 * Ly
        """
        self.R = wheel_radius
        self.Lx = wheel_base / 2.0
        self.Ly = track_width / 2.0
        
    def compute_wheel_velocities(self, vx: float, vy: float, wz: float) -> tuple[float, float, float, float]:
        """
        로봇의 목표 속도를 기반으로 4개 바퀴의 각속도를 계산합니다.
        (Standard Mecanum 'X' roller configuration)
        
        Args:
            vx (float): 전진 속도 (m/s)
            vy (float): 좌측 이동 속도 (m/s)
            wz (float): 반시계 방향 회전 각속도 (rad/s)
            
        Returns:
            tuple: (w_lf, w_lb, w_rf, w_rb) 단위: rad/s
        """
        # (Lx + Ly) 팩터
        K = self.Lx + self.Ly
        
        # 역기구학 방정식
        w_lf = (vx - vy - K * wz) / self.R
        w_lb = (vx + vy - K * wz) / self.R
        w_rf = (vx + vy + K * wz) / self.R
        w_rb = (vx - vy + K * wz) / self.R
        
        return w_lf, w_lb, w_rf, w_rb
