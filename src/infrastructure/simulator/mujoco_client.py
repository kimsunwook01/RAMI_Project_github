import mujoco

class MujocoClient:
    """
    MuJoCo 환경 초기화 및 시뮬레이션 상태 관리를 담당하는 싱글톤 지향 클래스입니다.
    """
    def __init__(self, xml_path: str):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
    def step(self):
        """1 step 시뮬레이션을 진행합니다."""
        mujoco.mj_step(self.model, self.data)
