import mujoco

class VisionProcessor:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
    def get_camera_info(self, camera_name: str) -> dict:
        """
        특정 카메라의 현재 전역(Global) 위치 및 시선 벡터(Forward Vector)를 반환합니다.
        Cheat Adapter 원칙에 따라, 실제 이미지 대신 기하학적 메타데이터를 활용합니다.
        """
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id == -1:
            return {"error": f"Camera '{camera_name}' not found"}
            
        pos = [float(x) for x in self.data.cam_xpos[cam_id]]
        
        # cam_xmat은 1D 배열(9개 요소)이므로 3x3 회전 행렬로 간주
        mat = self.data.cam_xmat[cam_id]
        
        # MuJoCo 카메라의 정면(Forward) 방향은 로컬 Z축의 음의 방향(-Z)입니다.
        # 회전 행렬의 3번째 열(Z축 벡터)에 -1을 곱합니다.
        fwd = [-float(mat[2]), -float(mat[5]), -float(mat[8])]
        
        return {
            "pos": pos,
            "forward_vector": fwd
        }
