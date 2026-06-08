import os
import sys
import math

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter
from src.application.services.manipulator_service import ManipulatorService
from src.domain.controllers.kinematics import KinematicsSolver

def test_global_ik():
    print("=== [Phase 2-3] Step 14: Iterative Global IK Solver 수치 검증 ===")
    
    xml_path = "config/rami_description/rami_world.xml"
    client = MujocoClient(xml_path)
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    # 서비스 초기화
    service = ManipulatorService(hardware=adapter, dt=client.model.opt.timestep)
    
    # 테스트 1: 특이점 회피를 위한 초기 자세 세팅 (Bent Pose)
    bent_pose = [0.0, 0.0, 0.0, -0.785398, 1.570796, -0.785398, 0.0, 0.0]
    service.cmd_joints = bent_pose.copy()
    
    # 목표 좌표 설정 (로봇 정면 약간 위)
    target_pos = [0.0, 0.4, 0.5]
    target_rpy = [0.0, 0.0, 0.0]
    target_rot_mat = KinematicsSolver.euler_to_rotation_matrix(*target_rpy)
    
    print(f"\n[목표 1] Pos: {target_pos}, RPY: {target_rpy}")
    
    # 글로벌 IK 연산 수행
    success = service.solve_global_ik(target_pos, target_rot_mat, max_iter=500, tol=0.001)
    
    if success:
        print("[SUCCESS] 수렴 성공!")
        # 검증을 위해 얻어낸 조인트를 물리엔진에 넣고 실제 좌표를 찍어봄
        pos, rot, _, _ = adapter.compute_virtual_fk_and_jacobian(service.global_target_joints)
        
        dx = pos[0] - target_pos[0]
        dy = pos[1] - target_pos[1]
        dz = pos[2] - target_pos[2]
        error_dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        print(f"  - 연산된 조인트 각도: {[round(q, 3) for q in service.global_target_joints]}")
        print(f"  - 실제 도달 위치: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}")
        print(f"  - 최종 위치 오차: {error_dist * 1000:.2f} mm")
        assert error_dist < 0.001, "오차가 허용 범위를 초과했습니다."
    else:
        print("[FAIL] 수렴 실패 (발산)")
        
    # 테스트 2: 완전히 다른 공간 좌표 (오른쪽 아래)
    target_pos2 = [-0.3, 0.3, 0.3]
    target_rot_mat2 = KinematicsSolver.euler_to_rotation_matrix(*target_rpy)
    
    print(f"\n[목표 2] Pos: {target_pos2}, RPY: {target_rpy}")
    service.cmd_joints = bent_pose.copy() # 다시 초기 자세에서 출발
    success2 = service.solve_global_ik(target_pos2, target_rot_mat2, max_iter=500, tol=0.001)
    
    if success2:
        print("[SUCCESS] 수렴 성공!")
        pos, rot, _, _ = adapter.compute_virtual_fk_and_jacobian(service.global_target_joints)
        dx = pos[0] - target_pos2[0]
        dy = pos[1] - target_pos2[1]
        dz = pos[2] - target_pos2[2]
        error_dist = math.sqrt(dx**2 + dy**2 + dz**2)
        print(f"  - 연산된 조인트 각도: {[round(q, 3) for q in service.global_target_joints]}")
        print(f"  - 실제 도달 위치: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}")
        print(f"  - 최종 위치 오차: {error_dist * 1000:.2f} mm")
        assert error_dist < 0.001, "오차가 허용 범위를 초과했습니다."
    else:
        print("[FAIL] 수렴 실패 (발산)")

if __name__ == "__main__":
    test_global_ik()
