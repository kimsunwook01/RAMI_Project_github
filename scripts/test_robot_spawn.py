import mujoco
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

def test_spawn():
    # 1. 병합된 환경 로드
    model = mujoco.MjModel.from_xml_path('d:/Programming/RAMI_Project/config/rami_description/rami_indoor_world.xml')
    data = mujoco.MjData(model)
    adapter = RamiMujocoAdapter(model, data)
    
    # 2. 로봇을 실내 (X=11.0, Y=16.0) 좌표로 이동 (root_x, root_y 조인트 제어)
    root_x_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
    root_y_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_y")
    if root_x_id != -1 and root_y_id != -1:
        data.qpos[model.jnt_qposadr[root_x_id]] = 0.0
        data.qpos[model.jnt_qposadr[root_y_id]] = 0.0
        
    # 3. 로봇 팔 안전 자세 설정
    safe_arm_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    adapter.control_arm_joints(safe_arm_pose)
    
    # 4. 물리 시뮬레이션 초기화 진행
    mujoco.mj_forward(model, data)
    
    # Check Contacts
    print("\n=== Contacts After Forward ===")
    for i in range(data.ncon):
        con = data.contact[i]
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom1)
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom2)
        print(f"Contact {i}: {g1} <-> {g2}")
    
    print("\n=== Initial Pose ===")
    arm_joints = ["lift_slide_joint", "rotation_joint", "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6"]
    actual = adapter.read_arm_joints()
    for name, act in zip(arm_joints, actual):
        print(f"{name}: {act}")
        
    # 5. 물리 시뮬레이션 1000스텝 진행 (약 2초)
    print("\n=== Stepping 1000 times (approx 2.0s) ===")
    for _ in range(1000):
        # 매 스텝마다 위치 제어 신호 유지
        adapter.control_arm_joints(safe_arm_pose)
        mujoco.mj_step(model, data)
        
    print("\n=== After 1000 Steps ===")
    actual = adapter.read_arm_joints()
    for name, act in zip(arm_joints, actual):
        print(f"{name}: {act}")
        
    # 폭발 여부 검사
    for act in actual:
        if abs(act) > 3.14:  # 기본적으로 0.0을 유지해야 하므로, 3.14를 넘으면 비정상으로 간주
            print("\n[WARNING] 로봇 관절 값이 비정상적으로 큽니다. 물리 엔진 폭발 의심!")
            return
            
    print("\n[SUCCESS] 로봇 관절이 안정적으로 유지되었습니다.")

if __name__ == "__main__":
    test_spawn()
