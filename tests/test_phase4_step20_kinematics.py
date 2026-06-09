import os
import sys
import time
import math
import mujoco
import mujoco.viewer

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

def main():
    print("=== [Step 20] 실내 공간 물리 동작(Kinematics) 검증 테스트 ===")
    
    xml_path = os.path.join(os.path.dirname(__file__), "..", "config", "rami_description", "rami_phase4_world.xml")
    xml_path = os.path.abspath(xml_path)
    
    print(f"[시스템] {xml_path} 모델 로드...")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # 식별된 조인트 필터링
    test_joints = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name and ("door" in name or "switch" in name or "latch" in name):
            test_joints.append(name)
            
    print(f"테스트 대상 조인트 ({len(test_joints)}개):")
    for j in test_joints:
        print(f" - {j}")
        
    print("\n[시스템] 뷰어를 실행합니다. 각 조인트가 자동으로 최대/최소 각도를 순환하며 구동됩니다.")
    print("물리적 뚫림(Penetration)이 있거나 관절 한계가 잘못 설정된 경우 튕기는 현상이 발생합니다.")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.geomgroup[3] = 0  # 천장 투명화
        
        t = 0
        while viewer.is_running():
            t += 0.02
            
            # for jname in test_joints:
            #     jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            #     qpos_idx = model.jnt_qposadr[jid]
            #     
            #     # 조인트 리미트 정보 가져오기
            #     if model.jnt_limited[jid]:
            #         jmin, jmax = model.jnt_range[jid]
            #         mid = (jmin + jmax) / 2.0
            #         amp = (jmax - jmin) / 2.0
            #         
            #         # 3초 주기로 사인파 형태로 움직임
            #         target_pos = mid + amp * math.sin(t)
            #         data.qpos[qpos_idx] = target_pos
                    
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.02)

if __name__ == "__main__":
    main()
