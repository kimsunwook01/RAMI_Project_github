import os
import sys
import mujoco
import mujoco.viewer

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

def main():
    print("=== [Step 19] 실내 공간 통합 환경 시각화 테스트 ===")
    
    xml_path = os.path.join(os.path.dirname(__file__), "..", "config", "rami_description", "rami_phase4_world.xml")
    xml_path = os.path.abspath(xml_path)
    
    print(f"[시스템] {xml_path} 모델을 로드합니다...")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    print("[시스템] 뷰어를 실행합니다.")
    print(" - 마우스 우클릭 드래그: 화면 회전")
    print(" - 마우스 좌클릭 더블클릭 후 드래그: 물체(문, 스위치) 조작")
    print(" - 키보드 숫자 '3' 키: Group 3(천장 등) 렌더링 토글 (기본값: 숨김)")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 천장(group 3)을 숨기기 위해 옵션에서 group 3을 끕니다.
        viewer.opt.geomgroup[3] = 0
        
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()

if __name__ == "__main__":
    main()
