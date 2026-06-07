import os
import sys
import time

# 루트 디렉토리를 sys.path에 추가하여 src 모듈을 임포트할 수 있도록 함
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
import mujoco.viewer
from src.infrastructure.simulator.mujoco_client import MujocoClient
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

def main():
    xml_path = "config/rami_description/rami_world.xml"
    
    # 1. 시뮬레이터 로드
    client = MujocoClient(xml_path)
    
    # 2. 어댑터(하드웨어 인터페이스) 생성
    adapter = RamiMujocoAdapter(client.model, client.data)
    
    print("뷰어를 실행합니다...")
    print("주행 시나리오: 전진(2초) -> 좌측이동(2초) -> 후진대각이동(2초) -> 회전(2초) -> 정지")
    
    # 3. 비동기(Passive) 뷰어 론치
    with mujoco.viewer.launch_passive(client.model, client.data) as viewer:
        start_time = time.time()
        
        while viewer.is_running():
            step_start = time.time()
            elapsed = time.time() - start_time
            
            # 주행 시나리오에 따른 속도 제어 (우선 전진만 테스트)
            if elapsed < 5.0:
                # 0~5초: 전진 (X축 +)
                adapter.move_base(0.3, 0.0, 0.0)
            else:
                # 5초 이후: 완전 정지
                adapter.move_base(0.0, 0.0, 0.0)
                
            # 물리 엔진 1스텝 연산
            client.step()
            
            # 뷰어 렌더링 동기화
            viewer.sync()
            
            # 시뮬레이션 타임스텝(보통 0.002초)에 맞춰 실시간 주행 모사
            time_until_next_step = client.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
