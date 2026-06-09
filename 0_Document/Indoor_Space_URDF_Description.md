# RAMI 실내 공간 URDF Description 패키지 설명서

이 문서는 Fusion 360의 `fusion360-urdf-ros2` 플러그인을 사용하여 내보낸(Export) **indoor_space_urdf_description** 폴더의 구성 요소와 각 파일의 역할을 정리한 문서입니다. `RAMI_URDF_Description.md`와 유사한 표준 구조를 가지고 있습니다.

## 📁 디렉토리 구조 요약
내보내기 된 폴더는 표준적인 **ROS 2 패키지 구조**를 따르고 있습니다. 본 프로젝트(RAMI)가 MuJoCo 물리 시뮬레이터를 기반으로 하지만, 이 구조를 파악해두면 실내 공간(벽, 문, 스위치 등)의 3D 모델 및 관절(Joint) 정보를 이해하는 데 큰 도움이 됩니다.

```text
indoor_space_urdf_description/
├── config/                      # RViz 시각화 및 시뮬레이터 브릿지 설정 파일
├── launch/                      # ROS 2 실행 스크립트 (RViz, Gazebo)
├── meshes/                      # 실내 공간의 각 구성요소별 3D 모델 파일 (.stl)
├── urdf/                        # 실내 공간의 물리/구조적 정의 파일 (Xacro, Gazebo 속성)
├── package.xml                  # ROS 2 패키지 정보 및 의존성 명세
├── setup.py / setup.cfg         # ROS 2 파이썬 패키지 빌드 설정 파일
└── resource/ & test/            # 패키지 빌드 인덱싱 및 테스트 관련 폴더
```

---

## 📄 주요 폴더 및 파일 상세 설명

### 1. `urdf/` 폴더
실내 공간의 고정된 구조물(벽, 천장)과 가동되는 객체(문, 스위치)들의 위치 및 연결 상태, 조인트를 정의하는 핵심 폴더입니다.
* **`indoor_space_urdf.xacro`:** 가장 중요한 파일입니다. 방의 벽면과 문, 스위치 등의 링크와 조인트 구조적 연결 상태, 위치, 물리적 속성을 정의한 URDF 파일입니다. (매크로를 지원하는 xacro 형식)
* **`indoor_space_urdf.gazebo`:** Gazebo 시뮬레이터에서 필요한 물리적 마찰 계수(Friction), 색상(Material) 정보가 담겨 있습니다.
* **`indoor_space_urdf.trans`:** 여닫이 문이나 스위치의 구동을 위한 모터(Actuator) 및 감속기(Transmission) 설정이 포함된 파일입니다. (실내 환경의 상호작용 요소를 제어할 때 참조)
* **`materials.xacro`:** 실내 구조물의 시각적인 색상(R, G, B, Alpha) 정보를 변수로 정의한 파일입니다.

### 2. `meshes/` 폴더
Fusion 360에서 설계한 실내 공간의 외형 데이터가 `.stl` 포맷으로 분할되어 저장된 폴더입니다. `urdf.xacro` 파일에서 시각적 모델(Visual)과 충돌 모델(Collision)로 참조합니다.
* **고정 구조물:** `base_link.stl` (바닥/기반), `wall_1.stl` (벽면), `ceiling_v1_1.stl` (천장)
* **문 및 관련 부속품:** `door_1_1.stl`, `door_2_1.stl` (여닫이 문), `door_handle_1_1.stl`, `door_handle_2_1.stl` (문고리), `latch_1_1.stl`, `latch_2_1.stl` (래치)
* **스위치 및 전등:** 
  * 토글 스위치 케이스 (`toggle_switch_case_*.stl`) 및 스위치 버튼 (`toggle_switch_H20_corridor-lamp_*.stl` 등)
  * 키 스위치 케이스 및 버튼 (`key_switch_case_room_2_1.stl`, `key_switch_room-ramp_2_1.stl`)
  * 전등 모델 (`lamp_r50_kitchen_1_1.stl` 등)

### 3. `launch/` 폴더
ROS 2 환경에서 실내 공간 모델을 손쉽게 띄우기 위한 런치 파이썬 스크립트입니다. (`display.launch.py`, `gazebo.launch.py`)

### 4. `config/` 폴더
시각화 및 시뮬레이션 브릿지를 위한 사전 설정 파일들입니다. RViz를 통해 가상 실내 공간의 모습을 시각화할 수 있습니다.

### 5. 패키지 설정 파일 (`package.xml`, `setup.py`)
이 폴더가 ROS 2 패키지로 인식되고 빌드될 수 있도록 돕는 파일입니다. 패키지의 버전, 작성자 이메일, 의존성 정보가 명시되어 있습니다.

---

## 💡 RAMI 프로젝트(MuJoCo) 적용 시 전처리 계획 (Phase 4 연동용)
위의 실내 공간 URDF 폴더를 MuJoCo 환경에 로드하기 위해 로봇 모델과 동일한 방식으로 전처리 작업을 진행하게 됩니다:

1. **URDF 변환:** Python 스크립트를 사용하여 `indoor_space_urdf.xacro`를 파싱 가능한 형태로 변환 후 MuJoCo 컴파일러를 통해 `.xml` (MJCF) 포맷으로 변환합니다.
2. **시뮬레이션 최적화 편집:**
   - 관찰자 시점(탑뷰) 렌더링을 용이하게 하기 위해 천장(`ceiling`) 및 시야를 가리는 주요 벽면을 비가시화(group="3") 처리하되, 물리적 충돌은 유지합니다.
   - 문, 문고리(`door_handle`), 똑딱이 스위치(`toggle_switch`) 조인트들이 현실처럼 작동하도록(스프링 복원력, 한계 각도 설정 등) 물리 엔진 변수를 수동으로 세밀하게 조정합니다.
3. **로봇 환경과의 병합:** 변환된 실내 환경 XML을 로봇이 존재하는 `rami_world.xml`에 병합(또는 Include)하여 통합 시나리오 테스트 공간을 구성합니다.
