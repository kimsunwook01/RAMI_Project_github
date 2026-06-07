# RAMI 로봇 URDF Description 패키지 설명서

이 문서는 Fusion 360의 `fusion360-urdf-ros2` 플러그인을 사용하여 내보낸(Export) **robot_RAMI_v1_urdf_description** 폴더의 구성 요소와 각 파일의 역할을 정리한 문서입니다.

## 📁 디렉토리 구조 요약
내보내기 된 폴더는 표준적인 **ROS 2 패키지 구조**를 따르고 있습니다. 비록 본 프로젝트(RAMI)가 MuJoCo 물리 시뮬레이터를 기반으로 하지만, 이 구조를 파악해두면 로봇의 3D 모델 및 관절(Joint) 정보를 이해하고 향후 다른 플랫폼으로 확장하는 데 큰 도움이 됩니다.

```text
robot_RAMI_v1_urdf_description/
├── config/                      # RViz 시각화 및 시뮬레이터 브릿지 설정 파일
├── launch/                      # ROS 2 실행 스크립트 (RViz, Gazebo)
├── meshes/                      # 로봇의 각 부품별 3D 모델 파일 (.stl)
├── urdf/                        # 로봇의 물리/구조적 정의 파일 (Xacro, Gazebo 속성)
├── package.xml                  # ROS 2 패키지 정보 및 의존성 명세
├── setup.py / setup.cfg         # ROS 2 파이썬 패키지 빌드 설정 파일
└── resource/ & test/            # 패키지 빌드 인덱싱 및 테스트 관련 폴더
```

---

## 📄 주요 폴더 및 파일 상세 설명

### 1. `urdf/` 폴더
로봇의 뼈대, 관절, 무게, 관성 모멘트 등을 정의하는 핵심 폴더입니다.
* **`robot_RAMI_v1_urdf.xacro`:** 가장 중요한 파일입니다. 로봇의 각 링크(부품)와 조인트(관절)의 구조적 연결 상태, 위치, 물리적 속성(질량, 관성)을 정의한 URDF 파일입니다. (매크로를 지원하는 xacro 형식)
* **`robot_RAMI_v1_urdf.gazebo`:** Gazebo 시뮬레이터에서 필요한 물리적 마찰 계수(Friction), 색상(Material), 그리고 센서 플러그인(카메라, IMU, 레이저 등) 정보가 담겨 있습니다.
* **`robot_RAMI_v1_urdf.trans`:** 각 관절을 구동하는 모터(Actuator)와 감속기(Transmission) 설정이 포함된 파일입니다. 로봇 팔이나 바퀴를 구동할 때 참조됩니다.
* **`materials.xacro`:** 로봇 모델의 시각적인 색상(R, G, B, Alpha) 정보를 변수로 정의한 파일입니다.

### 2. `meshes/` 폴더
Fusion 360에서 설계한 로봇의 외형 데이터가 `.stl` 포맷으로 분할되어 저장된 폴더입니다. `urdf.xacro` 파일에서 시각적 모델(Visual)과 충돌 모델(Collision)로 참조합니다.
* **동체 및 구동계:** `base_link.stl`, `weel_left_back_1.stl`, `weel_right_front_1.stl` 등 (메카넘 휠 및 모바일 베이스)
* **기둥 및 로봇 암:** `pillar_link_1.stl`, `lift_slide_link_1.stl`, `arm_link_1~6.stl` 등 (수직 리프트 및 다관절 암)
* **센서 시스템:** `depth_camera_1.stl`, `gripper_camera_1.stl`, `ultrasonic_sensor_1~8.stl`, `imu_sensor_1.stl`

### 3. `launch/` 폴더
ROS 2 환경에서 로봇 모델을 손쉽게 띄우기 위한 런치 파이썬 스크립트입니다.
* **`display.launch.py`:** 조인트 상태 퍼블리셔(Joint State Publisher) UI와 함께 RViz 3D 뷰어를 실행하여 모델의 구조와 관절의 움직임을 테스트해 볼 수 있습니다.
* **`gazebo.launch.py`:** Gazebo 시뮬레이터 환경에 로봇을 스폰(Spawn)하여 중력 및 물리 법칙이 적용된 상태를 테스트합니다.

### 4. `config/` 폴더
시각화 및 시뮬레이션 브릿지를 위한 사전 설정 파일들입니다.
* **`display.rviz` / `gazebo.rviz`:** RViz 실행 시 카메라 시점, 로봇 모델 투명도, TF(좌표계) 트리 표시 여부 등 화면 설정이 저장된 프리셋 파일입니다.
* **`ros_gz_bridge_gazebo.yaml`:** 최신 ROS 2와 Gazebo 시뮬레이터 간의 메시지 통신(토픽)을 연결해 주는 브릿지 설정 파일입니다.

### 5. 패키지 설정 파일 (`package.xml`, `setup.py`)
이 폴더가 ROS 2 패키지로 인식되고 빌드될 수 있도록 돕는 파일입니다. 패키지의 버전, 작성자 이메일, 그리고 실행 시 필요한 라이브러리(의존성) 정보가 명시되어 있습니다.

---

## 💡 RAMI 프로젝트(MuJoCo) 적용 시 참고사항
본 프로젝트는 **MuJoCo** 시뮬레이터를 기반으로 진행됩니다. MuJoCo는 URDF 파일(`.xacro` 컴파일 후 변환 필요)을 네이티브하게 로드할 수 있으므로, 아래와 같은 워크플로우를 권장합니다.

1. **URDF 컴파일:** `robot_RAMI_v1_urdf.xacro` 파일을 순수 `.urdf`로 변환합니다.
2. **MJCF 변환:** 변환된 `.urdf` 파일을 MuJoCo 내부 기능 또는 스크립트를 통해 MuJoCo 전용 포맷인 `.xml` (MJCF) 형식으로 변환합니다.
3. **MuJoCo 태그 추가:** 자동 변환된 MJCF 파일 내에 RAMI 프로젝트 특성에 맞는 Actuator(모터/슬라이드 구동기), 텐던(Tendon), 센서(카메라 렌더링, 레이저 레인지파인더 등) 설정을 수동으로 추가하여 최종 환경을 구축합니다.
