# RAMI 센서 구현 아키텍처 설계 문서 (Phase 3)

## 1. 개요
본 문서는 RAMI(Right-Arm Mobile Intelligence) 로봇의 **Phase 3: 비전 시스템 및 센서 연동**에 대한 세부 아키텍처와 구현 방침을 기술합니다. 초음파/IMU 센서는 무주코 시스템 메모리를 직접 파싱하여 고속으로 처리하며, 비전 시스템(카메라)은 향후 실제 로봇 이식성(Sim-to-Real)을 극대화하기 위해 오프스크린 렌더링(Offscreen Rendering)과 **YOLO v8** 기반의 정석적인 영상 처리(Computer Vision) 파이프라인을 도입합니다.

## 2. 센서 하드웨어 및 시각화 명세

### 2.1. 비전 센서 (Vision Sensors)
* **그리퍼 카메라 (`gripper_camera`)**
  * **위치:** `arm_link_5` 모듈의 `gripper_camera_1` 메쉬 위치.
  * **목적:** 
    1. 스위치 조작 시 QR 코드 인식 및 미세 조작용 근접 시야 제공.
    2. 일반 주행 시 주행 방향의 지면을 향하여 바닥에 떨어져 있는 장애물 감지.
  * **스펙:** 화각(FOV) 60도, 최대 인식 거리 1.0m.
  * **시각화:** 반투명한 노란색 사각뿔(Rectangular Pyramid)로 시야 범위 표시.
* **헤드 카메라 (`head_camera`)**
  * **위치:** 리프트 기둥 상단의 `depth_camera_1` 메쉬 위치.
  * **목적:** 전방 및 오버행 장애물 탐지용 뎁스(Depth) 및 이미지 정보 제공.
  * **스펙:** 화각(FOV) 80도, 최대 인식 거리 10.0m.
  * **시각화:** 반투명한 노란색 사각뿔로 시야 범위 표시.

### 2.2. 초음파 센서 (Ultrasonic Rangefinders)
* **초음파 센서 어레이 (`ultrasonic_sensor_1` ~ `8`)**
  * **위치:** `base_link` 하단 `ultrasonic_sensor_X_1` 메쉬 위치 (총 8개).
  * **방향 설정:** 
    1. 좌전방 대각 (Left-front diagonal)
    2. 전방 (Front)
    3. 우전방 대각 (Right-front diagonal)
    4. 우측면 (Right)
    5. 우후방 대각 (Right-rear diagonal)
    6. 후방 (Rear)
    7. 좌후방 대각 (Left-rear diagonal)
    8. 좌측면 (Left)
  * **목적:** 카메라 사각지대, 유리, 거울 등 비전 인식이 어려운 장애물 감지 및 충돌 회피.
  * **스펙:** 최대 감지 거리 5.0m.
  * **시각화:** 반투명한 하늘색 원뿔(Cone)로 8방향의 감지 범위 표시.

### 2.3. 관성 센서 (IMU Sensor)
* **IMU 센서 (`imu_sensor`)**
  * **위치:** `base_link` 하단 `imu_sensor_1` 메쉬 위치.
  * **목적:** 동체의 선형 가속도(Linear Acceleration) 및 각속도(Angular Velocity) 측정. 동적 안정성 제어 활용.

---

## 3. 소프트웨어 아키텍처

RAMI의 센서 데이터 처리는 레이어드 아키텍처(Layered Architecture) 패턴을 따르며, 인프라스트럭처 계층에서 하드웨어 데이터를 가공하여 애플리케이션 계층으로 전달합니다.

### 3.1. Infrastructure Layer (Processors)
* **`VisionProcessor` (`src/infrastructure/sensors/vision_processor.py`)**
  * **역할:** 정석적인 영상 처리 파이프라인. `mujoco.Renderer`를 통해 실제 카메라 관점의 RGB 픽셀 배열을 실시간으로 캡처한 뒤, YOLO v8(Nano) 모델을 통해 실제 하드웨어와 100% 동일한 과정으로 Bounding Box, Class, Confidence를 산출합니다.
* **`UltrasonicProcessor` (`src/infrastructure/sensors/ultrasonic_processor.py`)**
  * **역할:** 8채널 거리 데이터 수집 및 최소 거리(`min_dist`) 필터링을 수행합니다. 노이즈 처리 및 비상 정지(Fail-safe) 트리거의 기반이 됩니다.
* **`ImuProcessor` (`src/infrastructure/sensors/imu_processor.py`)**
  * **역할:** 가속도, 자이로스코프 데이터를 파싱하여 딕셔너리 또는 데이터 클래스 형태로 상위 계층에 반환합니다.

### 3.2. Application Layer (Service)
* **`SensorService` (`src/application/services/sensor_service.py`)**
  * **역할:** 세 가지 센서 프로세서를 래핑하여 상위 FSM(상태 머신)이나 컨트롤러에서 손쉽게 센서 통합 정보(`get_sensor_state()`)를 얻을 수 있도록 단일 인터페이스를 제공합니다.

---

## 4. 시각화 지오메트리 규칙
센서의 가시성을 높이기 위해 시뮬레이션 환경에 추가되는 지오메트리(Geometry)는 로봇의 물리적 구동이나 비전 센서의 레이캐스팅(Raycasting)에 간섭해서는 안 됩니다.
* **표준 STL 및 동적 스케일링:** 원뿔(Cone)이나 사각뿔(Pyramid) 형태의 규격화된 Binary STL 자산을 하나만 생성한 뒤, `<mesh scale="...">`을 통해 각 센서 스펙에 맞게 동적으로 크기를 조절하여 재사용합니다.
* **충돌 무시:** 시각화 메쉬의 `contype` 및 `conaffinity`는 0으로 설정하여 물리적 충돌을 절대 발생시키지 않습니다.
* **가시성 격리 및 그림자 제거:** 시각화 전용 형상은 `group="5"` 레이어로 완전히 분리하여 실제 로봇의 오프스크린 카메라 연산에서 제외시킵니다. 또한, 뷰어에서 그림자가 시야를 가리는 현상을 막기 위해 메인 조명에서 `castshadow="false"` 속성을 사용하거나 뷰어 렌더링 설정을 조절합니다.
* **종속성:** 해당 지오메트리들은 로봇의 특정 링크(베이스 또는 팔)에 종속되어 관절의 움직임에 따라 즉각적으로 회전 및 이동해야 합니다.

---

## 5. 단계별 구현 파이프라인 (Phase 3)

센서 시스템은 복잡도를 고려하여 아래와 같이 점진적으로 구현하고 검증합니다.

### 5.1. Step 16: XML 센서 부착 및 시야(FOV) 지오메트리 시각화 (완료)
- **작업 내용:** `rami_world.xml` 수정 및 커스텀 STL 적용.
- **상세:** 
  - `gripper_camera`, `head_camera`, 8개의 초음파(`rangefinder`), 1개의 `imu`(`accelerometer`, `gyro`) 컴포넌트를 정확한 URDF 위치에 장착 완료.
  - 2개의 반투명 노란색 사각뿔(카메라용), 8개의 반투명 하늘색 원뿔(초음파용)을 `group="5"`로 부착하여 물리 및 그림자 간섭 없이 부드럽게 렌더링 되도록 최적화 완료.

### 5.2. Step 17: 센서 프로세서 (Infrastructure) 모듈 구현
- **작업 내용:** 인프라 계층 파이썬 스크립트 작성 (`vision_processor.py`, `ultrasonic_processor.py`, `imu_processor.py`).
- **상세:** 
  - MuJoCo `data.sensordata` 파싱 로직 구현.
  - OpenCV 없이 기하학적 연산(내적)과 `mujoco.mj_ray` 광선 추적을 이용한 치트 어댑터 기반 QR 인식 알고리즘(가안) 프레임워크 작성.

### 5.3. Step 18: 센서 연동 통합 테스트 및 스트리밍 검증
- **작업 내용:** `SensorService` 구현 및 테스트 스크립트(`test_step18_sensors.py`) 작성.
- **상세:** 
  - 테스트 스크립트를 실행하여, 3D 뷰어에서 노란 사각뿔과 하늘색 원뿔이 부드럽게 렌더링되며 관절에 따라 잘 따라다니는지 눈으로 확인.
  - 터미널 출력 창에 초음파 8채널 거리 데이터와 IMU 데이터가 크래시(Crash) 없이 실시간으로 스트리밍 되는지 최종 검증.
