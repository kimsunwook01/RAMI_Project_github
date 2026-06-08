# RAMI 센서 구현 아키텍처 설계 문서 (Phase 3)

## 1. 개요
본 문서는 RAMI(Right-Arm Mobile Intelligence) 로봇의 **Phase 3: 비전 시스템 및 센서 연동**에 대한 세부 아키텍처와 구현 방침을 기술합니다. 무거운 이미지 프로세싱 부하를 없애고 시뮬레이터의 기하학적 연산을 활용하는 **치트 어댑터 원칙(Cheat Adapter Principle)**을 적용하여 실시간 센서 처리를 달성합니다.

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
  * **역할:** 화각(FOV) 및 거리 조건을 계산하고, `mujoco.mj_ray` 광선 추적을 이용해 장애물 가림(Occlusion) 여부를 판단합니다. 
  * **치트 로직:** 실제 이미지 프레임을 렌더링하지 않고 벡터 내적(Dot product)만으로 시야 내 객체를 판별합니다.
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
* **충돌 무시:** 시각화 메쉬의 `contype` 및 `conaffinity`는 0으로 설정하여 물리적 충돌을 발생시키지 않습니다.
* **시각적 방해 방지:** 반투명 처리(`rgba` 알파 채널 조절)를 통해 다른 객체를 완전히 가리지 않도록 합니다. 
* **종속성:** 해당 지오메트리들은 로봇의 특정 링크(베이스 또는 팔)에 종속되어 관절의 움직임에 따라 즉각적으로 회전 및 이동해야 합니다.
