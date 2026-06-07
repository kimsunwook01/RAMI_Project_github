# RAMI 소프트웨어 아키텍처 설계서 (Software Architecture Design)

## 1. 아키텍처 철학 (Design Philosophy)
본 프로젝트는 하드웨어 제어, 물리 시뮬레이션, 사용자 인프라 데이터 세트, 그리고 상위 인지 및 학습 알고리즘이 뒤엉켜 스파게티 코드가 되는 것을 원천 차단하기 위해 **클린 아키텍처 (Clean Architecture)** 사상을 도입합니다.

### 핵심 목표
* **관심사의 분리 (Separation of Concerns):** 물리 시뮬레이터(MuJoCo), 하드웨어 인터페이스(URDF/MJCF), 가상 사용자 데이터셋(JSON)은 RAMI의 핵심 제어 및 학습 로직(FSM 상태 머신, 매핑 알고리즘)과 철저히 분리됩니다.
* **시뮬레이션 독립성 (Simulation Agnostic):** 제어 알고리즘은 MuJoCo API에 직접 의존하지 않습니다. 추상화된 로봇 인터페이스(Protocol)를 통해 명령을 내리므로, 향후 실제 물리 로봇 하드웨어나 Isaac Sim 등 다른 시뮬레이터로 교체하더라도 제어 및 학습 로직 코드는 단 한 줄도 수정하지 않도록 설계합니다.
* **결합도 최소화 (Low Coupling):** 5대 선행 구현 로드맵(동체, 로봇암, 센서, 스위치, 문)에 따라 각 모듈을 다른 계층의 간섭 없이 독립적으로 유닛 테스트할 수 있는 구조를 지향합니다.

---

## 2. 계층형 폴더 구조 (Layered Directory Structure)
디렉토리 구조는 의존성의 방향을 명확히 정의합니다. 의존성은 항상 **바깥쪽 (Infrastructure, Presentation)**에서 **안쪽 (Domain, Core)**으로만 향해야 합니다.

### 2.1 소프트웨어 계층 구조 (`src/`)
```text
src/
├── domain/                      # 1. 도메인 계층 (가장 안쪽, 외부 의존성 없음)
│   ├── models/                  # 순수 Pydantic 데이터 모델 및 엔티티
│   │   ├── robot_state.py       # RAMI의 관절 상태, 동체 위치, 속도 데이터 모델
│   │   ├── environment_map.py   # 3D 가상 지도, 미확인/확인 스위치, 문 후보지 모델
│   │   └── user_config.py       # 사용자 정의 스위치-전등 가상 데이터셋 모델
│   └── controllers/             # 순수 수학적/제어 알고리즘 (I/O 및 시뮬레이터 코드 불가)
│       ├── kinematics.py        # 메카넘 휠 순방향/역방향 기하학 및 암 제어 수식
│       ├── state_machine.py     # 탐색(Coarse) -> 식별(Fine) -> 학습(Learning) FSM 로직
│       └── mapping_processor.py # 댑스/레이저 데이터를 기반으로 공간 특징점을 추출하는 로직
│
├── application/                 # 2. 애플리케이션 계층 (Use Case 비즈니스 흐름)
│   ├── interfaces/              # 외부 인프라 및 시뮬레이터가 구현해야 할 추상 인터페이스
│   │   ├── robot_hardware_io.py # 로봇 액추에이터 구동 및 센서 값 리딩 프로토콜 (Protocol)
│   │   └── config_repository.py # 사용자 정의 데이터셋(JSON) 읽기/쓰기 프로토콜
│   └── services/                # 도메인 제어기와 인터페이스를 조합한 시나리오 흐름 제어
│       ├── exploration_service.py # 집안 전체 3D 매핑 및 스위치 위치 마킹 시나리오
│       ├── identification_service.py # 마킹된 스위치 순회, QR 코드 매칭 및 전등 상관관계 학습
│       └── manipulation_service.py   # 문고리 하향 및 동체 연동 개폐 제어 시나리오
│
├── infrastructure/              # 3. 인프라 계층 (외부 기술 및 어댑터 구현체)
│   ├── simulator/               # MuJoCo 물리 엔진 연동 클래스
│   │   ├── mujoco_client.py     # MuJoCo 시뮬레이션 환경 초기화 및 스텝 실행 싱글톤
│   │   └── rami_mujoco_adapter.py # robot_hardware_io를 상속받아 data.ctrl, data.sensordata 제어
│   ├── dataset/                 # 사용자 초기 정의 데이터 입출력 구현체
│   │   └── json_config_repo.py  # 가상 환경 스위치 정면 사진 기반 사용자 정의 JSON 로드
│   └── sensors/                 # 센서 데이터 파싱 및 필터링 구현체
│       ├── ultrasonic_filter.py # 8채널 초음파 센서 값 중 min() 최단 거리 근사 추출기
│       └── vision_processor.py  # 손목 카메라 시야(FOV) 기반 가상 QR ID 매칭 프로세서
│
└── presentation/                # 4. 프레젠테이션 및 진입점 계층
    ├── terminal_app/            # 파이썬 가상 앱 터미널 인터페이스
    │   └── main.py              # 전체 RAMI 프로젝트 가동 및 연동 시나리오 메인 진입점
    └── visualizer/              # 관찰자 탑뷰 필터링 및 뷰어 렌더링 설정
        └── view_config.py       # 천장 메쉬(group 3, 4) 비가시화 및 카메라 시점 제어
```

### 2.2 전체 프로젝트 루트 구조
```text
RAMI_project/                    # 리포지토리 루트
├── 0_Document/                  # 프로젝트 관련 문서 디렉토리
├── .antigravity/                # 안티그래비티 IDE 로컬 프로젝트 설정 및 대화 세션 기록
├── config/                      # 로봇 가상 환경 및 하드웨어 모델 설정 디렉토리
│   ├── user_config.json         # 사용자 정의 가상 데이터셋 (QR ID, 방 위치, 전등 매핑 등)
│   └── rami_description/        # RAMI 로봇 하드웨어 정의 파일
│       ├── meshes/              # Fusion 360에서 내보낸 동체, 로봇암, 한국식 스위치 STL 파일
│       ├── rami_robot.urdf      # 로봇 결합 구조 URDF
│       └── rami_world.xml       # 가상 실내 환경 및 로봇 액추에이터/센서 통합 MJCF 파일
│
├── src/                         # 소프트웨어 계층 구조 (위 2.1 참고)
├── tests/                       # 선행 구현 5대 모듈별 단위 테스트 코드 (Unit Tests)
│   ├── test_base_movement.py    # Phase 1: 동체 전방향 이동 주행 테스트
│   ├── test_arm_control.py      # Phase 2: 리프트 및 암 위치 제어 테스트
│   ├── test_sensors.py          # Phase 3: 댑스, 초음파 센서 데이터 추출 테스트
│   ├── test_switch_learning.py  # Phase 4: 한국식 똑딱이 스위치 조작 학습 테스트
│   └── test_door_learning.py    # Phase 5: 문 및 문고리 연동 조작 학습 테스트
│
├── README.md                    # 프로젝트 가이드 문서
├── requirements.txt             # Python 패키지 의존성 목록 (pip)
├── environment.yml              # Conda 가상환경 패키지 목록
└── .env                         # 프로젝트 ID 환경 변수
```

---

## 3. 코드 레벨 결합도 낮추기 전략 (Decoupling Strategies)

### 3.1 추상화(Interface) 기반의 시뮬레이터 독립 패턴
RAMI의 상위 비즈니스 흐름(예 : 스위치 찾아가기)이 MuJoCo의 파이썬 라이브러리(`mujoco.MjModel`, `MjData`)를 직접 호출하면 시뮬레이터에 종속됩니다. `typing.Protocol`을 사용하여 하드웨어 입출력을 추상화합니다.

```python
# src/application/interfaces/robot_hardware_io.py
from typing import Protocol, List

class RobotHardwareIO(Protocol):
    def move_base(self, vx: float, vy: float, wz: float) -> None:
        """동체의 전진 속도 v_x 와, 좌우 평행 이동 속도 v_y , 그리고 회전 각속도 ω_z 를 직접 제어하는 인터페이스"""
        pass

    def get_ultrasonic_ranges(self) -> List[float]:
        """8채널 초음파 센서로부터 거리 데이터를 읽어오는 인터페이스"""
        pass
```

### 3.2 의존성 주입 (Dependency Injection)
상위 탐색 서비스는 하위 물리 엔진이 MuJoCo인지 실제 로봇 하드웨어 컨트롤러인지 상관하지 않습니다. 오직 생성자를 통해 주입받은 어댑터 프로토콜을 통해서만 소통합니다.

```python
# src/application/services/exploration_service.py
from src.application.interfaces.robot_hardware_io import RobotHardwareIO

class ExplorationService:
    def __init__(self, robot_io: RobotHardwareIO):
        # 의존성 주입을 통해 결합도를 대폭 낮춤
        self.robot_io = robot_io
        
    def execute_grid_search(self):
        # 센서 데이터를 읽어올 때도 구체 시뮬레이터 API 대신 추상 인터페이스 활용
        ranges = self.robot_io.get_ultrasonic_ranges()
        min_distance = min(ranges)
        
        if min_distance < 0.2: # 장애물 조우 시 비상 정지
            self.robot_io.move_base(0.0, 0.0, 0.0)
        else:
            self.robot_io.move_base(0.3, 0.0, 0.0) # 전진 속도 인가
```

---

## 4. 모듈 간 의존성 규칙 (Dependency Rule)
프로젝트 전반의 일관성과 검토 가독성을 위해 다음 클린 아키텍처 규칙을 엄격히 준수합니다.

> [!CAUTION]
> **개발 준수 및 안티패턴 (Anti-Patterns) 금지 조항**
> 1. `domain` 계층의 물리 수식 및 FSM 코드는 `mujoco` 라이브러리를 절대 import 할 수 없습니다. 모든 물리 제어 명령과 센서 값 피드백은 `application` 계층을 통해 인터페이스 객체로만 중계되어야 합니다.
> 2. `infrastructure/sensors` 의 가상 QR 코드 리더는 실제 이미지 분석 오버헤드를 우회하기 위해, 손목 카메라 오브젝트와 스위치 메쉬 간의 물리적 거리 및 화각(FOV) 연산 조건을 충족할 시 텍스트 ID를 반환하는 **규칙 기반 (Rule-based Cheat)** 어댑터로 콤팩트하게 구현합니다.
> 3. 관찰자 탑뷰를 구현하기 위한 천장 비가시화 설정(`group="3"`)은 `presentation/visualizer` 레이어에서 뷰어 렌더링 필터 플래그 제어를 통해서만 처리하며, 로봇의 물리 충돌 레이어인 `group="4"`는 온전히 보존하여 센서 감지가 이루어지도록 물리적 경계를 격리합니다.

---

## 5. 결론 및 기대 효과
이와 같은 클린 아키텍처 설계를 적용하면, 안티그래비티 IDE 내에서 단 한 명의 싱글 에이전트(부사수)에게 작업을 지시할 때 코드가 수정되는 레이어와 범위가 극도로 제한 됩니다. 

예를 들어, "메카넘 휠 마찰력 때문에 주행이 튀는 현상"이 발생하더라도 `domain` 이나 `application` 레이어는 건드릴 필요 없이 오직 `infrastructure/simulator/rami_mujoco_adapter.py` 와 `rami_world.xml` 파일의 물리 속성만 수정하면 되므로, 전체 제어 시나리오의 흐름을 망가뜨리지 않고 안전하게 **[Phase 1] 동체 움직임 구현** 부터 순차적으로 정복해 나갈 수 있습니다.