# RAMI 모바일 베이스 구동 아키텍처 설계

본 문서는 RAMI 로봇의 모바일 베이스를 구동하기 위한 가상 관절 제어 및 메카넘 휠 역기구학 시각적 렌더링에 관한 11단계 설계서입니다.

## 1. 아키텍처 핵심 사상
로봇의 메카넘 휠의 물리적 마찰과 슬립(Slip)을 시뮬레이터에서 완벽하게 모사하는 것은 연산 오버헤드가 크고 제어기 튜닝이 어렵습니다. 따라서 본 프로젝트는 **물리적 안정성과 시각적 정확성을 분리하는 하이브리드 구동 방식**을 채택합니다.

* **물리 이동 제어 (Virtual Joints):** `base_link`에 X(Slide), Y(Slide), Z(Hinge) 가상 관절을 부여하고 Velocity Actuator를 통해 제어합니다. 바퀴의 물리적 충돌체(Collision)는 비활성화하여, 바닥과의 마찰 없는 무결점의 옴니(Omni-directional) 주행을 달성합니다.
* **시각적 바퀴 동기화 (Mecanum Kinematics):** 4개의 바퀴는 물리적으로 지면을 밀어내지 않지만, 로봇의 이동 속도(vx, vy, wz)에 맞춰 메카넘 역기구학 모델에 의해 정확한 각속도로 회전시킵니다. 사용자의 눈에는 완벽한 메카넘 휠 구동으로 렌더링됩니다.

## 2. 모듈 구성도
```text
src/
├── domain/
│   └── controllers/
│       └── mecanum_controller.py      # 역기구학 수식 계산 (vx, vy, wz -> 4 wheel speeds)
├── application/
│   └── interfaces/
│       └── robot_hardware_io.py       # 하드웨어 제어 Protocol 추상화
└── infrastructure/
    └── simulator/
        ├── mujoco_client.py           # MuJoCo 환경 및 뷰어 로드
        └── rami_mujoco_adapter.py     # data.ctrl을 통한 액추에이터 제어 구현체
```

## 3. 단계별 구현 및 검증 파이프라인 (총 11단계)

안정적인 개발을 위해 단계마다 독립된 테스트 코드를 작성하고 검증합니다.

### [Phase A] 렌더링 보정 및 휠 단독 구동
1. **지면 밀착 렌더링:** `base_link`의 초기 `pos`의 Z축을 `-0.015m`로 하강시켜 100mm 직경 휠이 지면에 완벽히 닿도록 보정. (`test_step1_rendering.py`)
2. **바퀴의 부드러운 회전:** 휠 액추에이터 `kv` 및 관절 `damping` 최적화를 통해 버벅거림(Stuttering) 없는 회전 달성. (`test_step2_wheel_rotation.py`)
3. **개별 휠 수동 컨트롤:** 뷰어 환경에서 사용자가 각 휠을 직접 조작해 볼 수 있는 쉘 제공. (`test_step2_1_wheel_control.py`)

### [Phase B] 키네마틱스 제어기 설계
4. **메카넘 제어 모듈:** `MecanumController` 클래스 설계. 기하학적 파라미터(축거, 윤거, 바퀴 반경) 기반 수식 도입.

### [Phase C] 단방향 및 회전 이동 (가상 관절 + 휠 동기화)
5. **전진 구동:** (`test_step4_forward.py`)
6. **후진 구동:** (`test_step5_backward.py`)
7. **제자리 회전:** (`test_step6_rotation.py`)

### [Phase D] 대각선 이동 
8. **좌측 전진 대각 이동:** (`test_step7_diag_FL.py`)
9. **우측 전진 대각 이동:** (`test_step8_diag_FR.py`)
10. **좌측 후진 대각 이동:** (`test_step9_diag_BL.py`)
11. **우측 후진 대각 이동:** (`test_step10_diag_BR.py`)

### [Phase E] 통합 텔레오퍼레이션 (Teleoperation)
12. **통합 조종:** 키보드를 이용한 8방향 옴니 주행 및 회전 통합 제어. (`test_step11_teleop.py`)
