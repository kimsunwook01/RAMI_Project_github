# RAMI 매니퓰레이터 기구학 및 제어 아키텍처 (Phase 2)

본 문서는 RAMI 로봇의 리프트(Slide) 및 6축 로봇암(Hinge)을 제어하기 위한 자코비안 기반 역기구학(Inverse Kinematics) 알고리즘의 구조와 설계 사상을 명세합니다.

## 1. 아키텍처 핵심 사상
RAMI의 매니퓰레이터는 리프트 관절(1축)과 회전 관절(6축)이 결합된 **7-DOF 잉여 자유도(Redundant)** 시스템입니다. 전통적인 해석적(Analytical) 역기구학 해를 구하는 것은 매우 복잡하며 런타임 제약이 큽니다. 따라서 MuJoCo 물리 엔진이 제공하는 동역학 정보를 활용하여 빠르고 안정적인 **자코비안 기반 수치해석적 역기구학(Numerical IK)** 방식을 채택합니다.

## 2. 제어 대상 조인트 명세
로봇 베이스(`base_link`)로부터 말단부(`gripper_camera_1` 또는 종단점)까지 이어진 조인트 체인은 다음과 같습니다.

1. `lift_slide_joint`: Z축 수직 이동 (Range: -0.62 ~ 0.62)
2. `rotation_joint`: Z축 회전 (Shoulder Pan) (Range: -3.14 ~ 3.14)
3. `arm_joint_1`: -Z축 회전 (Shoulder Pan/Tilt 2) (Range: -1.57 ~ 1.57)
4. `arm_joint_2`: -X축 회전 (Shoulder Pitch) (Range: -1.57 ~ 1.57)
5. `arm_joint_3`: -X축 회전 (Elbow Pitch) (Range: -1.57 ~ 1.57)
6. `arm_joint_4`: -X축 회전 (Wrist Pitch) (Range: -1.57 ~ 1.57)
7. `arm_joint_5`: Y축 회전 (Wrist Roll) (Range: -3.14 ~ 3.14)
8. `arm_joint_6`: X축 회전 (Gripper Pitch) (Range: 0.0 ~ 1.57)

> **총 7자유도** 로봇암에 1자유도 리프트가 결합된 **8-DOF 하이브리드 시스템**입니다. 3차원 위치(XYZ)와 방향(Roll-Pitch-Yaw)을 모두 제어하고도 2개의 여유 자유도를 갖습니다.

## 3. 알고리즘 설계: Damped Least Squares (DLS) IK
단순한 자코비안 의사 역행렬(Pseudo-inverse, $J^{\dagger}$)은 로봇이 특이점(Singularity)에 가까워질 때 조인트 속도가 무한대로 발산하는 치명적인 단점이 있습니다. 이를 방지하기 위해 댐핑 요소($\lambda$)를 추가한 Damped Least Squares(DLS) 방식의 Levenberg-Marquardt 알고리즘을 사용합니다.

### 3.1. 제어 수식 및 가속도 제어 (Trajectory Generation)
- **위치 및 자세 오차 연산**: $e = \begin{bmatrix} e_{pos} \\ e_{ori} \end{bmatrix}$
- **자코비안 행렬 획득**: $J \in \mathbb{R}^{6 \times 7}$ (`mujoco.mj_jacBody` 활용)
- **조인트 각속도 연산 (DLS)**: 
  $\Delta q_{raw} = (J^T J + \lambda^2 I)^{-1} J^T e$
- **가속도 조절 및 스무딩 (Acceleration Control)**:
  급격한 조인트 구동으로 인한 동체 흔들림(Inertial shaking)을 방지하기 위해, 단순한 상수 스텝(Step) 업데이트가 아닌 **가속도 프로파일(예: 사다리꼴 속도 프로파일 또는 최저 저크(Minimum Jerk) 궤적)**을 생성하여 $\Delta q$를 필터링합니다.
  $\ddot{q}_{cmd} = K_p (q_{target} - q_{current}) + K_d (0 - \dot{q}_{current})$ (또는 S-Curve 보간)
- **위치 업데이트 (Integration)**:
  $q_{next} = q_{current} + \Delta q_{smooth}$

### 3.2. 모듈 구성도
```text
src/
├── domain/
│   └── controllers/
│       ├── kinematics.py           # DLS 기반 Numerical IK Solver 구현 (순수 파이썬)
│       └── trajectory.py           # 가속도/속도 제한 및 부드러운 궤적 생성 모듈 (S-Curve 등)
├── application/
│   └── interfaces/
│       └── robot_hardware_io.py    # 암 관절 제어용 프로토콜 추가
└── infrastructure/
    └── simulator/
        └── rami_mujoco_adapter.py  # MJCF의 position/velocity 액추에이터 제어 바인딩
```

## 4. 기구학 구현 전 사전 검증 파이프라인 (총 11단계)
본격적인 IK 제어기를 구현하기 전, 로봇암과 리프트의 개별 조인트 구동이 올바르게 작동하는지 확인하기 위해 다음의 11단계 하드웨어/시뮬레이션 튜닝 및 검증을 거칩니다.

### [Phase A] 충돌 처리 및 초기화 (Step 1~2)
1. **강체 충돌(Collision) 처리:** 로봇의 강체들이 서로 겹쳐지거나 통과하지 않도록 MJCF의 `contype` 및 `conaffinity`를 조정합니다.
2. **초기 자세 유지:** 리프트 조인트와 모든 암 조인트가 초기값(0)을 유지하도록 위치 제어기(Position Actuator)를 세팅하여, 중력의 영향 아래서도 기둥 중앙에서 팔을 뻗고 있는 자세를 단단히 유지합니다.

### [Phase B] 리프트 구동 검증 (Step 3)
3. **리프트 상하 왕복 이동:** 팔을 앞으로 뻗은 자세를 유지하면서 리프트가 상하로 부드럽게 이동하는지 검증합니다.
   * **Step 3-1. 리프트 수동 제어:** 사용자가 직접 값을 입력하여 리프트의 높이를 제어합니다. (이동 범위 초과 시 예외 처리 적용)

### [Phase C] 암 조인트 개별 회전 검증 (Step 4~10)
로봇암의 각 조인트가 허용된 한계 범위 내에서 한 방향으로 회전하다 막히면 반대 방향으로 회전하는 왕복 모션을 검증합니다.
4. **Step 4:** `rotation_joint` 왕복 테스트 (-180° ~ 180°)
5. **Step 5:** `arm_joint_1` 왕복 테스트 (-90° ~ 90°)
6. **Step 6:** `arm_joint_2` 왕복 테스트 (-90° ~ 90°)
7. **Step 7:** `arm_joint_3` 왕복 테스트 (-90° ~ 90°)
8. **Step 8:** `arm_joint_4` 왕복 테스트 (-90° ~ 90°)
9. **Step 9:** `arm_joint_5` 왕복 테스트 (-180° ~ 180°)
10. **Step 10:** `arm_joint_6` 왕복 테스트 (-90° ~ 0°)

### [Phase D] 통합 조인트 텔레오퍼레이션 (Step 11)
11. **통합 조인트 수동 제어:** 사용자가 직접 각 관절의 목표 각도를 입력하여 개별 제어하는 콘솔을 구현합니다. (입력값 범위 초과 및 잘못된 형식에 대한 강력한 예외 처리 포함)

---

## 5. 전신 제어(Whole-body Control) 확장 대비
11단계 검증 과정에서 로봇암 구동 시 발생하는 관성으로 인해 로봇 동체(Base)가 흔들리고 밀리는 현상이 확인되었습니다. 이를 향후 DLS IK 및 전신 제어에 반영하기 위해 하드웨어 인터페이스 레벨에 다음의 피드백 기능을 선제적으로 구축했습니다.
- **`read_arm_joints()`**: 리프트 및 암 7축의 실제 위치 피드백 (테스트 검증 시 상태 머신 전환 판단에 활용됨)
- **`read_wheel_joints()`**: 4개 메카넘 휠의 실제 회전 인코더 피드백
- **`read_base_pose()`**: 가상 `root` 조인트를 활용한 동체(Base)의 절대 좌표 및 방향 피드백

위 11단계 검증과 피드백 인터페이스 구축이 완벽히 끝나면, 이를 기반으로 본격적인 DLS IK 역기구학 모듈을 구현합니다.
