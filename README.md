# RAMI (Right-Arm Mobile Intelligence)

> **RAMI(라미)**는 스마트홈 허브나 별도의 가옥 내 인프라 구축 비용 없이, 기존 환경을 그대로 활용(Retrofitting)하여 문 개폐 및 전등 스위치를 유연하게 조작하는 **모바일 매니퓰레이터(Mobile Manipulator) 시뮬레이션 프로젝트**입니다.

## 📖 프로젝트 개요
* **핵심 가치:** 별도 공사 없이 기존 스위치와 문고리를 로봇이 직접 물리적으로 조작할 수 있도록 자율주행 및 매니퓰레이션 알고리즘을 구현합니다.
* **타겟 하드웨어:** 전방향 이동이 가능한 메카넘 휠 기반 모바일 베이스와 단일 우측 암(Single Right-Arm) 매니퓰레이터.
* **시뮬레이션 환경:** 물리 연산의 정밀한 제어 및 검증을 위해 **MuJoCo** 시뮬레이터를 사용합니다.

---

## 🏗 아키텍처 철학 (Clean Architecture)
이 프로젝트는 하드웨어 제어 코드와 시뮬레이터 API, 상위 인지 알고리즘이 뒤엉켜 스파게티 코드가 되는 것을 방지하기 위해 **클린 아키텍처(Clean Architecture)** 사상을 엄격히 따릅니다.
* **관심사의 분리:** 물리 시뮬레이터(MuJoCo)와 상위 제어 로직(FSM 상태 머신 등)은 철저히 분리됩니다.
* **시뮬레이션 독립성:** 로봇 하드웨어 입출력 인터페이스를 Protocol로 추상화하여, 향후 실제 하드웨어나 타 시뮬레이터(Isaac Sim 등)로 이식할 때 제어 로직의 수정을 최소화합니다.

👉 자세한 내용은 [소프트웨어 아키텍처 설계서](0_Document/RAMI_Software_Architecture.md)를 참고하세요.

---

## 📁 주요 디렉토리 구조
```text
RAMI_project/
├── 0_Document/                  # 아키텍처 설계, 기획서 및 컨텍스트 문서
├── config/                      # 로봇 가상 환경 (MJCF, URDF) 및 가상 데이터셋 설정
├── src/                         # 핵심 소프트웨어 계층 (Clean Architecture)
│   ├── domain/                  # 1. 도메인 계층 (데이터 모델, 순수 수학적/제어 알고리즘)
│   ├── application/             # 2. 애플리케이션 계층 (유스케이스 흐름, 인터페이스 정의)
│   ├── infrastructure/          # 3. 인프라 계층 (MuJoCo 어댑터, 센서 규격, JSON 구현체)
│   └── presentation/            # 4. 프레젠테이션 계층 (진입점 및 시각화 설정)
├── tests/                       # 각 모듈별 독립 단위 테스트 코드
├── environment.yml              # Conda 가상환경 패키지 목록
├── requirements.txt             # Pip 패키지 의존성 목록
└── README.md                    # 현재 프로젝트 소개 문서
```

---

## 🛠 개발 로드맵 (Phases)
프로젝트는 모듈 간 결합도를 최소화하고 독립적인 유닛 테스트가 가능하도록 아래의 5단계 로드맵으로 진행됩니다.

* **[Phase 1]** 동체 움직임 구현 (전방향 메카넘 휠 및 가상 조인트) - ✅ 완료
* **[Phase 2]** 로봇 암 및 엔드 이펙터 위치(Position) 제어 구현 - ✅ 완료
* **[Phase 3]** 센서 시스템(댑스, 손목 카메라, 초음파) 레이아웃 및 데이터 추출 - ✅ 완료
* **[Phase 4]** 실내 공간 모델링 및 통합 (환경 최적화, 조명/QR 부착, 물리 디버깅) - ✅ 완료
* **[Phase 5]** 한국식 시소형 똑딱이 스위치 조작법 및 정밀 궤적 학습 - ✅ 완료
* **[Phase 6]** 문고리 회전 및 동체 연동 문 개폐 상태 머신(FSM) 구현 - ⏳ 대기 중

👉 자세한 기획 내용과 예외 처리 사항은 [개발 기획서](0_Document/RAMI_development_design.md)를 참고하세요.

---

## 🚀 시작하기 (Getting Started)

### 환경 설정 및 가상환경 활성화
리포지토리를 클론한 뒤, 제공된 `environment.yml`을 사용하여 Conda 가상환경을 세팅합니다.

```bash
# 가상환경 생성 (RAMI_Project)
conda env create -f environment.yml

# 가상환경 활성화
conda activate RAMI_Project
```

### YOLOv8 모델 다운로드 (최초 1회)
```bash
pip install ultralytics
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

---

## 📚 관련 문서 모음
프로젝트 개발 간 동기화와 원칙 준수를 위해 아래 문서들을 최우선적으로 참조합니다.
* [개발 기획서 (Development Design)](0_Document/RAMI_development_design.md)
* [소프트웨어 아키텍처 설계서 (Software Architecture)](0_Document/RAMI_Software_Architecture.md)
* [AI 어시스턴트 컨텍스트 가이드 (AI Context & Sync)](0_Document/CONTEXT.md)
* [명령어 리스트 (Command List)](Command_list.md)
