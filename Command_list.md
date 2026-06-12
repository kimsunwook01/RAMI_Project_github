# 📌 RAMI 프로젝트 주요 명령어 가이드 (Command List)

본 문서는 프로젝트를 로컬 환경에 재현하고, 핵심 테스트 스크립트들을 실행하기 위해 자주 사용하는 명령어들을 정리한 가이드입니다.

## 1. 가상환경 및 의존성 세팅 (환경 재현)

**가상환경 생성 (최초 1회)**
`environment.yml`을 기반으로 격리된 Python 실행 환경을 구성합니다.
```bash
conda env create -f environment.yml
```

**가상환경 활성화**
```bash
conda activate RAMI_Project
```

**가상환경 비활성화**
```bash
conda deactivate
```

**핵심 모듈 및 의존성 트리 설치**
가상환경이 활성화된 상태에서 프로젝트 실행에 필요한 핵심 패키지들을 설치합니다. (`environment.yml` 생성 시 자동 처리되지만 패키지 추가 시 사용)
```bash
pip install -r requirements.txt
```

---

## 2. 의존성 목록 갱신 관리 (참고용)

**conda 환경 파일 갱신**
```bash
conda env export --from-history > environment.yml
```
*(주의: 이 명령어는 conda install로 직접 설치한 이력만 추출합니다.)*

**pip 패키지 목록 갱신**
```bash
pip freeze > requirements.txt
```
*(주의: 전체 의존성 트리가 덤프되므로 갱신 후에는 프로젝트에서 직접 import 하는 핵심 모듈들 위주로 필터링하는 것을 권장합니다.)*

---

## 3. 테스트 스크립트 실행 명령어

**CLI 기반 메카넘 휠 주행 수동 제어 테스트**
동체 이동 로직과 역기구학 모델을 테스트합니다.
```bash
python tests/test_base_teleop_cli.py
```

**센서 통합 테스트 (초음파, 비전 등)**
시뮬레이터 내에서 각종 센서 데이터가 정상 수집/처리되는지 스트리밍하여 검증합니다.
```bash
python tests/test_phase3_step18_sensors.py
```

**실내 환경 내 자율 스위치 조작 통합 데모 (Phase 5)**
YOLO 및 QR 탐지를 이용해 타겟에 접근하여 스위치를 조작하는 전체 시나리오를 실행합니다.
```bash
python scripts/run_switch_task.py
```
