가상환경 활성화
# conda activate RAMI_Project

가상환경 비활성화
# conda deactivate

conda 라이브러리 목록 관리 파일 생성 및 갱신 명령어
# conda env export --from-history > environment.yml
# 해당 명령어는 conda install로 설치한 기록만 확인하고 갱신한다.

pip 라이브러리 목록 관리 파일 생성 및 갱신 명령어
# pip freeze > requirements.txt
# 해당 명령어는 pip install로 설치한 기록만 확인하고 갱신한다.

자주 사용하는 테스트 스크립트 실행 명령어
# python tests/test_base_teleop_cli.py     (CLI 기반 메카넘 휠 주행 수동 제어)
# python tests/test_phase3_step18_sensors.py (라이다, 초음파, 비전 센서 통합 테스트)
# python scripts/run_switch_task.py        (실내 환경 내 자율 스위치 조작 통합 데모)
