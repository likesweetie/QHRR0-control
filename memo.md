sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

if ! ip link show vcan0 > /dev/null 2>&1; then
  sudo ip link add dev vcan0 type vcan
fi

sudo ip link set up vcan0

python3 -m Dashboard.backend.app




python3 -m robot_controller.main --config robot_controller/configs/robot_controller.yaml



너는 로봇 제어기 소프트웨어 프로젝트를 인수인계하기 위한 기술 문서를 작성하는 에이전트다.

목표:
현재 repository를 분석하여 docs/ 아래에 로봇 제어기 handoff 문서를 작성하라.
이 문서는 새로운 개발자가 코드를 수정하거나 실제 로봇에서 실행하기 전에 반드시 알아야 하는 구조, 안전 경계, 실행 절차, 통신 인터페이스, 테스트 방법을 설명해야 한다.

중요 원칙:
0.한눈에 알아볼 수 있는 요약본을1~2페이지 분량으로 만들어라.
 - 이 요약본은 프로젝트 디자인의 핵심철학과 의도, 주의점을 엔지니어에게 효율적으로 전달할 수 있어야 한다.

1. 추측하지 말 것.
   - 코드에서 확인 가능한 내용만 확정적으로 적어라.
   - 불명확한 내용은 `UNKNOWN:` 또는 `TODO(owner):`로 표시하라.
   - 임의로 안전 동작, fallback, default 값을 만들어내지 말 것.

2. 로봇 안전 관련 항목은 가장 보수적으로 문서화할 것.
   - 예외를 조용히 무시하는 fallback을 권장하지 말 것.
   - fallback이 존재하면 trigger condition, action, recovery condition, log message, test case를 반드시 표로 정리하라.
   - safety-critical parameter가 config default로 조용히 대체되는 구조가 있으면 위험 항목으로 표시하라.

3. 코드 설명보다 런타임 구조를 우선할 것.
   - 어떤 프로세스가 뜨는지
   - 누가 어떤 프로세스를 시작/종료하는지
   - 어떤 데이터가 어디서 어디로 흐르는지
   - 어떤 프로세스가 죽으면 전체 시스템이 어떻게 반응하는지
   - shared memory, CAN, file log, socket 등의 경계를 명확히 설명하라.

4. 문서는 한국어로 작성하되, 코드 식별자, 파일명, 클래스명, config key는 원문 그대로 유지하라.

5. Mermaid diagram을 사용해 다음 그림을 포함하라.
   - 전체 프로세스 구조
   - 데이터 흐름
   - startup sequence
   - shutdown sequence
   - fault state transition

생성할 파일:
- docs/HANDOFF.md
- docs/ARCHITECTURE.md
- docs/RUNBOOK.md
- docs/SAFETY.md
- docs/CONTROL_LOOP.md
- docs/IPC_SHM.md
- docs/CAN_INTERFACE.md
- docs/CONFIG_SCHEMA.md
- docs/LOGGING.md
- docs/TEST_PLAN.md
- docs/CHANGELOG.md

각 파일 요구사항:

1. docs/HANDOFF.md
   - 프로젝트 목적
   - 책임 범위와 비책임 범위
   - 빠른 시작
   - 문서 맵
   - 실제 로봇 실행 전 checklist
   - 가장 중요한 위험 경고 5개

2. docs/ARCHITECTURE.md
   - 디렉토리 구조
   - 주요 모듈과 클래스
   - 프로세스/스레드 구조
   - ProcessSupervisor의 책임
   - CAN daemon의 책임
   - SafetyManager가 있다면 그 책임과 제외 범위
   - simulation mode와 hardware mode의 차이
   - Mermaid architecture diagram 포함

3. docs/RUNBOOK.md
   - 개발 환경 설치
   - build 절차
   - simulation 실행 방법
   - hardware 실행 방법
   - 정상 종료 절차
   - 비정상 종료 후 복구 절차
   - log 확인 방법
   - 자주 쓰는 command 예시

4. docs/SAFETY.md
   - system state 정의: idle, ready, running, damping, fault, emergency_stop 등
   - 각 state 진입 조건과 탈출 조건
   - fault table 작성
   - fallback policy 작성
   - command timeout 정책
   - NaN/Inf action 처리 정책
   - CAN timeout 처리 정책
   - SHM corruption/version mismatch 처리 정책
   - 실제 로봇 실행 전 safety checklist

5. docs/CONTROL_LOOP.md
   - main loop frequency
   - policy inference frequency
   - decimation
   - observation 생성 순서
   - action 해석 방식
   - PD 제어식
   - joint order
   - 단위
   - 좌표계
   - saturation/limit
   - torque/position/velocity command 흐름
   - timing diagram 포함

6. docs/IPC_SHM.md
   - shared memory 생성 주체
   - shared memory 이름
   - lifetime
   - reader/writer ownership
   - struct layout
   - field table
   - magic number/version field 여부
   - synchronization 방식
   - Python/C++ ABI 주의사항
   - struct 변경 규칙

7. docs/CAN_INTERFACE.md
   - CAN interface name
   - bitrate
   - classic CAN/CAN FD 여부
   - device table
   - CAN ID table
   - payload layout
   - unit
   - timeout
   - bus-off 처리
   - candump/cansend 디버깅 방법
   - vcan 기반 테스트 방법

8. docs/CONFIG_SCHEMA.md
   - YAML config 파일 목록
   - 각 key의 의미
   - type
   - unit
   - default 여부
   - required 여부
   - safety-critical 여부
   - validation rule
   - 잘못된 config 예시와 기대 동작

9. docs/LOGGING.md
   - log directory 구조
   - 각 log file의 의미
   - CSV field table
   - timestamp 기준
   - decimation
   - supervisor log
   - CAN daemon log
   - debugging guide

10. docs/TEST_PLAN.md
   - smoke test
   - unit test
   - integration test
   - simulation test
   - hardware dry-run test
   - fault injection test
   - acceptance criteria
   - 각 test의 command와 expected result

11. docs/CHANGELOG.md
   - 사람이 읽기 쉬운 changelog 형식으로 작성
   - Added / Changed / Fixed / Removed / Safety 항목 사용
   - 아직 release가 없다면 [Unreleased]만 작성

분석 방법:
1. repository의 README, config, launch script, main entrypoint를 먼저 읽어라.
2. main loop, ProcessSupervisor, CAN daemon, shared memory, logger, config loader 관련 파일을 찾아라.
3. 실제 실행 명령을 코드와 스크립트에서 확인하라.
4. 불확실한 값은 절대 만들어내지 말고 UNKNOWN으로 표시하라.
5. 문서 마지막에 “검증 필요 항목” 섹션을 만들고, 사람이 확인해야 할 질문을 정리하라.

출력 품질 기준:
- 복붙 가능한 command를 포함할 것.
- 표를 적극적으로 사용할 것.
- Mermaid diagram을 포함할 것.
- safety-critical 항목은 굵게 표시할 것.
- 임의의 안전 보장 표현을 쓰지 말 것.
- “안전하게 처리됨”, “적절히 처리됨” 같은 모호한 표현을 피하고, 정확한 조건과 동작을 적을 것.


금지 사항:

- 코드를 수정하지 말 것. 문서만 작성할 것.
- 확인되지 않은 실행 명령을 만들어내지 말 것.
- safety fallback을 새로 제안하면서 현재 구현된 것처럼 쓰지 말 것.
- default config 값을 추측하지 말 것.
- TODO를 숨기지 말 것.
- "probably", "seems", "maybe"를 확정 문장처럼 번역하지 말 것.
- 실제 로봇에서 안전하다고 단정하지 말 것.