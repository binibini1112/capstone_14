# YOLO와 FPGA 기반 지능형 드론 추격 시스템


2026. 05. 17.









한성대학교  
전자트랙  
14조










종합설계 프로젝트 보고서




YOLO와 FPGA 기반 지능형 드론 추격 시스템











작 성 자 :  14조  
지도교수 :  정 영 모 교수님




# 차   례

I. 프로젝트 개요  
II. 시스템 구성  
III. 각 모듈별 동작 원리  
IV. 모듈별 설계  
V. 전체 시스템 설계  
VI. 제작내용 (회로도, 소스코드 등 첨부)  
VII. 결과물 설명 (결과물 관련 이미지(사진) 첨부)  
VIII. 프로젝트 수행 결과 분석  
    1. 재학중 취득한 기초지식의 활용 내용  
    2. 재학중 취득한 실험지식의 활용 내용  
    3. 본 프로젝트 수행과정에서의 설계 능력 향상 내용  
    4. 본 프로젝트 수행과정에서의 문제 해결 내용  
    5. 본 프로젝트 수행과정에서의 실무 능력 향상 내용  
    6. 본 프로젝트 수행과정에서의 팀원간 협동 내용  
    7. 개발된 결과물에 대한 전시 방법 계획  

< 참고 문헌 >  
< 종합설계 프로젝트 수행 후기 >


# I. 프로젝트 개요

본 프로젝트의 목표는 카메라와 음향 센서를 이용하여 드론을 인식하고, 인식된 목표를 팬틸트 장치가 실시간으로 추적하도록 하는 지능형 드론 추격 시스템을 구현하는 것이다. Jetson 보드는 영상 인식 및 음향 인식과 같이 연산량이 큰 인공지능 처리를 담당하고, Ultra96 FPGA 보드는 추적 제어 연산과 모터 제어 중계 역할을 담당한다. 최종적으로 드론이 카메라 영상 중앙에 유지되도록 pan/tilt Dynamixel 모터를 제어하는 시스템을 제작한다.

기존 Jetson 단독 제어 방식은 YOLO 추론과 모터 제어가 한 장치에서 동시에 수행되기 때문에 추론 부하가 커질 때 모터 명령 지연이나 누락이 발생할 수 있다. 반대로 FPGA PL이 Dynamixel UART 통신까지 직접 담당하는 방식은 회로 복잡도와 전기적 위험이 커진다. 본 프로젝트에서는 두 방식의 단점을 줄이기 위해 Jetson, Ultra96 PS, Ultra96 PL의 역할을 분리하였다.

최종 목표 구조는 다음과 같다.

```text
Jetson YOLO bbox
-> UDP 5016
-> Ultra96 PS bridge
-> AXI 0xA0000000
-> Ultra96 PL bbox error/goal 계산
-> Ultra96 PS readback
-> USB/U2D2
-> Dynamixel pan/tilt motor
```

YOLO가 목표를 검출하지 못하는 경우에는 오디오 방향 추정 결과를 fallback으로 사용한다.

```text
Jetson ReSpeaker/Tello audio angle
-> UDP 5016
-> Ultra96 PS angle-to-pan 변환
-> USB/U2D2
-> Dynamixel pan motor
```

이 구조를 통해 영상 기반 목표 추적을 1순위로 사용하고, 영상 검출이 불안정한 순간에는 오디오 방향 정보를 이용하여 추적 방향을 보조한다.


# II. 시스템 구성

전체 시스템은 크게 Jetson 인식부, Ultra96 제어부, 팬틸트 구동부, 사용자 표시 및 검증부로 구성된다.

Jetson 인식부는 카메라 영상을 입력받아 YOLO 모델로 드론의 bounding box를 검출한다. 검출된 bounding box의 중심 좌표, 너비, 높이, confidence, 프레임 크기 정보를 Ultra96로 전송한다. 또한 ReSpeaker 또는 Tello audio 모델을 이용하여 목표의 방향각을 계산하고, YOLO 검출 실패 시 오디오 fallback 데이터로 전송한다.

Ultra96 제어부는 PS와 PL로 역할을 나누었다. PS는 UDP 패킷 수신, AXI-Lite register write/read, USB/U2D2 Dynamixel 통신, 오디오 방향각의 pan goal 변환을 담당한다. PL은 영상 bounding box 중심 오차를 이용하여 pan/tilt goal을 계산한다. 즉 PL은 빠르고 반복적인 추적 연산에 집중하고, PS는 운영체제 및 장치 드라이버가 필요한 통신 처리를 담당한다.

팬틸트 구동부는 Dynamixel 모터 2개를 사용하여 pan축과 tilt축을 제어한다. Ultra96 PS는 U2D2 USB 인터페이스를 통해 Dynamixel goal position을 송신한다. 제어 명령은 목표 중심이 화면 중앙에서 얼마나 벗어났는지를 기준으로 생성된다.

사용자 표시 및 검증부는 Jetson 화면에 YOLO detection box, 중심 reticle, 추적 상태, FPS, Ultra96 응답 상태를 표시한다. 또한 pipeline echo와 JSON 로그를 통해 실제 명령 지연, 검출률, `src=pl` 또는 `src=ps_direct` 여부를 확인할 수 있도록 구성하였다.


# III. 각 모듈별 동작 원리

## 1. 영상 인식 모듈

영상 인식 모듈은 카메라 프레임을 입력받고 YOLO 기반 드론 검출 모델을 실행한다. 모델은 화면 안의 드론 후보 bounding box를 산출하며, confidence 기준을 통과한 결과 중 추적에 가장 적합한 box를 선택한다. 선택된 box의 중심 좌표 `cx`, `cy`와 크기 `bw`, `bh`는 화면 중심과의 오차 계산에 사용된다.

목표 중심 오차는 다음과 같이 정의된다.

```text
error_x = bbox_cx - frame_width / 2
error_y = bbox_cy - frame_height / 2
```

`error_x`가 양수이면 목표가 화면 오른쪽에 있으므로 pan 모터가 오른쪽으로 이동해야 하고, 음수이면 왼쪽으로 이동해야 한다. `error_y`는 tilt 방향 보정에 사용된다.

실제 드론 추적에서는 YOLO box가 순간적으로 튀거나 화면 가장자리의 잘못된 box가 선택될 수 있다. 이를 줄이기 위해 edge box filter, large jump reject, confidence threshold를 적용하였다. 단, Jetson 쪽 smoothing을 과하게 적용하면 추적 지연이 커지므로 최종 구조에서는 무거운 smoothing은 줄이고 PL 제어 연산이 중심 역할을 하도록 설계하였다.

## 2. 오디오 fallback 모듈

오디오 fallback 모듈은 YOLO가 목표를 검출하지 못하는 순간에 목표 방향을 유지하거나 다시 탐색하기 위한 보조 기능이다. ReSpeaker 또는 Tello audio 모델을 통해 방향각을 산출하고, 해당 각도를 Ultra96 PS로 전달한다. PS는 방향각을 pan goal로 변환하여 pan 모터를 이동시킨다.

오디오 정보는 영상보다 위치 정밀도가 낮으므로 pan/tilt 정밀 추적에는 사용하지 않고, YOLO 미검출 시 탐색 방향을 보조하는 용도로 제한하였다. 이를 통해 잘못된 오디오 방향이 직접적인 공격적 모터 움직임을 만드는 위험을 줄였다.

## 3. Ultra96 PS bridge 모듈

Ultra96 PS bridge는 Jetson에서 전송한 UDP 패킷을 수신한다. YOLO target command인 `T` 명령을 받으면 bbox 정보를 PL register에 기록하고, PL이 계산한 pan/tilt goal을 다시 읽어온다. 이후 U2D2를 통해 Dynamixel goal position을 전송한다.

PS bridge의 주요 기능은 다음과 같다.

- UDP `5016` 포트 수신
- `PING`, `PLPING`, `G`, `T`, `A` 명령 처리
- AXI-Lite register write/read
- PL lazy-open을 통한 안정성 확보
- USB/U2D2 Dynamixel sync write
- 오디오 angle-to-pan fallback 처리

PL 접근은 시스템 시작 즉시 수행하지 않고, 실제 `PLPING` 또는 `T` 명령이 들어올 때 열도록 lazy-open 방식으로 설계하였다. 이는 잘못된 AXI 접근으로 Ultra96 USB Ethernet이나 SSH가 끊기는 문제를 줄이기 위한 안정성 설계이다.

## 4. Ultra96 PL 추적 연산 모듈

PL 모듈은 AXI-Lite slave 형태로 설계되었다. PS가 현재 pan/tilt goal, bbox 중심 좌표, frame size, confidence, valid flag 등을 register에 기록하면, PL은 화면 중심과 목표 중심의 오차를 계산하고 다음 pan/tilt goal을 산출한다.

PL 제어는 단순한 비례 제어 기반으로 구성하였다.

```text
pixel_error -> deadband 적용 -> gain 적용 -> max correction clamp -> next goal
```

deadband는 목표가 화면 중앙 근처에 있을 때 불필요한 미세 진동을 줄이기 위해 사용한다. max correction clamp는 잘못된 box나 급격한 움직임으로 인해 모터가 한 번에 과도하게 이동하는 것을 막는다. pan과 tilt의 물리적 특성이 다르기 때문에 X축과 Y축의 correction limit을 분리하였다.

## 5. 팬틸트 모터 제어 모듈

팬틸트 장치는 Dynamixel 모터 2개로 구성된다. pan 모터는 좌우 방향을, tilt 모터는 상하 방향을 담당한다. Ultra96 PS는 U2D2 USB 인터페이스를 이용하여 Dynamixel Protocol 기반 goal position 명령을 보낸다.

모터 제어에서는 다음 요소를 고려하였다.

- center position 기준 pan/tilt offset 계산
- goal position min/max 제한
- profile velocity 및 acceleration 설정
- U2D2 serial write 시간 고려
- 잘못된 bbox에 의한 급격한 이동 방지

## 6. UI 및 로그 모듈

Jetson UI는 실시간 카메라 화면 위에 target box, 화면 중앙 reticle, FPS, 검출 상태, Ultra96 응답 정보를 표시한다. 디버깅을 위해 pipeline echo 옵션을 제공하며, 필요 시 JSONL 로그로 프레임별 상태를 저장한다.

대표 실행 명령은 다음과 같다.

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```


# IV. 모듈별 설계

## 1. Jetson 소프트웨어 설계

Jetson 소프트웨어는 Python 기반으로 구성하였다. `jetson/jetson_node.py`가 메인 실행 노드이며, 카메라 입력, YOLO 추론, target selection, UI 표시, Ultra96 송신을 담당한다. 제어 송신부는 `jetson/src/control/ultra_yubin_motor.py`에 분리하여 UDP request/reply, command period, smoothing parameter, 응답 파싱을 관리한다.

주요 설계 변수는 환경변수로 조정할 수 있도록 하였다. 이를 통해 bitstream을 다시 빌드하지 않고도 YOLO confidence, command period, smoothing alpha, target jump threshold 등을 데모 현장에서 조정할 수 있다.

## 2. Ultra96 PS bridge 설계

PS bridge는 C 언어로 작성하였다. Linux userspace에서 UDP socket, `/dev/mem` AXI 접근, USB serial 통신을 처리한다. PL 경로가 불안정할 때는 PS direct mode 또는 no-PL mode로 동작할 수 있게 하여 단계별 검증이 가능하도록 하였다.

검증 순서는 다음과 같이 설계하였다.

```text
PING -> PLPING -> G pan tilt -> T bbox -> Jetson 실시간 입력
```

`PING`은 PL 접근 없이 PS bridge가 살아 있는지 확인한다. `PLPING`은 AXI read를 통해 PL register 접근을 확인한다. `G`는 직접 pan/tilt goal을 전송하여 U2D2 및 Dynamixel 경로를 확인한다. `T`는 실제 bbox 기반 PL 계산과 모터 명령을 함께 검증한다.

## 3. Ultra96 PL RTL 설계

PL RTL은 AXI-Lite register interface와 goal compute logic으로 구성하였다. register write/read handshake가 안정적으로 수행되도록 AXI-Lite timing을 수정하였고, write response hang 및 readback timing 문제를 해결하는 방향으로 개선하였다.

추적 제어에서는 X/Y 축의 correction limit을 분리하였다. pan은 드론의 좌우 움직임을 빠르게 따라가야 하므로 상대적으로 큰 correction을 허용하고, tilt는 화면 상하 방향 오차에 과하게 반응하지 않도록 더 작은 correction을 적용하였다.

## 4. 배포 및 빌드 설계

Vivado bitstream은 Windows 환경에서 빌드하고 Jetson으로 전송하는 흐름을 사용하였다. Jetson에서는 bitstream load, Ultra96 PS bridge 배포, 서비스 재시작을 스크립트로 자동화하였다.

주요 스크립트는 다음과 같다.

- `tools/windows/build_and_send.ps1`: Vivado build 및 전송
- `tools/load_bitstream_only.sh`: bitstream load 단독 수행
- `tools/deploy_ultra96_ps_usb.sh`: Ultra96 PS bridge 배포 및 재시작
- `run_demo.sh`: 최종 통합 데모 실행
- `run_demo_audio.sh`: 오디오 fallback 포함 데모 실행
- `run_demo_ps_safe.sh`: 안전한 PS 중심 검증 실행
- `run_demo_pl_drive.sh`: PL 추적 경로 검증 실행


# V. 전체 시스템 설계

전체 시스템은 실시간성을 확보하기 위해 각 장치의 강점에 맞게 역할을 나누었다. Jetson은 GPU 기반 인공지능 추론에 적합하므로 YOLO와 오디오 모델을 담당한다. Ultra96 PL은 작은 정수 연산을 낮은 지연으로 반복하는 데 적합하므로 bbox 오차 기반 goal 계산을 담당한다. Ultra96 PS는 Linux 환경에서 UDP, USB, AXI 접근을 안정적으로 처리할 수 있으므로 통신 중계 역할을 담당한다.

데이터 흐름은 최신 상태 기반으로 설계하였다. 드론 추적에서는 오래된 명령이 누적되는 것보다 최신 frame의 목표 위치를 반영하는 것이 중요하다. 따라서 UDP 패킷은 상태 snapshot으로 처리하고, bridge에서는 가능한 최신 명령을 처리하도록 구성하였다.

안전성 측면에서는 다음 설계를 반영하였다.

- PL 접근 전 `PING`, `PLPING` 단계 검증
- no-PL mode와 PS direct mode 제공
- motor goal min/max 제한
- edge box 및 large jump reject
- laser auto-on 비활성 기본값
- 오디오 fallback은 pan 탐색 보조로 제한
- PL lazy-open으로 Ultra96 USB Ethernet 안정성 확보


# VI. 제작내용 (회로도, 소스코드 등 첨부)

## 1. 하드웨어 구성

사용 부품은 다음과 같다.

- Jetson 계열 보드: YOLO 및 오디오 AI 추론
- Ultra96-V2: PS/PL 기반 제어 중계 및 추적 연산
- USB Ethernet: Jetson과 Ultra96 통신
- U2D2: Ultra96 PS와 Dynamixel 통신
- Dynamixel 모터 2개: pan/tilt 구동
- 카메라: 드론 영상 입력
- ReSpeaker 또는 오디오 입력 장치: 방향 추정 fallback

회로 연결 개념도는 다음과 같다.

```text
Camera/Audio
    |
    v
Jetson YOLO/Audio
    |
    | UDP 5016
    v
Ultra96 PS bridge
    |                 \
    | AXI-Lite         \ USB serial
    v                  v
Ultra96 PL          U2D2 -> Dynamixel Pan/Tilt
goal compute
```

## 2. 주요 소스코드

본 프로젝트의 주요 구현 파일은 다음과 같다.

- `jetson/jetson_node.py`: Jetson 메인 추적 노드
- `jetson/src/control/ultra_yubin_motor.py`: Ultra96 UDP 모터 컨트롤러
- `jetson/src/audio_fallback.py`: 오디오 fallback 처리
- `jetson/src/vision/vision_tracker.py`: 영상 추적 관련 처리
- `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c`: Ultra96 PS UDP/AXI/U2D2 bridge
- `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v`: Ultra96 PL AXI-Lite goal compute RTL
- `hardware/pl_goal_compute/tb/pl_goal_compute_axi_tb.v`: PL testbench
- `tools/deploy_ultra96_ps_usb.sh`: Ultra96 배포 스크립트
- `run_demo.sh`: 최종 데모 실행 스크립트

## 3. 대표 제어 패킷

Jetson에서 Ultra96로 전송하는 target command는 bbox 중심과 frame 크기 정보를 포함한다.

```text
T cx cy bw bh fw fh conf valid distance laser_base
```

Ultra96 bridge는 이에 대해 pan/tilt goal, source, USB write 성공 여부 등을 포함한 응답을 반환한다. 실험 시 `src=pl`이 표시되면 PL 기반 goal 계산 경로가 사용되고 있음을 의미한다.

## 4. 빌드 및 검증 명령

PL RTL 시뮬레이션:

```bash
iverilog -g2012 -o /tmp/ultra_yubin_v1_pl_goal_tb \
  hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v \
  hardware/pl_goal_compute/tb/pl_goal_compute_axi_tb.v
vvp /tmp/ultra_yubin_v1_pl_goal_tb
```

PS bridge 문법 검증:

```bash
gcc -Wall -Wextra -fsyntax-only hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c
```

Jetson Python 문법 검증:

```bash
python3 -m py_compile jetson/jetson_node.py jetson/src/config.py \
  jetson/src/ui/display.py jetson/src/control/ultra_yubin_motor.py
```

최종 데모 실행:

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```


# VII. 결과물 설명 (결과물 관련 이미지(사진) 첨부)

본 프로젝트의 결과물은 드론을 실시간으로 인식하고 팬틸트 장치가 드론 방향을 추적하는 통합 시스템이다. 카메라 영상에서 드론이 검출되면 Jetson 화면에는 bounding box와 중심 reticle이 표시되고, Ultra96는 해당 좌표를 이용하여 pan/tilt 모터를 제어한다.

결과물의 주요 기능은 다음과 같다.

- YOLO 기반 드론 검출
- 화면 중심 기준 target error 계산
- Ultra96 PS/PL 기반 pan/tilt goal 계산 및 전송
- Dynamixel 팬틸트 추적
- YOLO 미검출 시 오디오 방향 fallback
- 실시간 UI overlay
- pipeline 로그 및 latency 분석
- PL/PS 경로별 안전 검증 모드

첨부할 이미지 및 사진 목록은 다음과 같다.

1. 전체 시스템 구성 사진: Jetson, Ultra96, U2D2, Dynamixel 팬틸트 장치 연결 사진
2. 카메라 영상 UI 사진: YOLO bounding box와 center reticle이 표시된 화면
3. Ultra96 연결 사진: USB Ethernet 및 U2D2 연결 상태
4. Vivado block design 또는 RTL 구조 이미지
5. 데모 실행 로그 화면: `src=pl`, FPS, confidence, pan/tilt 응답 표시
6. 드론 추적 시연 사진: 드론 위치 변화에 따라 팬틸트가 추적하는 장면


# VIII. 프로젝트 수행 결과 분석

## 1. 재학중 취득한 기초지식의 활용 내용

본 프로젝트에서는 전자회로, 디지털논리회로, 마이크로프로세서, 신호 및 시스템, 제어공학, 통신공학에서 학습한 기초지식을 활용하였다. 화면 중심 오차를 기반으로 pan/tilt 제어량을 계산하는 과정에서는 좌표계 변환과 비례 제어 개념을 적용하였다. FPGA RTL 설계에서는 레지스터, 클록, reset, AXI-Lite handshake와 같은 디지털 시스템 지식을 활용하였다. UDP 통신과 serial 통신에서는 네트워크 및 직렬 통신의 기본 원리를 적용하였다.

## 2. 재학중 취득한 실험지식의 활용 내용

실험 수업에서 수행한 계측, 디버깅, 단계별 검증 방법을 프로젝트에 적용하였다. 전체 시스템을 한 번에 검증하지 않고 `PING`, `PLPING`, `G`, `T`, 실시간 YOLO 입력 순서로 나누어 확인하였다. 또한 로그를 기반으로 FPS, 검출률, Ultra96 응답, 모터 움직임을 비교하여 문제 원인을 분리하였다.

## 3. 본 프로젝트 수행과정에서의 설계 능력 향상 내용

처음에는 Jetson이 모든 기능을 처리하는 구조와 PL이 직접 모터 통신까지 담당하는 구조를 검토하였다. 최종적으로는 Jetson, Ultra96 PS, Ultra96 PL의 역할을 분리하는 구조를 선택하였다. 이 과정에서 단순히 기능 구현만 고려하는 것이 아니라 실시간성, 안정성, 구현 난이도, 전기적 위험, 시연 가능성을 함께 고려하는 설계 능력을 향상시켰다.

## 4. 본 프로젝트 수행과정에서의 문제 해결 내용

프로젝트 수행 중 PL AXI register 접근 시 Ultra96 USB Ethernet 또는 SSH 연결이 불안정해지는 문제가 발생하였다. 이를 해결하기 위해 PL 접근을 시작 시점에 바로 수행하지 않고 명령이 들어올 때만 수행하는 lazy-open 구조를 적용하였다. 또한 AXI-Lite write/readback timing 문제를 분석하고 RTL handshake를 수정하였다.

드론 추적에서는 검출 latency와 모터 반응 지연이 문제였다. 분석 결과 FPGA 연산 자체보다는 카메라 frame age, YOLO inference, Jetson smoothing, U2D2 serial write, Dynamixel 물리 응답이 주요 지연 요인임을 확인하였다. 이에 따라 Jetson smoothing을 줄이고, edge filter와 large jump reject만 유지하며, PL 제어 상수를 조정하는 방식으로 추적 응답을 개선하였다.

## 5. 본 프로젝트 수행과정에서의 실무 능력 향상 내용

본 프로젝트에서는 Python, C, Verilog, shell script, PowerShell, Vivado, Linux network, serial device, TensorRT 모델 등을 함께 다루었다. 단일 언어 또는 단일 보드 중심의 개발이 아니라 여러 장치와 소프트웨어 계층을 연결하는 통합 개발 경험을 얻었다. 또한 로그 기반 분석, 환경변수 기반 튜닝, 배포 자동화, fallback mode 설계 등 실제 임베디드 시스템 개발에 가까운 방법을 학습하였다.

## 6. 본 프로젝트 수행과정에서의 팀원간 협동 내용

팀원들은 영상 인식, 오디오 처리, Ultra96/FPGA, 모터 제어, 시연 준비 역할을 나누어 개발을 진행하였다. 각 기능은 독립적으로 개발하되 최종적으로 UDP command contract, telemetry format, 모터 goal 기준, 모델 파일 경로 등을 맞추어 통합하였다. 문제가 발생했을 때는 로그와 재현 조건을 공유하여 원인을 특정하고, 한 팀원이 작성한 기준 코드를 다른 팀원이 참고하여 안정적인 구조로 개선하였다.

## 7. 개발된 결과물에 대한 전시 방법 계획

전시에서는 카메라 앞에서 드론 또는 드론 모형을 움직이고, 팬틸트 장치가 이를 추적하는 모습을 시연한다. 화면에는 YOLO bounding box, center reticle, FPS, confidence, Ultra96 응답을 표시한다. 관람자가 시스템 구조를 이해할 수 있도록 Jetson, Ultra96, U2D2, Dynamixel 팬틸트 장치를 보이게 배치하고, 데이터 흐름도를 함께 제시한다.

시연 순서는 다음과 같다.

1. Jetson 카메라 화면에서 드론 검출 확인
2. Ultra96 bridge `PING` 및 `src=pl` 응답 확인
3. 드론을 좌우/상하로 이동시키며 팬틸트 추적 확인
4. YOLO 검출이 끊기는 상황에서 오디오 fallback 방향 보조 설명
5. 로그를 통해 FPS, confidence, pan/tilt 응답 확인


# 설계 구성요소 및 현실적 제한요소 반영

## 1. 설계 구성요소

목표 설정: 드론을 영상과 음향으로 인식하고 팬틸트 장치가 실시간 추적하는 시스템을 목표로 설정하였다.

분석: Jetson-only 방식, PL 직접 UART 방식, PS/PL 분리 방식의 장단점을 비교하였다. 실험 로그를 통해 실제 지연의 주요 원인이 FPGA propagation delay가 아니라 카메라, YOLO, smoothing, serial write, 모터 물리 응답에 있음을 분석하였다.

합성: YOLO, 오디오 fallback, UDP 통신, AXI-Lite, PL goal compute, USB/U2D2, Dynamixel 모터 제어를 하나의 시스템으로 결합하였다.

제작: Jetson Python 소프트웨어, Ultra96 PS C bridge, Ultra96 PL Verilog RTL, 배포 스크립트, 데모 실행 스크립트를 제작하였다.

시험: RTL testbench, C syntax check, Python compile check, UDP `PING/PLPING/G/T` 단계 검증, 실시간 demo run으로 시험하였다.

평가: 드론 검출률, FPS, `src=pl` 응답 여부, 팬틸트 추적 안정성, 오디오 fallback 동작 여부를 기준으로 평가하였다.

## 2. 현실적 제한요소

산업표준: UDP/IP, AXI-Lite, USB serial, Dynamixel Protocol, TensorRT, Verilog HDL 등 실제 산업에서 사용되는 기술과 인터페이스를 적용하였다.

경제성: 고가의 전용 추적 장비 대신 보유한 Jetson, Ultra96, U2D2, Dynamixel 모터를 조합하여 제한된 비용 내에서 시스템을 구현하였다.

윤리성: 참고 코드와 팀원 코드의 역할을 구분하고, 팀원별 구현 결과를 통합하는 방식으로 진행하였다. 드론 추적 기능은 안전한 실내 시연 범위에서 사용하도록 제한하였다.

안정성: 모터 goal 제한, fallback mode, no-PL mode, PS direct mode, lazy PL open, laser auto-on 비활성 기본값 등을 적용하여 장치 손상과 위험 동작 가능성을 줄였다.

신뢰성: 장시간 실행 시 UDP 응답, U2D2 인식, USB Ethernet 유지, 모터 명령 응답을 로그로 확인하도록 구성하였다.

미학: 전시 시에는 하드웨어 배선이 보이되 데이터 흐름을 이해할 수 있도록 배치하고, 화면 UI는 bounding box와 중심 reticle 중심으로 단순하게 구성한다.


# < 참고 문헌 >

1. AMD Xilinx, Vivado Design Suite User Guide.
2. AMD Xilinx, AXI Reference Guide.
3. Avnet, Ultra96-V2 Hardware User Guide.
4. ROBOTIS, Dynamixel Protocol 2.0 e-Manual.
5. NVIDIA, Jetson Linux Developer Guide.
6. Ultralytics, YOLO Documentation.
7. OpenCV Documentation.
8. Python Socket Programming Documentation.
9. Verilog HDL 및 AXI-Lite 관련 강의 자료.
10. 한성대학교 전자트랙 전공 실험 및 종합설계 강의 자료.


# < 종합설계 프로젝트 수행 후기 >

본 프로젝트를 수행하면서 단순히 인공지능 모델을 실행하는 것만으로는 실제 추적 시스템을 완성할 수 없다는 것을 알게 되었다. 영상 인식 결과가 좋아도 통신 지연, 모터 응답, 제어 상수, 잘못된 검출값 처리, 장치 안정성이 함께 맞아야 실제 시연 가능한 결과물이 된다.

특히 Jetson, Ultra96 PS, Ultra96 PL, U2D2, Dynamixel이 모두 연결된 시스템에서는 문제가 발생했을 때 원인이 한 곳에만 있지 않았다. 따라서 전체 시스템을 작은 단계로 나누어 검증하고, 로그를 남기며, 안전한 fallback mode를 준비하는 것이 중요하다는 것을 배웠다.

이번 프로젝트를 통해 영상 인식, FPGA 설계, 임베디드 Linux, 네트워크 통신, 모터 제어를 통합하는 경험을 얻었다. 최종적으로는 하드웨어와 소프트웨어가 함께 동작하는 실시간 시스템을 직접 설계하고 개선하면서 종합설계의 목적에 맞는 실무형 문제 해결 능력을 키울 수 있었다.
