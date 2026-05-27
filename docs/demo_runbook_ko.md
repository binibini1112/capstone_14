# 최종 데모 실행 가이드

## Jetson 실행

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo_pl_drive.sh
```

기본 실행은 다음 기능을 포함합니다.

- YOLO11n TensorRT 기반 드론 검출
- 현재 기본 비전 엔진: `drone0525jh.engine`
- Ultra96-V2 PS/PL 기반 팬틸트 제어
- ReSpeaker 오디오 fallback
- CNN6 방향 모델 기반 소리 방향 회전
- C모터 레이저 tick 보정
- Raspberry Pi dashboard telemetry 송신

## 오디오 fallback 상태

현재 데모 기본값은 안정형입니다.

```text
TELLO_AUDIO_THRESHOLD=0.95
TELLO_AUDIO_MIN_AVG_SCORE=0.85
TELLO_AUDIO_MIN_RMS=0.0020
TELLO_AUDIO_DIRECTION_MODE=cnn6
TELLO_AUDIO_SKIP_GCC_WITH_CNN6=1
TELLO_AUDIO_ALSA_DEVICE=auto
TELLO_AUDIO_STABLE_RESET_GAP_SEC=3.0
```

`TELLO_AUDIO_ALSA_DEVICE=auto`는 USB 카드 번호가 바뀌어도 ReSpeaker를 자동으로 찾기 위한 설정입니다.

오디오 fallback은 3개 판단 창에서 2표 이상 같은 방향이면 팬을 돌립니다. 팬 회전 명령이 실제로 나간 직후에는 오디오 방향 투표 버퍼를 초기화하며, 감지 사이 공백이 3초를 넘으면 남아 있던 단독 표를 버리고 새 판단으로 시작합니다.

## C모터 레이저 상태

현재 데모 기본값은 closed-loop 보정을 끈 안정형입니다.

```text
LASER_CAMERA_CENTER_TICK=1945
LASER_CAMERA_CENTER_RANGE_COMP=1
LASER_SPOT_CLOSED_LOOP=0
```

즉 레이저 점을 화면에서 검출해 계속 따라가는 방식이 아니라, 캘리브레이션 기반 기준 tick과 거리 보정을 이용합니다.

## Ultra96 정면 기준

Ultra96 PS bridge의 정면 기준은 `/home/xilinx/ultra_yubin_v1/front_center.env`에 저장됩니다.

예:

```text
PAN=2048
TILT=2772
```

현장 조정은 다음 스크립트로 수행할 수 있습니다.

```bash
python3 scripts/ultra_yubin_calibrate_front.py --set 2048 2772 --center
```

레이저 HIT 판정이 발생하면 데모 연출을 위해 약 1초간 현재 방향을 유지한 뒤 `CENTER` 명령으로 정면 기준으로 복귀합니다.

## Raspberry Pi Dashboard

```bash
cd ~/jh
./run_dashboard.sh
```

## Tello Scenario

```bash
cd ~/jh
./run_drone.sh
```

프로그램에서 `ARM`을 입력한 뒤 시나리오 키를 선택합니다.
