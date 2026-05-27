# 레이저 모터 로직 요약

작성 기준: 현재 최종 데모 명령

```bash
./run_demo_pl_drive.sh
```

이 문서는 Dynamixel ID 3으로 구성된 레이저 모터의 현재 런타임 로직을 정리한다.

## 현재 기본값

`scripts/run_demo_pl_drive.sh` 기준 레이저 모터 관련 핵심 설정은 다음과 같다.

```bash
LASER_CAMERA_CENTER_LOCK=1
LASER_CAMERA_CENTER_TICK=1945
LASER_CAMERA_CENTER_RANGE_COMP=1
LASER_CAMERA_CENTER_RANGE_COMP_USE_DISTANCE=1
LASER_CAMERA_CENTER_NEAR_DISTANCE_MM=1000
LASER_CAMERA_CENTER_FAR_DISTANCE_MM=3000
LASER_CAMERA_CENTER_FAR_OFFSET_TICK=36
LASER_SPOT_CLOSED_LOOP=0
ULTRA_CHAN_LASER_ID=3
```

즉 현재 데모는 레이저 점을 영상에서 계속 추적해 닫힌루프로 보정하는 방식이 아니라,
현장 보정된 기준 tick과 거리 보정값을 사용해 레이저 모터 목표값을 정한다.

## 런타임 흐름

1. `scripts/run_demo_pl_drive.sh`
   - 최종 데모 환경변수를 설정한다.
   - Ultra96 PS bridge를 배포/재시작한다.
   - `scripts/run_demo.sh`를 통해 `jetson/jetson_node.py`를 실행한다.

2. `jetson/jetson_node.py`
   - YOLO11n bbox에서 중심 좌표와 bbox 크기를 얻는다.
   - bbox 크기 기반 거리 추정값 `distance_mm`를 만든다.
   - `LASER_CAMERA_CENTER_TICK=1945`를 기준 레이저 tick으로 사용한다.
   - 거리 보정이 켜져 있으면 1m~3m 범위에서 `0..36 tick` 범위의 offset을 더한다.
   - 최종 레이저 기준 tick을 Ultra96 `T` 명령의 `laser_base` 값으로 전달한다.

3. `jetson/src/control/ultra_yubin_motor.py`
   - Jetson에서 Ultra96-V2 PS bridge로 UDP `T` 명령을 전송한다.
   - 명령에는 bbox 중심, bbox 크기, frame 크기, confidence, 거리, 레이저 기준 tick이 포함된다.

4. `ultra96/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c`
   - Ultra96 PS bridge가 `T` 명령을 수신한다.
   - pan/tilt 목표값은 PL goal compute 결과를 사용한다.
   - 레이저 모터 목표값은 Jetson이 보낸 `laser_base`와 bbox 중심 오차 기반 image offset을 이용해 계산한다.
   - 최종 goal position을 Dynamixel ID 3 레이저 모터로 전송한다.

## HIT 판정 후 복귀

현재 데모는 `F` 키로 레이저 HIT 판정을 수행한다.

HIT가 확인되면 즉시 정면으로 돌아가지 않고:

1. 약 1초간 현재 방향을 유지한다.
2. 이후 Ultra96 `CENTER` 명령을 보내 pan/tilt를 정면 기준으로 복귀시킨다.
3. 복귀 직후 짧은 시간 동안 비전/오디오 명령이 다시 끌고 가지 않도록 hold/blind를 적용한다.

관련 기본값:

```bash
FIRE_RETURN_CENTER_ON_HIT=1
FIRE_RETURN_CENTER_DELAY_SEC=1.0
FIRE_RETURN_CENTER_HOLD_SEC=1.0
FIRE_RETURN_CENTER_AUDIO_BLIND_SEC=1.0
```

## 관련 파일

```text
scripts/run_demo_pl_drive.sh
jetson/jetson_node.py
jetson/src/config.py
jetson/src/distance_model.py
jetson/src/control/ultra_yubin_motor.py
ultra96/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c
```

## 디버그 체크리스트

1. 데모 시작 로그에서 레이저 설정 확인

```text
laser_center_lock=1:1945
range_comp=1:distance:36
```

2. Ultra96 응답에서 레이저 모터 필드 확인

```text
laser_base=<base>,laser=<final>,laser_img=<offset>,laser_id=3,usb=1
```

3. 레이저 모터가 움직이지 않아 보일 때

```text
laser_base와 laser가 거의 같으면 bbox 중심이 화면 중심에 가깝다는 뜻이다.
present == goal이면 모터는 이미 목표 위치에 도달한 상태다.
tick 변화가 작으면 눈으로는 움직임이 거의 안 보일 수 있다.
```
