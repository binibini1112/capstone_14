# C Motor Logic Summary

작성 시점: 2026-05-25

이 문서는 laser C motor, 즉 Dynamixel ID 3으로 레이저 상하 보정을 하는 현재 런타임 로직과 관련 파일 상태를 정리한다.

## Current Default

현재 `run_demo_pl_drive.sh` 기본값은 다음 상태다.

```bash
LASER_CAMERA_CENTER_LOCK=0
LASER_BBOX_DIRECT_AIM=1
ULTRA_CHAN_LASER_ID=3
LASER_BBOX_TICK_MODEL_PATH=models/laser_bbox_tick_calibration.json
DISTANCE_MODEL_PATH=models/laser_distance_calibration.json
```

핵심은 `LASER_CAMERA_CENTER_LOCK=0`이다. 이제 C모터는 고정 center tick만 쓰지 않고, `models/laser_bbox_tick_calibration.json`에 있는 bbox height -> C motor base tick 테이블을 사용한다.

## Runtime Flow

1. `run_demo_pl_drive.sh`
   - 최종 데모용 환경변수를 잡는다.
   - Ultra96 PS bridge를 배포/재시작한다.
   - `run_demo.sh`를 통해 `jetson/jetson_node.py`를 실행한다.

2. `jetson/jetson_node.py`
   - YOLO bbox에서 `bw`, `bh`, `raw_cy`를 얻는다.
   - `DistanceEstimator`로 거리 추정값 `distance_mm`를 만든다.
   - `LaserTickEstimator`로 `bh -> laser_base_tick`을 보간한다.
   - `laser_goal_for_bbox()`는 Jetson 쪽 직접 `D 3 tick` fallback에 쓸 수 있는 최종 tick을 계산한다.
   - 정상 추적 `T` 경로에서는 최종 tick이 아니라 `laser_base_tick`을 Ultra96로 보낸다.

3. `jetson/src/control/ultra_yubin_motor.py`
   - `motor.control(...)`이 UDP `T` 명령을 만든다.
   - center-lock이 꺼져 있으면 마지막 인자로 `laser_base_tick`을 넣는다.
   - 예: `T cx cy bw bh fw fh conf valid dist laser_base`
   - `set_laser_tick()`은 별도 직접 명령이며 `D 3 <tick>`을 보낸다. 수동 보정, runtime calibration, 또는 `T`가 안 나간 경우의 direct aim fallback에서 사용된다.

4. `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c`
   - Ultra96 PS bridge가 `T`를 받는다.
   - pan/tilt goal은 PL 결과 또는 PS fallback으로 계산한다.
   - C motor는 다음 식으로 계산한다.

```text
laser_base = Jetson이 보낸 laser_base_override가 있으면 그 값
             없으면 bridge 내부 distance table로 계산

laser_img_ticks = bbox cy가 화면 중심에서 벗어난 정도를 vertical FOV로 tick 변환

laser_goal = clamp(laser_base + laser_img_ticks)
```

   - 이후 pan/tilt는 sync write, C motor는 ID 3에 `GOAL_POSITION` write를 한다.

## Active C Motor Calibration

현재 기본 실행에서 실제로 쓰는 C모터 base tick 테이블:

```text
models/laser_bbox_tick_calibration.json
```

내용 요약:

```text
type: bbox_height_to_laser_tick
samples: 5
distance anchors: 1.0m, 1.5m, 2.0m, 2.5m, 3.0m
C motor ID: 3
tick range in active samples: about 1985 to 2001
```

이 파일은 `LaserTickEstimator`가 읽는다. bbox height가 샘플 사이에 있으면 선형 보간하고, 샘플 범위 밖이면 가장 가까운 끝값을 쓴다.

## Distance Model Role

`models/laser_distance_calibration.json`도 로드된다.

하지만 `LASER_CAMERA_CENTER_LOCK=0`이고 `LaserTickEstimator`가 정상적으로 base tick을 만든 경우, C모터 최종 goal에는 거리 모델이 직접 들어가지 않는다. 이때 C모터 base는 `laser_bbox_tick_calibration.json`에서 온다.

거리 모델은 다음 경우에 의미가 있다.

- telemetry/log에 `distance_mm`를 남김
- `laser_base_tick`이 없어서 Ultra96 bridge가 내부 거리 table로 fallback할 때 사용
- `LASER_CAMERA_CENTER_LOCK=1`로 다시 켰을 때 center tick에 range offset을 더하는 데 사용

## Center Lock Path

이전 설정은 다음 경로였다.

```bash
LASER_CAMERA_CENTER_LOCK=1
LASER_CAMERA_CENTER_TICK=1965
LASER_CAMERA_CENTER_RANGE_COMP=1
LASER_CAMERA_CENTER_FAR_OFFSET_TICK=36
```

이 경우 `LaserTickEstimator`의 bbox-height table을 우회한다.

동작은 단순하다.

```text
active_center_tick = LASER_CAMERA_CENTER_TICK + range_offset
```

`range_offset`은 거리 1m -> 3m 사이를 기준으로 `0..36 tick`을 선형 보정한다. 즉 center-lock은 안정적이지만, 예전에 거리마다 맞춘 C tick table을 그대로 쓰는 방식은 아니다.

현재는 이 lock을 꺼두었으므로 기본 데모 경로에서는 사용하지 않는다.

## Ultra96 Bridge Embedded Table

`pl_udp_usb_dxl_bridge.c` 안에도 C모터 거리 table이 하드코딩되어 있다.

```text
250mm  -> 1920
500mm  -> 1952
750mm  -> 1978
1000mm -> 1985
1250mm -> 1992
1500mm -> 2000
1750mm -> 2000
2000mm -> 2000
2250mm -> 2002
2500mm -> 2002
2750mm -> 2006
3000mm -> 2006
```

이 table은 다음 상황에서 사용된다.

- Jetson이 `T` 명령에 `laser_base_override`를 0으로 보낸 경우
- `CENTER`, `G`, `A` 같은 PS bridge 직접 명령에서 C goal을 계산하는 경우

현재 정상 추적에서는 Jetson이 `laser_base_tick`을 보내므로, 이 embedded table은 주 경로가 아니라 fallback/직접명령 경로다.

## Direct Commands

Ultra96 bridge가 받는 C모터 관련 UDP 명령:

```text
D <id> <goal_tick>
DREL <id> <delta_tick>
T ... <distance_mm> <laser_base_override>
```

예:

```bash
python3 tools/dxl_id_probe.py --start 3 --end 3
```

최근 확인 결과:

```text
FOUND id=3 read=1 config=1 usb=1 present=1995 goal=1995
```

즉 ID 3은 응답 중이고, 현재 goal에 도달해 멈춘 상태였다.

## Files Used In Current Default Demo

`run_demo_pl_drive.sh`
: 데모 profile. 현재 `LASER_CAMERA_CENTER_LOCK=0`으로 바뀌어 있음.

`run_demo.sh`
: laser direct aim 주기, min delta, runtime calibration 기본값을 잡고 `run_jetson.sh` 실행.

`jetson/jetson_node.py`
: bbox에서 C motor base/goal 계산, telemetry, runtime calibration key handling.

`jetson/src/distance_model.py`
: `DistanceEstimator`, `LaserTickEstimator` 구현.

`jetson/src/control/ultra_yubin_motor.py`
: Jetson -> Ultra96 UDP 명령 생성. `T`와 `D 3 tick` 전송.

`jetson/src/config.py`
: C모터/레이저 관련 환경변수 기본값.

`hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c`
: Ultra96 PS UDP bridge. 실제 Dynamixel ID 3 write 수행.

`models/laser_bbox_tick_calibration.json`
: 현재 기본 C motor bbox-height base tick table.

`models/laser_distance_calibration.json`
: 거리 추정 table. 현재 lock-off 주 경로에서는 C base를 직접 만들지는 않지만 fallback과 telemetry에 필요.

## Files Not Used By Current Default Demo Path

아래 파일들은 삭제 대상이라는 뜻이 아니다. 현재 `./run_demo_pl_drive.sh` 기본 실행에서 직접 읽히지 않는다는 의미다.

`models/laser_tick_calibration.json`
: 수동 거리별 C tick 원본 샘플. `tools/laser_c_calibrate.py`가 만든 `distance_to_laser_tick` 데이터다. 현재 Jetson runtime은 이 JSON을 직접 읽지 않는다.

`models/laser_tick_table.json`
: 위 raw calibration을 압축한 runtime table. JSON 파일 자체는 현재 runtime에서 직접 읽지 않는다. 같은 값 계열이 Ultra96 bridge C 코드 안에 하드코딩되어 fallback으로 사용된다.

`models/laser_bbox_tick_calibration_pre_final_drive_cleanup_20260517_2130.json`
: cleaned active table을 만들기 전 백업. 샘플 45개. 현재 runtime은 직접 읽지 않는다.

`models/laser_bbox_tick_calibration_raw_backup_20260517_1555.json`
: 더 큰 raw backup. 샘플 337개. 현재 runtime은 직접 읽지 않는다.

`tools/laser_c_calibrate.py`
: 카메라 center reticle 기준으로 C모터를 수동 조정하고 `models/laser_tick_calibration.json`을 만드는 도구. 데모 runtime에서는 실행되지 않는다.

`tools/laser_drone_c_calibrate.py`
: 실제 드론 bbox를 보면서 C모터 tick 샘플을 수집하는 도구. 데모 runtime에서는 실행되지 않는다.

`tools/bbox_distance_calibrate.py`
: bbox size -> distance 샘플을 수집하는 도구. `models/laser_distance_calibration.json` 갱신용이다.

`docs/laser_c_motor_plan.md`
: 초기 설계 문서. 현재 구현 설명과 일부 차이가 있을 수 있다.

## Useful Knobs

```bash
# 현재 기본값. bbox-height calibration table 사용.
LASER_CAMERA_CENTER_LOCK=0

# 이전 안정형 center-lock 경로로 되돌릴 때.
LASER_CAMERA_CENTER_LOCK=1

# C모터 base tick table 교체.
LASER_BBOX_TICK_MODEL_PATH=/path/to/laser_bbox_tick_calibration.json

# C모터 직접 명령 fallback 주기/민감도.
LASER_BBOX_AIM_UPDATE_PERIOD_SEC=0.12
LASER_BBOX_AIM_MIN_DELTA_TICK=4

# bbox cy 기반 image vertical correction에 쓰는 FOV.
ULTRA_CHAN_LASER_VERTICAL_FOV_DEG=43

# C motor ID.
ULTRA_CHAN_LASER_ID=3
```

## Debug Checklist

1. ID 3 응답 확인

```bash
python3 tools/dxl_id_probe.py --start 3 --end 3
```

2. 데모 시작 로그 확인

```text
laser_center_lock=0:1965
```

3. Ultra96 reply 확인

```text
T,...,laser_base=<base>,laser=<final>,laser_img=<offset>,laser_id=3,usb=1
```

4. C모터가 안 움직여 보일 때 확인할 것

```text
laser_base와 laser가 거의 같으면 target cy가 화면 중심에 가깝다는 뜻이다.
present == goal이면 모터는 이미 목표 위치에 도달한 상태다.
tick 변화가 1985~2005 근처로 작으면 눈으로는 거의 안 움직여 보일 수 있다.
```
