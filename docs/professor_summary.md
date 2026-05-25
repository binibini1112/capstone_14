# ultra_yubin 프로젝트 현황 요약

## 1. 주제

드론 기반 객체 추적 시스템에서 Jetson Nano의 영상/음성 인식 결과를 Ultra96 FPGA 보드로 넘기고, Ultra96의 PL/PS를 이용해 팬틸트 Dynamixel 모터를 제어하는 구조를 구현한다.

기존 Jetson 단독 제어 방식은 YOLO 실행 중 모터 명령 지연과 누락이 생길 수 있고, PL이 Dynamixel UART까지 직접 처리하는 방식은 추가 회로와 전기적 위험이 크다. 본 프로젝트는 두 방식의 단점을 줄이기 위해 PL은 추적 연산만 담당하고, 실제 모터 통신은 Ultra96 PS의 USB/U2D2가 담당하는 구조를 사용한다.

## 2. 최종 목표 구조

### YOLO 우선 경로

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

### 오디오 fallback 경로

```text
Jetson ReSpeaker/Tello audio angle
-> UDP 5016
-> Ultra96 PS angle-to-pan 변환
-> USB/U2D2
-> Dynamixel pan motor
```

YOLO bbox가 검출되면 항상 YOLO 경로가 1순위이고, YOLO가 미검출될 때만 오디오 방향 추적을 fallback으로 사용한다. 오디오 방향 계산은 RTL로 넣지 않고 PS에서 처리하도록 결정했다.

## 3. 현재 구현된 내용

- `ultra_yubin` 단일 프로젝트 폴더로 정리
- Jetson YOLO 송신 노드 구현
- ReSpeaker/Tello audio fallback 코드 포함
- Junmo/Junyoung Tello detector 모델 포함
  - `models/drone_best_final_0520.pt`
  - `models/drone_best_final_0520.engine`
- Tello audio 모델 포함
  - `models/tello_detector.tflite`
  - `models/tello_detector.keras`
  - `models/tello_audio_config.json`
- Ultra96 PS UDP/AXI/USB bridge 구현
- PL AXI-Lite RTL 구현
- Vivado 자동 빌드/전송 스크립트 구현
- Ultra96 legacy auto-start 서비스 제거
- GitHub 저장소 정리 및 업로드

## 4. 검증된 사항

- Vivado 2023.1 bitstream build 성공
- Ultra96-V2 board part 인식 성공
  - `avnet-tria:ultra96v2:part0:1.3`
- PL AXI 주소 매핑 확인
  - base: `0xA0000000`
  - range: `0x1000`
- Bitstream load 후 Ultra96 USB Ethernet/SSH 유지 확인
- PS bridge 실행 후 UDP `PING` 응답 확인
- PS bridge가 U2D2 `/dev/ttyUSB0`를 인식
- U2D2 goal write 경로에서 `usb=1` 응답 확인
- 오디오 fallback은 PS에서 angle-to-pan으로 처리되도록 구현

## 5. 최근 발견한 문제

PL AXI read는 응답하지만, PL write 명령을 보낼 때 Ultra96 USB Ethernet이 내려가는 문제가 있었다.

확인된 증상:

- `PLPING`: 응답함
- `G 2048 2772`: 이전 RTL에서는 Ultra96 USB/SSH가 끊김
- 이후 RTL 수정으로 한 차례 `G` 명령은 성공했으나, `pan/tilt` readback 값이 기대값과 다르게 `0`으로 읽히는 문제가 남음

현재 판단:

- Jetson 네트워크 문제나 SSH 설정 문제가 아니라 PL AXI-Lite slave의 write/read handshake 및 register readback 타이밍 문제로 보고 있다.
- 따라서 PL 경로를 포기한 것이 아니라, AXI-Lite RTL을 안정화하는 중이다.

## 6. 현재 수정 방향

현재 RTL은 다음 방향으로 수정 중이다.

- 브릿지 시작 시 바로 `/dev/mem`으로 PL을 건드리지 않도록 lazy-open 방식 적용
- `PING`은 PL 접근 없이 확인 가능
- `PLPING`, `G`, `T` 명령이 들어올 때만 PL register 접근
- AXI-Lite write response가 hang되지 않도록 handshake 구조 수정
- PL register 초기값을 명시적으로 설정
- `WSTRB` 문제 가능성을 줄이기 위해 32-bit register write는 우선 full-word write로 처리

이 수정 후 다시 Vivado bitstream을 빌드하고 다음 순서로 검증할 예정이다.

## 7. 다음 검증 순서

1. Windows Vivado에서 최신 RTL로 bitstream 재빌드
2. Jetson으로 bitstream 자동 전송
3. Ultra96에 bitstream만 load
4. SSH/USB Ethernet 유지 확인
5. PL-lazy bridge 실행
6. UDP `PING` 확인
7. `PLPING`으로 PL read 확인
8. `G 2048 2772`로 PL write 및 U2D2 write 확인
9. `PLPING`에서 `pan=2048`, `tilt=2772` readback 확인
10. `T bbox` 명령으로 PL bbox 계산 확인
11. Jetson YOLO 실시간 입력으로 전체 구조 확인

## 8. 남은 해결 과제

- PL AXI-Lite write/readback 안정화
- `T bbox` 입력에 대해 PL 계산값이 정상적으로 pan/tilt goal로 반영되는지 확인
- YOLO 실행 중 latency/jitter 측정
- Jetson-only 방식 대비 모터 명령 누락 감소 여부 측정
- 실제 카메라/드론 입력에서 YOLO 우선 추적과 오디오 fallback 전환 확인

## 9. 현재 결론

전체 시스템 방향은 확정되어 있고, Jetson과 Ultra96 PS/U2D2 경로는 대부분 동작 확인이 끝났다. 현재 핵심 이슈는 PL AXI-Lite register write/readback 안정화이며, 이 부분이 해결되면 최종 목표 구조인 `Jetson -> Ultra96 PS -> PL 계산 -> Ultra96 PS USB/U2D2 -> Dynamixel` 검증으로 넘어갈 수 있다.
