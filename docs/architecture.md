# ultra_yubin 최종 후보 구조

## 데이터 흐름

`Jetson YOLO bbox -> UDP 5016 -> Ultra96 PS bridge -> AXI 0xA0000000 -> Ultra96 PL -> AXI readback -> Ultra96 PS USB/U2D2 -> Dynamixel`

`Jetson ReSpeaker audio fallback -> UDP 5016 -> Ultra96 PS angle-to-pan -> Ultra96 PS USB/U2D2 -> Dynamixel`

## 역할

- Jetson은 무거운 영상 추론과 ReSpeaker/Tello audio 판단을 담당한다.
- Ultra96 PL은 bbox 중심 오차로 pan/tilt goal을 계산한다.
- Ultra96 PS는 UDP/AXI/USB 직렬 통신과 audio fallback angle-to-pan 변환을 담당한다.

## 채택 기준

- Jetson-only 대비 모터 명령 지연과 지터가 줄어야 한다.
- YOLO 실행 중 모터 명령 누락이 줄어야 한다.
- Ultra96 PS의 USB/U2D2 송신이 장시간 유지되어야 한다.
- 실제 드론 카메라 입력에서 실시간 추적이 유지되어야 한다.
