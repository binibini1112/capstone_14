# ultra_yubin_v1 Tracking Tuning Status

Last updated: 2026-05-17

## 2026-05-17 Demo Candidate Summary

Current best stable command:

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo_pl_drive.sh --pipeline-echo --pipeline-echo-every 30
```

Use this as the rollback baseline. It drives tracking through the Ultra96 PL
path (`src=pl`) and keeps the PS-best control law mirrored in RTL. This is the
best currently verified presentation path because it gives the FPGA/PL a real
role: bbox error to pan/tilt goal computation.

Reference commit:

```text
f3f4a72 pl: mirror ps best tracking profile
```

Jetson-side experimental improvement:

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo_pl_drive_lead1.sh --pipeline-echo --pipeline-echo-every 30
```

This keeps the same PL-drive motor path, but Jetson sends a small predicted
bbox center based on bbox velocity:

- `TRACK_LEAD_FRAMES=1.0`
- `TRACK_LEAD_MAX_PX=70`
- `TRACK_MOTOR_MIN_CONF=0.60`
- `ULTRA_CHAN_CONTROL_PERIOD_SEC=0.007`
- `ULTRA_YUBIN_V1_PROFILE_ACCEL=170`
- `ULTRA_YUBIN_V1_PROFILE_VELOCITY=370`

Observed test note from 2026-05-17 16:51 KST:

- Command: `./run_demo_pl_drive_lead1.sh --pipeline-echo --pipeline-echo-every 30`
- Average FPS: `36.9`
- User visual impression: acceptable / promising
- Risk: if YOLO bbox jumps, lead compensation can over-lead or aim at a false
  motion direction. If that happens, immediately roll back to the stable
  `run_demo_pl_drive.sh` command above.

## Current Goal

Keep the drone as close as possible to the camera center during the demo.
Perfect zero-error centering is physically impossible because the loop includes
camera capture, YOLO inference, Jetson-to-Ultra96 UDP, PL calculation, PS USB
commanding, and Dynamixel motion. The practical target is fast center recovery
without losing the drone or overshooting off-screen.

## Latest Observation

The fully raw Jetson-to-PL test improved latency, but it made motor motion too
aggressive:

- PL path was confirmed by `src=pl`.
- FPS was good, around 25-28 FPS.
- Detection improved when `YOLO_CONF` was reduced from 0.70 to 0.60.
- Removing all Jetson-side filtering allowed unstable boxes such as `cx=106`
  or `cy=706` to drive the motors, causing pan/tilt jumps.

Conclusion: Jetson should not smooth heavily, but it must still reject edge and
large-jump boxes. PL should handle the actual control law.

## Current Run Profile

`run_demo.sh` is now tuned for performance:

- YOLO confidence default: `0.60`
- Motor confidence default: `0.60`
- Camera center reticle is always visible.
- Laser auto-on is disabled by default to avoid flicker during tracking tests.
- Jetson-side center lock and deadband are disabled.
- Heavy Jetson smoothing and step limits are disabled.
- A small input smoothing remains:
  - X alpha: `0.92`
  - Y alpha: `0.84`
- Edge boxes are filtered:
  - X margin: `28 px`
  - Y margin: `36 px`
- Large target jumps are rejected:
  - jump hold: `420 px`
  - confirm: `140 px`

Run:

```bash
cd /home/jetson/ultra_yubin_v1 && ./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```

For pipeline JSON logging:

```bash
cd /home/jetson/ultra_yubin_v1 && RUN_DEMO_PIPELINE_LOG=1 ./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```

## RTL Changes Pending Bitstream Build

`hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v` now separates X/Y motor
correction limits:

- `TRACK_DEADBAND_X = 10`
- `TRACK_DEADBAND_Y = 10`
- `TRACK_MAX_CORRECTION_X = 64`
- `TRACK_MAX_CORRECTION_Y = 44`
- `TRACK_MAX_CORRECTION_CLOSE_X = 28`
- `TRACK_MAX_CORRECTION_CLOSE_Y = 22`

Reason:

- Pan needs enough speed to follow lateral drone motion.
- Tilt was overreacting and hitting high/low positions too often.
- Close targets should move more gently because bbox noise is larger in pixels.

Verification passed:

```bash
iverilog -g2012 -o /tmp/ultra_yubin_v1_pl_goal_tb hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v hardware/pl_goal_compute/tb/pl_goal_compute_axi_tb.v && vvp /tmp/ultra_yubin_v1_pl_goal_tb
gcc -Wall -Wextra -fsyntax-only hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c
python3 -m py_compile jetson/jetson_node.py jetson/src/config.py jetson/src/ui/display.py jetson/src/control/ultra_yubin_motor.py
bash -n run_demo.sh
```

## Next Required Steps

1. Build the new bitstream on Windows:

```powershell
cd C:\Users\hansung\examples\ultra_yubin_v1
.\build_and_send.ps1
```

2. Deploy on Jetson:

```bash
cd /home/jetson/ultra_yubin_v1 && ULTRA_YUBIN_V1_RESTART=1 ./tools/deploy_ultra96_ps_usb.sh
```

3. Run demo test:

```bash
cd /home/jetson/ultra_yubin_v1 && ./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```

Check:

- `reply='T,...src=pl'`
- Drone stays near the fixed center reticle.
- Pan does not swing across the whole range after one bad box.
- Tilt does not repeatedly hit `3012` or a limit.
- Detection ratio is better than the previous `660/1334`.

## If It Still Feels Wrong

If too slow:

- Increase `TRACK_MAX_CORRECTION_X` to `72`.
- Increase `TRACK_MAX_CORRECTION_Y` to `52`.

If too shaky:

- Decrease `TRACK_MAX_CORRECTION_X` to `56`.
- Decrease `TRACK_MAX_CORRECTION_Y` to `36`.
- Increase `TRACK_DEADBAND_X/Y` to `14`.

If detection keeps dropping:

- Test `YOLO_CONF=0.55 TRACK_TARGET_MIN_CONF=0.55 TRACK_MOTOR_MIN_CONF=0.58`.
- Do not lower motor confidence too far unless false positives are acceptable.

## Audio-Only ReSpeaker Test Note - 2026-05-17 14:18 KST

Test condition:

- Script baseline: `./run_demo_ps_fast2_audio.sh`
- Camera intentionally covered.
- ReSpeaker audio fallback only.
- Several songs/noise sources were played at the same time.
- Drone sound direction search still worked.
- Verified distance so far: up to `2 m` only.

Important limitation:

- This result does not yet prove audio-only search beyond `2 m`.
- Next test should repeat at `2.5 m` and `3 m`, then confirm vision reacquires after the pan turns toward the sound.
