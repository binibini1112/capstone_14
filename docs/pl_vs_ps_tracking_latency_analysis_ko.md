# PL vs PS Tracking Latency Analysis

Date: 2026-05-16

Scope:
- Main code under test: `/home/jetson/ultra_yubin_v1`
- Read-only reference: `/home/jetson/ultra_chan`
- Do not edit teammate folders when applying this plan.

## Short Conclusion

The current lag is not mainly an FPGA propagation-delay limit.

The PL computation itself is effectively sub-millisecond. The visible delay is much more likely from:

1. camera frame age and YOLO inference,
2. Jetson-side target smoothing, jump hold, deadband, and control-rate skipping,
3. synchronous UDP/reply plus U2D2 serial write time,
4. Dynamixel profile velocity/acceleration and mechanical inertia.

So the answer is:

- PL path overhead: real, but probably only around 1-3 ms in this implementation.
- Total visual-to-motion delay: commonly 40-90 ms, sometimes more.
- Main fix direction: keep PL for capstone defense, but make PL compute the Chanhee-style controller and reduce Jetson-side filtering/scheduling latency.

## Call Chain Comparison

### Yubin v1 Current Intended Path

Path:

```text
Camera frame
-> YOLO detector
-> target bbox center selection
-> UltraYubinMotorController.control()
-> UDP "T cx cy bw bh fw fh conf valid distance laser_base"
-> Ultra96 PS bridge
-> AXI writes to PL goal-compute IP
-> PL computes next pan/tilt goal
-> PS reads PL goal
-> PS writes Dynamixel goal over U2D2
-> pan/tilt motors move
```

Important code references:

- Jetson sends the UDP control packet in `jetson/src/control/ultra_yubin_motor.py:317`.
- Synchronous request path sends UDP and waits for reply in `jetson/src/control/ultra_yubin_motor.py:86`.
- Rate skip happens before sending in `jetson/src/control/ultra_yubin_motor.py:273`.
- Jetson-side smoothing happens in `jetson/src/control/ultra_yubin_motor.py:297`.
- Ultra96 bridge receives and drains to newest UDP packet in `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:931`.
- The `T ...` target command is parsed in `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1205`.
- PL branch starts at `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1225`.
- PS direct branch starts at `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1216`.
- PL register writes are done by `pl_cmd_set_track()` in `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:265`.
- U2D2 sync write to pan/tilt is done in `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:725`.

### Chanhee Reference Path

Practical behavior:

```text
Camera frame
-> YOLO detector / tracking loop
-> UltraChanMotorController.control()
-> UDP "T cx cy bw bh fw fh conf valid"
-> Ultra96 PS bridge
-> PS computes pan/tilt directly by ps_track_step()
-> PS writes Dynamixel goal over U2D2
-> pan/tilt motors move
```

Important code references:

- Chanhee motor controller sends `T ...` in `src/control/ultra_chan.py:248`.
- It uses a synchronous UDP request/reply in `src/control/ultra_chan.py:96`.
- It skips commands by deadband/rate at `src/control/ultra_chan.py:198` and `src/control/ultra_chan.py:217`.
- The bridge default is `track_direct_ps = 1` in `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:736`.
- The PS direct tracking law is `ps_track_step()` in `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:175`.

Key difference:

- Chanhee's bridge defaults to PS direct tracking.
- Yubin v1 defaults to PL tracking in source (`track_direct_ps = 0`) at `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:760`.
- If live logs show `src=ps_direct`, the deployed bridge is still PS direct. For final PL defense, target replies must show `src=pl`.

## Where PL Is Used In Yubin v1

In the Ultra96 PS bridge, command `T` chooses one of two branches:

```c
if (no_pl || track_direct_ps) {
    ps_track_step(...);
    goal_src = no_pl ? "ps" : "ps_direct";
} else {
    pl_cmd_set_goal(...);
    pl_cmd_set_track(...);
    usleep(1000);
    pan_now = pl_read_pan_goal(...);
    tilt_now = pl_read_tilt_goal(...);
}
```

References:

- Branch condition: `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1216`
- PL write/read branch: `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1225`
- Explicit 1 ms wait after PL trigger: `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1245`
- Reply includes `src=%s`: `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1274`

The RTL currently implements Chanhee-style control constants:

- deadband 14 px: `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v:49`
- max step 72 tick, close max step 32 tick: `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v:51`
- pixel/8 gain: `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v:165`
- pan/tilt goal update: `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v:348`

Important: RTL source changes only affect hardware after a new Vivado bitstream is built and deployed.

## Estimated Latency By Stage

These are practical estimates from the code and normal Jetson/U2D2 behavior, not oscilloscope measurements.

| Stage | Estimate | Reason |
|---|---:|---|
| Camera frame age | 8-33 ms | 60 fps gives up to 16.7 ms; buffering/exposure can add more |
| YOLO TensorRT inference | 20-40 ms | Your logs show total avg around 24-28 fps; old Chanhee logs show inference around 23-28 ms |
| Jetson target filtering/smoothing | 0-2 frames | `TARGET_CENTER_SMOOTHING_*`, max step, jump hold can intentionally lag |
| Jetson control period | up to 18-20 ms | `ULTRA_CHAN_CONTROL_PERIOD_SEC` default is 0.018-0.020 |
| UDP local Ethernet round trip | usually <1-3 ms | Same local link; not dominant |
| PS direct compute | <0.1 ms | integer arithmetic in C |
| PS -> PL MMIO + readback | about 1-2 ms | code has six AXI writes plus explicit `usleep(1000)` |
| U2D2 packet write | about 4-8 ms at 57600 baud | sync-write packet is about 29 bytes, plus `tcdrain()` |
| Dynamixel movement response | variable, often 20+ ms | profile velocity/accel and load dominate physical catch-up |

Expected extra delay of Yubin PL path over Chanhee PS-direct path:

- Mostly `usleep(1000)` plus MMIO writes/readbacks.
- Rough estimate: 1-3 ms.
- This is much smaller than camera + inference + motor movement delay.

## Diagnosis

### (a) Communication path / PL propagation delay

Classification: minor contributor.

The PL computation itself is not the visible lag source. The RTL update occurs in one AXI clocked write cycle after `TRACK_CMD` is accepted. The bridge adds a deliberate `usleep(1000)` before reading back PL goals at `pl_udp_usb_dxl_bridge.c:1245`, so the PL path costs around 1 ms plus MMIO overhead. That is too small to explain the large "following behind" feel by itself.

However, if the live bridge is accidentally running `src=ps_direct`, then PL is not being used for target control. For capstone defense, logs must show `src=pl` on `T,1,...` target replies.

### (b) Tracking logic / control algorithm

Classification: major contributor.

Lag can be introduced by several deliberate filters:

- Jetson deadband skip at `ultra_yubin_motor.py:250`.
- Jetson rate skip at `ultra_yubin_motor.py:273`.
- Jetson coordinate smoothing at `ultra_yubin_motor.py:297`.
- Additional target smoothing, max-center-step, jump-hold, and motor-step limiting in `jetson/jetson_node.py:551` through `jetson/jetson_node.py:646`.
- PL/PS tracking law uses proportional-only pixel error divided by 8 and clamped at 72 or 32 ticks, as shown in `pl_udp_usb_dxl_bridge.c:203` and RTL `pl_goal_compute_axi.v:176`.

This means the system is stable and not too twitchy, but it can trail a fast target. This is the main thing to tune.

### (c) Loop rate / scheduling / buffering

Classification: major contributor.

Yubin v1 currently runs detection inline in the main loop:

- frame read: `jetson/jetson_node.py:468`
- detector call: `jetson/jetson_node.py:475`
- motor control call: `jetson/jetson_node.py:686`

Chanhee's main path has an asynchronous YOLO worker that keeps the newest frame/result separate:

- worker class begins at `ultra_chan/main.py:94`
- main loop submits newest frame at `ultra_chan/main.py:780`
- latest result is consumed at `ultra_chan/main.py:781`
- direct follow/send happens around `ultra_chan/main.py:958` and `ultra_chan/main.py:972`

So Chanhee may feel better not only because of PS direct control, but because it avoids blocking the display/control loop on every inference step.

## Improvements Without Changing Structure

Keep PL, but reduce software-induced lag:

1. Use the newest frame only.
   - Keep camera buffer at 1.
   - Drain stale UDP packets is already done in bridge at `pl_udp_usb_dxl_bridge.c:943`.
   - The same newest-only idea should be used for camera and inference.

2. Reduce smoothing only on the motor-control path.
   - Test `TARGET_CENTER_SMOOTHING_NEAR=0.85`, `TARGET_CENTER_SMOOTHING_FAR=1.0`.
   - Test `ULTRA_CHAN_SMOOTH_ALPHA_X=1.0`, `ULTRA_CHAN_SMOOTH_ALPHA_Y=1.0`.
   - Keep UI smoothing separate if the display becomes jittery.

3. Lower command period carefully.
   - Current demo default is 18 ms in `run_demo.sh:55`.
   - Test 10-12 ms only if U2D2 does not saturate.
   - If commands queue up or motor buzzes, return to 18 ms.

4. Tune PL P gain / clamp.
   - Current gain is pixel/8, max 72 ticks.
   - More aggressive test: pixel/6 or pixel/5, max 90-110 ticks.
   - If oscillation appears near center, keep deadband 10-14 px and reduce close max step.

5. Add lead compensation.
   - Estimate bbox velocity from previous centers.
   - Send `cx + vx * lead_frames`, `cy + vy * lead_frames` to PL.
   - Start with 1.0 frame lead.
   - Clamp lead to about 30-80 px so false detections do not kick the gimbal.

6. Dynamixel profile tuning.
   - Current bridge defaults profile accel=60, velocity=180 in `pl_udp_usb_dxl_bridge.c:761`.
   - For faster response, test velocity 250-350 and accel 100-180.
   - This can reduce physical lag more than PL/PS path changes.

## Structural Improvement Options

### Option A: Keep PL And Match Chanhee Feel

This is the recommended capstone path.

Use:

```text
Jetson bbox -> Ultra96 PS UDP receiver -> PL computes goal -> PS U2D2 writer
```

But make PL control law match the best PS-direct controller:

- deadband 14 px
- gain pixel/8 initially
- clamp 72 tick, close clamp 32 tick
- then tune gain/clamp upward only after confirming no oscillation

This is already represented in the v1 RTL source. It still needs a new bitstream build/deploy to be active on hardware.

### Option B: PS Direct Fallback For Debug Only

Use `--track-direct-ps` or deploy with direct-PS only to compare behavior. This is useful for debugging, but it weakens the "why Ultra96/FPGA?" defense because the target-control computation bypasses PL.

### Option C: Split Control Responsibility

If PL readback overhead or AXI sequencing becomes annoying, PL can output correction deltas rather than absolute goals:

```text
PL: bbox error -> pan_delta/tilt_delta
PS: clamp around front, send U2D2
```

This still uses PL for the control algorithm while letting PS handle safety and motor IO.

## Recommended Test Plan

1. Build/deploy new bitstream and force PL mode.

```bash
cd /home/jetson/ultra_yubin_v1
ULTRA_YUBIN_V1_TRACK_DIRECT_PS=0 ULTRA_YUBIN_V1_RESTART=1 \
ULTRA_YUBIN_V1_SKIP_PL_LOAD=0 ULTRA_YUBIN_V1_SKIP_PL_INIT=0 \
ULTRA_YUBIN_V1_SKIP_DXL_INIT=0 ULTRA_YUBIN_V1_SKIP_CHECK=1 \
./tools/deploy_ultra96_ps_usb.sh
```

2. Run and confirm target replies use PL.

```bash
cd /home/jetson/ultra_yubin_v1
./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```

Required evidence:

```text
reply='T,...src=pl'
```

If it says `src=ps_direct`, the PL path is not being used for target tracking.

3. Latency-tuning test command.

```bash
cd /home/jetson/ultra_yubin_v1
ULTRA_CHAN_SMOOTH_ALPHA_X=1.0 ULTRA_CHAN_SMOOTH_ALPHA_Y=1.0 \
TARGET_CENTER_SMOOTHING_NEAR=0.90 TARGET_CENTER_SMOOTHING_FAR=1.0 \
TARGET_MOTOR_MAX_STEP_PX=500 TARGET_MAX_CENTER_STEP_PX=500 \
ULTRA_CHAN_CONTROL_PERIOD_SEC=0.012 \
./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```

If tracking improves but jitters, back off in this order:

1. `ULTRA_CHAN_CONTROL_PERIOD_SEC=0.018`
2. `ULTRA_CHAN_SMOOTH_ALPHA_X=0.90`, `ULTRA_CHAN_SMOOTH_ALPHA_Y=0.88`
3. lower PL max correction or increase deadband.

## Action Checklist

- [ ] Rebuild Vivado bitstream after RTL changes.
- [ ] Deploy with `ULTRA_YUBIN_V1_TRACK_DIRECT_PS=0`.
- [ ] Confirm runtime target replies show `src=pl`.
- [ ] Run one baseline log with current defaults and save avg fps / `motor_ms` / `src` distribution.
- [ ] Test no/low smoothing command above.
- [ ] If lag remains, add 1-frame bbox velocity lead before `motor.control()`.
- [ ] If physical motion is still slow, increase Dynamixel profile velocity/accel and retest.
- [ ] Only use `src=ps_direct` as a comparison baseline, not the final capstone path.
