# Tracking Change Risk Review

Date: 2026-05-16

Scope:
- Main code: `/home/jetson/ultra_yubin_v1`
- Reference only: `/home/jetson/ultra_chan`
- Do not edit teammate/reference folders.

## Baseline

The previous latency analysis concluded that the FPGA/PL propagation delay is not the main visible lag source. The current larger contributors are camera frame age, YOLO inference, Jetson-side filtering/rate limiting, U2D2/Dynamixel write time, and physical motor response.

Important code points:

| Area | Code reference | Why it matters |
|---|---|---|
| Main frame/inference loop | `jetson/jetson_node.py:468`, `jetson/jetson_node.py:475` | Frame read and YOLO run inline in the main loop. |
| Target center shaping | `jetson/jetson_node.py:551`, `jetson/jetson_node.py:572`, `jetson/jetson_node.py:582`, `jetson/jetson_node.py:629` | Limits and smoothing can intentionally delay the command. |
| Motor command call | `jetson/jetson_node.py:686` | Final point before sending target to motor controller. |
| Motor rate skip | `jetson/src/control/ultra_yubin_motor.py:273` | `ULTRA_CHAN_CONTROL_PERIOD_SEC` can skip sends. |
| Motor smoothing | `jetson/src/control/ultra_yubin_motor.py:297` | `ULTRA_CHAN_SMOOTH_ALPHA_X/Y` changes command smoothing. |
| UDP target packet | `jetson/src/control/ultra_yubin_motor.py:317` | Sends `T cx cy ...` to Ultra96 bridge. |
| PL/PS branch | `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1216` | Chooses PS direct or PL path. |
| PL branch | `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1225` | Target command goes through PL here. |
| PL wait | `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1245` | Adds explicit 1 ms wait before PL readback. |
| U2D2 write | `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1270` | Final Dynamixel goal write. |
| PL gain/clamp constants | `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v:49`, `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v:51`, `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v:165` | Deadband, max step, and pixel/8 gain. |
| Bridge matching constants | `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:44`, `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:45` | PS fallback/direct must match PL behavior. |
| Dynamixel profile defaults | `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:761`, `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:762` | Current accel=60, velocity=180. |
| PL mode default | `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:760` | Source default is `track_direct_ps = 0`, but live logs must confirm `src=pl`. |
| Chanhee async reference | `/home/jetson/ultra_chan/main.py:94`, `/home/jetson/ultra_chan/main.py:780`, `/home/jetson/ultra_chan/main.py:781` | Reference structure uses latest-frame/latest-result worker style. |

Risk scale:
- Low: easy runtime rollback, low hardware risk.
- Medium: can cause bad tracking but rollback is simple.
- High: can cause violent motion, unstable demo, rebuild risk, or hard-to-debug concurrency problems.

## Summary Table

| Change | Expected benefit | Risk grade | D-day safety |
|---|---|---|---|
| 1. Weaken/remove smoothing and step filters | Less visual lag | Medium | Safe until D-1 if tested as env-only change |
| 2. Shorten control period 0.018 -> 0.012 | Faster command updates | Medium | Safe until D-2; avoid 0.008 near demo |
| 3. Raise PL P gain and max step | Stronger correction | High | Finish by D-4 or earlier |
| 4. Increase Dynamixel profile velocity/accel | Faster physical motor response | Medium-High | Mild values by D-2; aggressive values by D-4 |
| 5. Add lead/prediction compensation | Compensates camera/YOLO delay | Medium-High | Finish by D-3; use env kill switch |
| 6. Add async YOLO worker | Reduces pipeline blocking | High | Finish by D-5 or earlier |
| 7. Rebuild bitstream and force PL mode | Required for PL defense | Medium deploy risk, high if late | Finish by D-3, ideally D-5 |

## 1. Smoothing / Filter Weakening Or Removal

Proposed:

```bash
ULTRA_CHAN_SMOOTH_ALPHA_X=1.0
ULTRA_CHAN_SMOOTH_ALPHA_Y=1.0
TARGET_CENTER_SMOOTHING_NEAR=0.90
TARGET_CENTER_SMOOTHING_FAR=1.0
TARGET_MOTOR_MAX_STEP_PX=500
TARGET_MAX_CENTER_STEP_PX=500
```

Risk grade: Medium.

### Functional Risks

| Risk | Why it can happen | Symptom |
|---|---|---|
| Jitter near center | Jetson motor smoothing at `jetson/src/control/ultra_yubin_motor.py:297` is reduced. Target smoothing in `jetson/jetson_node.py:582` is also reduced. | Camera shakes even when drone is almost centered. |
| Overshoot/hunting | Bigger center jumps pass through because `TARGET_MAX_CENTER_STEP_PX` and `TARGET_MOTOR_MAX_STEP_PX` are relaxed at `jetson/jetson_node.py:572` and `jetson/jetson_node.py:629`. | Target crosses center, gimbal corrects back, repeats. |
| False positive jump | Filters currently soften sudden bbox jumps. Removing them makes one wrong bbox immediately move the motors. | Camera suddenly points at wall/light/noise. |
| Worse lock stability | Small bbox wobble becomes real motor movement. | Drone is inside bbox, but frame keeps twitching. |

No-target behavior is mostly safe because no-target resets smoothing/history around `jetson/jetson_node.py:699`. But when detections flicker on/off, the first reacquired bbox can produce a large jump.

### Hardware Risks

Medium. This change alone does not directly raise motor speed, but it can send more aggressive goal changes. If combined with higher P gain or higher Dynamixel profile, it can increase:
- motor heating,
- gear load,
- mount shock,
- current spikes.

### System Risks

Low-Medium. CPU/GPU load does not increase much. U2D2 load can rise indirectly because fewer commands are skipped by smoothing/step limiting.

### Rollback

Immediate runtime rollback:

```bash
unset ULTRA_CHAN_SMOOTH_ALPHA_X ULTRA_CHAN_SMOOTH_ALPHA_Y
unset TARGET_CENTER_SMOOTHING_NEAR TARGET_CENTER_SMOOTHING_FAR
unset TARGET_MOTOR_MAX_STEP_PX TARGET_MAX_CENTER_STEP_PX
./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```

Or explicitly return to current demo defaults:

```bash
ULTRA_CHAN_SMOOTH_ALPHA_X=0.82 ULTRA_CHAN_SMOOTH_ALPHA_Y=0.76 \
TARGET_CENTER_SMOOTHING_NEAR=0.70 TARGET_CENTER_SMOOTHING_FAR=0.92 \
TARGET_MOTOR_MAX_STEP_PX=260 TARGET_MAX_CENTER_STEP_PX=360 \
./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```

## 2. Control Period Shortening

Proposed:
- `ULTRA_CHAN_CONTROL_PERIOD_SEC=0.012`
- Possibly test `0.008`.

Current demo default is set in `run_demo.sh:55`. The actual rate skip is in `jetson/src/control/ultra_yubin_motor.py:273`.

Risk grade: Medium for `0.012`, High for `0.008`.

### Functional Risks

| Period | Approx command rate | Risk |
|---:|---:|---|
| 0.018 sec | 55 Hz | Current safer baseline. |
| 0.012 sec | 83 Hz | Usually reasonable test point. |
| 0.008 sec | 125 Hz | Likely too aggressive unless serial and motors prove stable. |

Shorter period reduces waiting before a new command can be sent. But if the motor cannot physically move that fast, more commands do not always mean better tracking. It can feel more nervous without actually reducing camera lag.

### Hardware Risks

At 0.012, moderate. At 0.008, high if sync request/reply and U2D2 writes stack up.

The bridge writes the final Dynamixel goal around `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:1270`. At 57600 baud, one sync write can already cost several milliseconds. If command period gets close to serial write time, the bus can become the bottleneck.

Possible symptoms:
- motor buzzing,
- delayed commands,
- dropped/repeated replies,
- higher motor temperature,
- unstable pan/tilt motion.

### System Risks

Medium. More UDP packets and replies increase Jetson/Ultra96 scheduling pressure. It is unlikely to break AXI by itself, but it can make logging/telemetry noisier and expose timing bugs.

### Rollback

```bash
ULTRA_CHAN_CONTROL_PERIOD_SEC=0.018 ./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```

Do not keep `0.008` unless logs show stable replies and no motor buzz for several minutes.

## 3. PL P Gain And Max Step Increase

Proposed:
- Pixel error `/8` -> `/6` or `/5`.
- Max step `72` -> `90` to `110` tick.
- Requires RTL edit and Vivado bitstream rebuild.

Current related code:
- RTL max step: `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v:51`
- RTL pixel/8 helper: `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v:165`
- Bridge PS fallback constants: `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:44`, `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:45`

Risk grade: High.

### Functional Risks

| Risk | Why it can happen | Symptom |
|---|---|---|
| Oscillation | Higher P gain reacts harder to the same bbox noise. | Drone crosses center repeatedly. |
| Overshoot | Larger max step pushes past target before next frame arrives. | Fast snap, then correction back. |
| Close-range instability | Close target has larger pixel motion. Large gain is more dangerous near center. | Strong shaking at short distance. |
| False positive violence | A wrong bbox can command a larger correction. | Camera jumps hard to wrong location. |

If this is changed, keep a lower close-range clamp. For example, even if far max is 90-110, close max should stay around 32-45 until proven stable.

### Hardware Risks

High. Larger goal jumps create sharper mechanical movement:
- motor heat,
- gear stress,
- mount flex,
- current spikes,
- possible USB/U2D2 instability if motors pull too much current.

### System / Deploy Risks

High because it requires bitstream rebuild. RTL source changes do nothing until the new bitstream is built and deployed. A mismatched bridge/RTL pair can make PS fallback and PL behavior differ.

Live verification must check target replies:

```text
src=pl
```

If logs show `src=ps_direct`, the new PL gain is not being used.

### Rollback

Rollback is harder than env changes:
1. Restore previous RTL constants.
2. Restore matching bridge constants if changed.
3. Rebuild bitstream.
4. Redeploy old bitstream/HWH or run the saved previous bitstream.

Before testing, save:
- `bitstream/ultra_yubin_v1.bit`
- `bitstream/ultra_yubin_v1.hwh`
- current RTL file
- current bridge C file

Do not start this change within 2 days of final demo unless there is already a known-good old bitstream to restore.

## 4. Dynamixel Profile Velocity / Acceleration Increase

Proposed:
- `profile velocity 180 -> 250~350`
- `profile accel 60 -> 100~180`

Current defaults are in `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:761` and `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:762`. The bridge writes profile acceleration/velocity in `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:641` and `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:645`.

Risk grade: Medium-High.

### Functional Risks

This can make tracking feel much faster because it attacks physical motor lag, not just software lag. But if the controller is already too aggressive, faster motor profiles amplify overshoot.

Typical symptoms:
- better catch-up on smooth motion,
- worse shaking near center,
- stronger jump on false detection,
- more visible mechanical recoil.

### Hardware Risks

High compared with pure software filters:
- motor heating,
- shorter gear life,
- mount shock,
- loose screws over time,
- voltage dip when pan and tilt move together.

Test by touching motor temperature after short runs. If it is uncomfortable to touch, back down.

### System Risks

Low-Medium. The profile itself does not increase packet count, but faster motion can trigger more visual correction and make the control loop more active.

### Rollback

Return to:

```text
profile_accel=60
profile_velocity=180
```

If these are passed through deploy script flags/env, revert those settings. If hardcoded, restore the C defaults and redeploy the bridge.

Recommended stepping:
1. 220/80
2. 250/100
3. 300/140
4. 350/180 only if mount and motor stay stable

## 5. Lead / Prediction Compensation

Proposed:
- Estimate bbox velocity.
- Send `cx + vx * lead_frames`, `cy + vy * lead_frames`.
- Start `lead_frames=1.0`.

Risk grade: Medium-High.

### Functional Risks

Lead compensation can reduce the feeling that the camera is always behind the drone. It is useful because camera+YOLO delay is much larger than PL delay. But prediction is dangerous when the measurement is wrong.

| Risk | Cause | Mitigation |
|---|---|---|
| Wrong-direction jump | bbox switches target or false positive appears | Disable lead for low confidence and sudden bbox size jumps. |
| Over-lead | drone stops but prediction keeps pushing | Clamp lead offset, decay velocity quickly. |
| Edge instability | target near image edge has poor bbox center | Disable lead near edge. |
| No-target reacquire jump | stale velocity used after missing target | Reset velocity on no-target at `jetson/jetson_node.py:699`. |

Start with:
- `lead_frames=0.5` or `1.0`,
- max lead clamp 30-50 px first,
- disable lead when confidence is low,
- disable lead when bbox height changes too suddenly.

### Hardware Risks

Medium. Prediction itself is software, but bad prediction can command larger/faster motion. Risk increases when combined with high P gain and high motor profile.

### System Risks

Low-Medium. CPU cost is tiny. The real risk is logic correctness and stale state.

### Rollback

Implement behind an env flag:

```bash
TARGET_LEAD_FRAMES=0
```

or similar. If there is no kill switch, rollback requires code revert.

This should not be merged into demo behavior unless it has a one-command disable path.

## 6. Async YOLO Worker, Chanhee Style

Proposed:
- Separate main/control loop and YOLO inference loop.
- Use latest result only.

Reference points:
- Worker starts around `/home/jetson/ultra_chan/main.py:94`.
- Main loop submits newest frame around `/home/jetson/ultra_chan/main.py:780`.
- Main loop consumes latest result around `/home/jetson/ultra_chan/main.py:781`.

Risk grade: High.

### Functional Risks

This can help a lot if inline inference is blocking control/display. But it creates stale-result and synchronization problems.

Failure scenarios:
- motor acts on old bbox after the drone already moved,
- no-target state races with a late detection result,
- frame id mismatch makes logs confusing,
- UI shows one frame while motor follows another frame,
- shutdown becomes unstable.

Required safety rules:
- queue size 1 or latest-frame only,
- latest-result age limit, for example reject result older than 80-120 ms,
- frame id/time stamp attached to every result,
- reset prediction and smoothing when result is stale,
- one owner thread for YOLO model/GPU context.

### Hardware Risks

Medium. If stale results are accepted, the gimbal can move in the wrong direction. If latest-only and max-age checks are correct, hardware risk is manageable.

### System Risks

High:
- thread safety,
- memory growth if queue is not bounded,
- GPU context behavior,
- Ctrl+C/shutdown issues,
- harder debugging.

The project has already seen shutdown-only segfaults after exit. Adding inference threads can make shutdown behavior worse unless carefully owned and joined.

### Rollback

Keep inline inference as default. Add async worker behind an env flag:

```bash
YOLO_ASYNC_WORKER=0
```

If async is implemented without a flag, rollback requires code revert. Do this only with enough time to test.

## 7. Bitstream Rebuild And PL Mode Forced Deploy

Proposed:

```bash
ULTRA_YUBIN_V1_TRACK_DIRECT_PS=0
ULTRA_YUBIN_V1_RESTART=1
```

Risk grade: Medium for planned deploy, High if done right before demo.

### Functional Risks

The biggest risk is believing PL is active when it is not. Source default is `track_direct_ps = 0` at `hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c:760`, but deployment flags or old bridge process can still run direct PS.

Live target replies must show:

```text
src=pl
```

If target replies show:

```text
src=ps_direct
```

then the capstone PL tracking argument is weak because pan/tilt target correction is being computed in PS.

### Hardware Risks

Low-Medium if the bitstream matches tested behavior. High if the rebuilt RTL has aggressive gain/clamp and is not tested.

### System / Deploy Risks

| Risk | Detail |
|---|---|
| Build failure | Vivado may fail timing/DRC or produce missing artifact. |
| Wrong artifact deployed | Old `.bit`/`.hwh` can be copied accidentally. |
| Bridge mismatch | C constants and RTL constants can differ. |
| PL init/open failure | Bridge can fall back or fail if `/dev/mem`/register map is wrong. |
| Demo confusion | Logs with `src=ps_direct` undermine PL explanation. |

This should not make Linux fail to boot if bitstream loading is a runtime step, but it can make the tracking service fail or fall back.

### Rollback

Keep a known-good backup:

```text
bitstream/ultra_yubin_v1.bit
bitstream/ultra_yubin_v1.hwh
```

Emergency options:
1. Redeploy previous bitstream/HWH.
2. Restart bridge with PS direct only for emergency demo.
3. Revert RTL and rebuild if there is enough time.

For final capstone defense, emergency PS direct is operationally useful but weakens the "we use PL for tracking control" claim.

## Interaction Risks

### Risk Amplification

Some changes multiply each other:

| Combination | Risk |
|---|---|
| Smoothing off + PL gain `/5` + max step 110 | High oscillation and false-positive jump risk. |
| Control period 0.008 + high profile velocity | U2D2/motor command pressure plus harsher movement. |
| Lead compensation + weak filters + low confidence threshold | Wrong bbox gets predicted forward and chased harder. |
| Async YOLO + no max-result-age + lead compensation | Stale result becomes a predicted wrong command. |
| High Dynamixel accel + large PL max step + flexible mount | Mechanical shock and visible bounce. |
| Rebuilt bitstream + no old backup + demo day | Hard rollback if PL behavior is bad. |

### Must Avoid

Do not apply this all at once:

```bash
ULTRA_CHAN_SMOOTH_ALPHA_X=1.0
ULTRA_CHAN_SMOOTH_ALPHA_Y=1.0
TARGET_CENTER_SMOOTHING_NEAR=1.0
TARGET_CENTER_SMOOTHING_FAR=1.0
TARGET_MOTOR_MAX_STEP_PX=500
TARGET_MAX_CENTER_STEP_PX=500
ULTRA_CHAN_CONTROL_PERIOD_SEC=0.008
```

combined with:
- PL gain `/5`,
- max step 110,
- profile velocity 350,
- profile accel 180,
- lead compensation enabled.

That combination is likely to oscillate, overcorrect, and stress the mechanism.

## Safe Application Order

Recommended order:

1. **Backup and baseline**
   - Save current bitstream/HWH.
   - Save logs from current run.
   - Confirm target replies show either `src=pl` or `src=ps_direct`.

2. **Force PL mode with conservative behavior**
   - Use PL path first with current `/8`, max 72.
   - Confirm `src=pl`.
   - Do not raise gain yet.

3. **Env-only smoothing reduction**
   - Test `ULTRA_CHAN_SMOOTH_ALPHA_X/Y=1.0`.
   - Test `TARGET_CENTER_SMOOTHING_NEAR=0.85~0.90`, `FAR=0.95~1.0`.
   - Keep max step moderate first.

4. **Control period 0.012**
   - Only after smoothing test is stable.
   - Watch motor buzz and reply stability.

5. **Mild Dynamixel profile**
   - Try 250/100 before 350/180.
   - Watch heat and mount shock.

6. **Lead compensation with kill switch**
   - Start lead 0.5 or 1.0 frame.
   - Clamp lead to 30-50 px first.
   - Disable on low confidence/stale result.

7. **PL gain increase**
   - Try `/6` and max 90 before `/5` and 110.
   - Keep close max lower.
   - Requires rebuild/deploy and longer validation.

8. **Async YOLO worker**
   - Do only when baseline is stable.
   - Use latest-only and max-age checks.
   - Avoid near final demo unless already proven.

## D-Day Schedule Recommendation

Assume D-day means final demonstration day.

| Deadline | Safe work |
|---|---|
| D-7 to D-5 | Async YOLO worker, major RTL changes, bitstream rebuild experiments. |
| D-5 to D-3 | PL mode forced deployment, `/6` gain test, moderate Dynamixel profile, lead compensation with kill switch. |
| D-3 to D-2 | Env-only smoothing/control-period tuning, mild profile tuning, logging/validation. |
| D-1 | Only runtime env tuning with instant rollback. No RTL rebuild, no async refactor, no aggressive profile. |
| D-day | Use last known-good command only. Do not change bitstream or controller logic. |

## Rollback Matrix

| Change | Rollback difficulty | Rollback method |
|---|---|---|
| Smoothing/filter envs | Easy | Unset envs or restore `run_demo.sh` defaults. |
| Control period env | Easy | `ULTRA_CHAN_CONTROL_PERIOD_SEC=0.018`. |
| PL gain/max step RTL | Hard | Restore RTL/bridge constants, rebuild, redeploy old bitstream. |
| Dynamixel profile | Medium | Restart bridge/deploy with accel=60 velocity=180. |
| Lead compensation | Easy if env-gated, medium otherwise | Set lead env to 0; otherwise revert code. |
| Async YOLO worker | Medium-Hard | Disable env flag if available; otherwise revert code. |
| Force PL bitstream | Medium | Redeploy old bitstream/HWH or emergency PS direct mode. |

## Most Important Practical Recommendation

The safest starting combination is:

```bash
ULTRA_YUBIN_V1_TRACK_DIRECT_PS=0 \
ULTRA_CHAN_CONTROL_PERIOD_SEC=0.018 \
ULTRA_CHAN_SMOOTH_ALPHA_X=1.0 \
ULTRA_CHAN_SMOOTH_ALPHA_Y=1.0 \
TARGET_CENTER_SMOOTHING_NEAR=0.85 \
TARGET_CENTER_SMOOTHING_FAR=0.95 \
TARGET_MOTOR_MAX_STEP_PX=300 \
TARGET_MAX_CENTER_STEP_PX=400 \
./run_demo.sh --pipeline-echo --pipeline-echo-every 30
```

Then verify:

```text
src=pl
```

If it jitters, reduce aggression before touching RTL:

```bash
ULTRA_CHAN_SMOOTH_ALPHA_X=0.90
ULTRA_CHAN_SMOOTH_ALPHA_Y=0.88
TARGET_CENTER_SMOOTHING_NEAR=0.75
TARGET_CENTER_SMOOTHING_FAR=0.90
```

Only after this is stable should PL gain, Dynamixel profile, lead compensation, or async worker be changed.

## Final Judgment

The lag is partly structural, but not mainly PL propagation delay. The biggest safe wins are:

1. Use PL mode for the capstone story, but keep conservative PL constants first.
2. Reduce Jetson-side smoothing/step limits gradually.
3. Test 0.012 sec control period, but avoid 0.008 unless logs and motors prove stable.
4. Tune Dynamixel profile carefully because it affects real hardware stress.
5. Add lead compensation only with clamps and a kill switch.
6. Treat async YOLO as a larger refactor, not a last-minute tuning change.

