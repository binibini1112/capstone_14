# Laser C Motor Plan

## Goal

Add a third Dynamixel motor, called C, above the current pan-tilt camera mount.
C carries the laser and corrects the vertical offset between the camera optical
axis and the laser axis.

## Mechanical Layout

Recommended structure:

```text
pan motor
  -> tilt motor
      -> camera mount
      -> C motor mount
          -> laser module
```

C should rotate the laser mainly in the vertical direction. When the camera
target is centered, moving only C should move the laser spot up and down in the
camera image. If the laser moves diagonally or sideways, the C mount alignment
needs to be corrected first.

## Calibration Strategy

Use bbox size as a distance proxy because no range sensor is currently present.
For several known distances:

1. Put the drone or a target at a fixed distance.
2. Track/align the pan-tilt camera so the target is centered.
3. Move only C until the laser hits the target.
4. Store `(bbox_h, c_goal)` or `(bbox_area, c_goal)`.
5. During demo, interpolate between table entries.

Example table shape:

```json
[
  {"bbox_h": 180, "c_goal": 2050},
  {"bbox_h": 130, "c_goal": 2120},
  {"bbox_h": 90,  "c_goal": 2240},
  {"bbox_h": 60,  "c_goal": 2380}
]
```

## Implementation Direction

Start in software/Ultra96 PS, not PL:

- Jetson keeps YOLO detection and sends bbox data.
- Ultra96 PS receives bbox data and drives pan, tilt, and C.
- PL continues to accelerate pan-tilt target computation.
- C is updated slowly from a calibration table and smoothed to avoid jitter.

Suggested motor IDs:

```text
pan  = 1
tilt = 2
C    = 3
```

## Notes for Later

- Keep C update slower than pan-tilt, about 5-10 Hz.
- Smooth bbox height before mapping to C.
- Clamp C to a narrow safe range during early tests.
- Add manual C nudge script before enabling automatic laser correction.
