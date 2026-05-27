# Laser Motor Plan

## Goal

Add a third Dynamixel motor above the current pan-tilt camera mount.
This laser motor carries the laser module and corrects the vertical offset
between the camera optical axis and the laser axis.

## Mechanical Layout

Recommended structure:

```text
pan motor
  -> tilt motor
      -> camera mount
      -> laser motor mount
          -> laser module
```

The laser motor should rotate the laser mainly in the vertical direction.
When the camera target is centered, moving only the laser motor should move
the laser spot up and down in the camera image. If the laser moves diagonally
or sideways, the laser motor mount alignment should be corrected first.

## Calibration Strategy

Use bbox size as a distance proxy because no range sensor is currently present.
For several known distances:

1. Put the drone or a target at a fixed distance.
2. Track/align the pan-tilt camera so the target is centered.
3. Move only the laser motor until the laser hits the target.
4. Store `(bbox_h, laser_goal)` or `(bbox_area, laser_goal)`.
5. During demo, interpolate between table entries or use a center-lock profile.

Example table shape:

```json
[
  {"bbox_h": 180, "laser_goal": 2050},
  {"bbox_h": 130, "laser_goal": 2120},
  {"bbox_h": 90,  "laser_goal": 2240},
  {"bbox_h": 60,  "laser_goal": 2380}
]
```

## Current Demo Direction

The final demo currently uses a stable center-lock profile:

```bash
LASER_CAMERA_CENTER_LOCK=1
LASER_CAMERA_CENTER_TICK=1945
LASER_CAMERA_CENTER_RANGE_COMP=1
```

The bbox-height table approach remains a calibration option, but it is not the
default path used by `./run_demo_pl_drive.sh`.

Suggested motor IDs:

```text
pan         = 1
tilt        = 2
laser motor = 3
```

## Notes for Later

- Keep laser motor updates slower than pan-tilt, about 5-10 Hz.
- Smooth bbox height before mapping to laser motor tick.
- Clamp the laser motor to a narrow safe range during early tests.
- Add a manual laser nudge script before enabling automatic laser correction.
