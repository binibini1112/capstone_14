# Drone Scenarios

Press `p` in the drone CLI, then choose a scenario number.

## 1 - `1_rc_front_view_infinity`

RC-based front-view infinity. `forward_back` is always `0`; only `left_right` and `up_down` move.

Use this only after the simple SDK movement scenarios are stable. It uses timed RC control, so the exact shape depends on battery, floor texture, lighting, and drift.

## 2 - `2_front_view_corner_pause`

Start -> up 50cm -> right 50cm -> down 50cm -> left 50cm back to start. Every vertex waits 1 second.

This is the safest visual shape because it uses only single-axis Tello SDK movement commands.

## 3 - `3_demo_rectangle_center_return`

Demo path at manually set 80-100cm altitude:

1. right 50cm
2. wait 1 second
3. forward 50cm
4. wait 1 second
5. left 100cm
6. wait 1 second
7. back 50cm
8. wait 1 second
9. right 50cm to return to center
10. wait 3 seconds

Set the altitude manually before running this scenario.

## 4 - `4_stage_demo_forward_side_climb`

Stage demo path:

1. up 50cm
2. forward 200cm
3. left 200cm
4. right 200cm
5. up 100cm
6. hover until Jetson sends `laser.hit_detected=true`
7. auto hit response lands the drone
