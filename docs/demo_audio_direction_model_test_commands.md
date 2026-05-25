# Audio Direction Model Demo Test Commands

Default final demo uses the Junyoung CNN6 new direction model:

```bash
./run_demo_pl_drive.sh
```

Use the same parameter preset with only the script name changed to compare models.

## Model Scripts

| Model | Script | Path |
| --- | --- | --- |
| Junyoung new / final default | `./run_demo_pl_drive.sh` | `models/audio_direction/junyoung_cnn6_new/audio_angle_cnn_final.tflite` |
| JH model 1 | `./run_demo_pl_drive_jhmodel_tmp.sh` | `jhmodle/audio_angle_4ch_1dcnn_float32.tflite` |
| JH model 2 | `./run_demo_pl_drive_jhmodel2_tmp.sh` | `audio_angle_experiment_outputs_tflite/audio_angle_4ch_rawwave_1dcnn_float32.tflite` |
| JH model 3 / bgtest | `./run_demo_pl_drive_jhmodel3_bgtest_tmp.sh` | `audio_angle_experiment_outputs_tflite_bgtest/audio_angle_4ch_rawwave_1dcnn_float32_bgtest.tflite` |

## Preset 1: Stable

```bash
TELLO_AUDIO_THRESHOLD=0.98 \
TELLO_AUDIO_MIN_AVG_SCORE=0.90 \
TELLO_AUDIO_MIN_RMS=0.0025 \
TELLO_AUDIO_STABLE_WINDOW=5 \
TELLO_AUDIO_STABLE_MIN_VOTES=3 \
TELLO_AUDIO_MOTOR_NOISE_BLIND_SEC=1.0 \
./run_demo_pl_drive.sh
```

## Preset 2: Distance

```bash
TELLO_AUDIO_THRESHOLD=0.95 \
TELLO_AUDIO_MIN_AVG_SCORE=0.85 \
TELLO_AUDIO_MIN_RMS=0.0020 \
TELLO_AUDIO_STABLE_WINDOW=5 \
TELLO_AUDIO_STABLE_MIN_VOTES=3 \
TELLO_AUDIO_MOTOR_NOISE_BLIND_SEC=1.0 \
./run_demo_pl_drive.sh
```

## Preset 3: Fast Response

```bash
TELLO_AUDIO_THRESHOLD=0.90 \
TELLO_AUDIO_MIN_AVG_SCORE=0.80 \
TELLO_AUDIO_MIN_RMS=0.0025 \
TELLO_AUDIO_STABLE_WINDOW=3 \
TELLO_AUDIO_STABLE_MIN_VOTES=2 \
TELLO_AUDIO_MOTOR_NOISE_BLIND_SEC=0.8 \
./run_demo_pl_drive.sh
```

## Recommended Test Order

1. Preset 1 + Junyoung
2. Preset 1 + JH model 1
3. Preset 1 + JH model 2
4. Preset 1 + JH model 3/bgtest
5. If Preset 1 misses the drone at range, repeat the best two models with Preset 2.
6. Use Preset 3 only for quick reaction-speed checks in a quiet place.

For example, JH model 3 with the distance preset:

```bash
TELLO_AUDIO_THRESHOLD=0.95 \
TELLO_AUDIO_MIN_AVG_SCORE=0.85 \
TELLO_AUDIO_MIN_RMS=0.0020 \
TELLO_AUDIO_STABLE_WINDOW=5 \
TELLO_AUDIO_STABLE_MIN_VOTES=3 \
TELLO_AUDIO_MOTOR_NOISE_BLIND_SEC=1.0 \
./run_demo_pl_drive_jhmodel3_bgtest_tmp.sh
```
