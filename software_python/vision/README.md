# MediaPipe/world alignment prototype

This is an experimental external tool. It does not modify or call an EKF update.
The current experiment is fixed-camera: the camera remains stationary while we
inspect the relationship between MediaPipe landmarks, a chosen source-frame
representation, and the existing EKF world frame. Dynamic camera pose
estimation and measurement fusion are deliberately out of scope.

## Coordinate conventions

- **MediaPipe image landmarks:** `x` increases right, `y` increases down, and `z` is relative depth (more negative is generally closer to the camera). `x` and `y` are normalized by image width/height; this is not metric 3-D camera position and is never treated as metres by this prototype. The tracker preserves all 21 landmarks and a wall-clock timestamp.
- **MediaPipe WorldLandmarks:** when enabled, MediaPipe supplies approximately metric 3-D hand-local coordinates in metres. Their origin is near the hand's geometric centre, so they are not an absolute camera pose or camera-fixed translation. They are the preferred source for the current 3-D hand-geometry experiment, but any fitted transform must be labelled `hand_local_to_ekf`, not `camera_to_ekf`.
- **Firmware sensor frame:** the raw MPU6886/M5Unified accelerometer and gyroscope axes are the sensor/body axes after firmware calibration. The repository does not expose a separate physical axis-remapping matrix, so no undocumented sign flip is applied here.
- **NavEKF/world frame:** `NavEKF` stores position in metres and publishes a quaternion through `getQuaternion()` / the `NAV_QUAT_STREAM_ON` telemetry block. The quaternion is `(w,x,y,z)` and represents world-from-sensor; its DCM maps sensor vectors into the EKF world. The world frame is initialized from the first stationary Mahony attitude when the EKF arms. Because yaw has no magnetometer reference, its absolute heading is session/setup dependent. The EKF uses a world +Z gravity convention, as shown by its gravity subtraction in `NavEKF.h`.
- **Euler angles:** firmware display angles are encoded yaw/pitch/roll, and the existing Python helper interprets them as intrinsic `ZYX`. The alignment prototype uses the quaternion, not Euler angles.

## Transformation and calibration

The prototype uses the explicit similarity transform only when the source
coordinates justify that model:

`p_world = scale * R_world_camera @ p_camera + t_world_camera`.

`R_world_camera` is a camera-to-EKF rotation only if `p_camera` is genuinely
camera-fixed metric data. For current MediaPipe WorldLandmarks, it is instead a
source-to-EKF registration between hand-local coordinates and the EKF frame;
the hand-local origin means it cannot provide absolute wrist translation by
itself. For normalized image landmarks, the transformed result is explicitly
visualization coordinates only and has no physical metre interpretation.

Start with identity only when the source axes and EKF/world axes are deliberately
co-located. For a reproducible calibration, collect at least three non-
collinear corresponding points in both frames and run `fit_similarity()`. The
SVD fit returns a proper rotation, positive scale, and translation.

A practical first experiment is: keep the camera fixed, start the EKF so it
arms, keep the glove orientation approximately constant, and perform controlled
left/right, forward/back, and up/down motions. With WorldLandmarks, inspect hand
shape/orientation consistency and do not interpret the hand-local wrist as
absolute camera translation. With a separately calibrated camera-fixed metric
source, inspect absolute translation against EKF position. The output is a
frame-consistency check, not absolute camera localization.

For paired calibration samples in a CSV with `camera_x/y/z` and `world_x/y/z` columns, fit the transform with:

`python -m vision.calibrate_alignment points.csv vision/calibration.json`

## Running

Install the optional experiment dependencies in the active environment:

`pip install mediapipe opencv-python matplotlib numpy`

From `software_python`:

Camera-only, preferred 3-D hand-local source:

`python -m vision.run_alignment --source world --no-ekf --camera 0 --calibration vision/hand_local_to_ekf.json --log vision/camera_only.csv --show`

Live MediaPipe plus existing glove telemetry:

`python -m vision.run_alignment --source world --hand r --camera 0 --calibration vision/hand_local_to_ekf.json --log vision/live_alignment.csv --show`

Use `--source image` for normalized image landmarks. The logger marks these as
`image_normalized`, and the configured scale is only a visualization scale.
Use `--source world` (or the legacy `--world-landmarks` flag) for MediaPipe
WorldLandmarks, marked `world_metric_hand_local`.

Recorded-data visualization, with no camera, glove, or MediaPipe:

`python -m vision.replay_alignment vision/live_alignment.csv --frame -1 --plot-offset`

Each CSV row preserves the MediaPipe PC timestamp, PC telemetry receipt time,
the parser-relative timestamp, raw `device_us`, an unwrapped device-clock
estimate, and the paired MediaPipe-PC-minus-device offset. The log also keeps
the PC-receipt-minus-device diagnostic. The offset plot describes the observed
clock relationship plus capture/transport timing; it does not establish
synchronization.

The bridge enables `NAV_QUAT_STREAM_ON` and `MOTION_STREAM_ON` through the existing `Reader`/`ParseSerial` path. It reads `nav_quat`, `position`, `nav_armed`, and `device_us`; it never sends MediaPipe data back to the firmware, injects a measurement, or modifies `NavEKF.h`.

## Limitations / next step

Camera pose is fixed and offline. There is no dynamic camera pose estimation,
absolute camera pose, hand-to-glove registration, solved time synchronization,
or EKF measurement update. Before using MediaPipe as a future measurement,
validate the rigid registration, source origin, scale, latency, quaternion
convention, and the `nav_armed`/ZUPT quality gates over recorded controlled
movements.
