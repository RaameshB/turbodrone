# Monocular vSLAM (ROS 2 Jazzy + stella_vslam)

Camera-only. No IMU needed, but map/trajectory scale is arbitrary and drifts.
Start with translation (not pure rotation) so the map can initialise.

Runs in the `ros2` distrobox (Ubuntu 24.04, ROS 2 Jazzy). Workspace: `~/slam_ws`.

## Workflow
1. Short drone session (backend running with `DRONE_TYPE=wifi_cam`):
   - `python slam/record_frames.py slam/recordings/calib --seconds 60`  (checkerboard in view, moved/tilted)
   - `python slam/record_frames.py slam/recordings/flight1`             (the flight itself)
2. Drone off. Calibrate (inside distrobox, needs OpenCV):
   `distrobox enter ros2 -- python3 slam/calibrate.py slam/recordings/calib --cols 9 --rows 6`
   → writes `slam/config/drone.yaml`
3. Run SLAM on the recording (real-time replay):
   `slam/run.sh --dir slam/recordings/flight1 --config slam/config/drone.yaml`
   Live instead: `slam/run.sh --url http://127.0.0.1:8000/mjpeg --config slam/config/drone.yaml`
   Add `--map-out map.msg` to save the map.

Output topics: `/run_slam/camera_pose` (PoseStamped, map frame), `/run_slam/keyframes`.

## Pipeline check (no drone)
`slam/run.sh --dir slam/data/tum_xyz_frames --config slam/config/tum_fr1_mono.yaml`
(needs `slam/data/orb_vocab.fbow` and the converted TUM fr1/xyz frames).
