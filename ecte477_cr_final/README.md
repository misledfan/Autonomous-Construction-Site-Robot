# construction_robot (ECTE477 project)

## Nodes and communication
| Node | Purpose | Interfaces |
|---|---|---|
| `destination_selector` (destination_selector.py) | Terminal menu; sends the chosen zone to Nav2; stops on RED, resumes on GREEN; triggers zone verification and prints the result | **action client** `/navigate_to_pose` (nav2_msgs/NavigateToPose); **service clients** `/sheet_detector/enable` (std_srvs/SetBool), `/verify_zone` (std_srvs/Trigger); sub `/sheet_signal`; pub `/requested_zone`, `/cmd_vel` |
| `sheet_detector` | Red/green A4 sheet detection (OpenCV HSV + area/shape gating + 3-of-4 debounce), 4 Hz on 160 px frames | sub camera; pub `/sheet_signal` (std_msgs/String, latched); **service server** `/sheet_detector/enable` |
| `zone_verifier` | Zone colour (OpenCV), shape counts in the zone colour only (OpenCV contours), object counts (custom CNN, cv2.dnn), majority vote over 7 fresh frames | sub camera, `/requested_zone`; **service server** `/verify_zone`; pub `/zone_report` |
| Nav2 (turtlebot3_navigation2) | map_server, AMCL, planner (NavFn), controller (DWB), recoveries, RViz | params: `config/nav2_params.yaml` |

Why these types: navigation is long-running with feedback and cancel, so it's an **action**. Enable and verify are one request with one answer, so they're **services**. The sheet state changes continuously, so it's a **latched topic**.

## Run (lab PC; the robot runs bringup + v4l2_camera over SSH)
    source ~/tb3_env.sh
    ros2 launch ecte477_cr_final construction_robot.launch.py
Then do 2D Pose Estimate in RViz, wait for Navigation: active, and pick a zone in the xterm menu.

## Tools
* `ros2 run ecte477_cr_final snapshot`: saves camera frames (Enter) to ~/cr_snapshots
* `ros2 run ecte477_cr_final test_vision IMG...`: runs the same vision code offline and writes *_annotated.png
* `ros2 run ecte477_cr_final hsv_tuner IMG --colour red`: tunes the HSV ranges in config/vision.yaml
* `python3 training/train_cnn.py --data training/data --out models/object_cnn.onnx`: retrains the CNN

## Files
* `config/zones.yaml`: zone goal poses (map frame) and colours
* `config/vision.yaml`: HSV ranges, rates, thresholds
* `config/nav2_params.yaml`: Nav2 tuned for the small arena (inflation 0.28 m, goal tolerance 0.10 m / 0.15 rad, AMCL update every 0.10 m / 0.10 rad)
* `models/object_cnn.onnx`: custom CNN (NumPy-trained, 3 conv layers); `training/`: dataset and training script
* `test/test_vision_utils.py`: unit tests on photos of the real markers
