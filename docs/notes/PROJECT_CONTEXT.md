# ECTE477 Project: Autonomous Construction-Site Robot (context file for Claude CLI)

> **Instructions for Claude CLI. Read all of this before doing anything.**
>
> **Who I am and what I want**
> - I'm a student doing the ECTE477 group project, using **ROS 2 Humble in WSL2 (Ubuntu 22.04) on my own Windows laptop**.
> - I need to **understand** everything we build, because I have to explain it in the report and the live demo. Explain each design decision and each command before running it.
> - Build in the **milestone order** in Section 9. Get one piece working and tested before starting the next.
> - Write code that is clear, commented, parameterised and robust. Don't produce the generic "one big script" every other group will hand in.
>
> **Rules**
> 1. **Never hard-code paths or detections.** The assessors may change the environment or destination. Put zone poses, HSV ranges, thresholds and topic names in YAML config or ROS parameters.
> 2. **Don't invent facts about the arena.** I'll provide photos or descriptions of each zone's markers and objects, and the CNN dataset link. Ask me for them when they're needed (Section 6.4).
> 3. **Safety:** don't send motion commands or navigation goals to the real robot unless I confirm it's on the floor in a clear area and I'm watching.
> 4. **Don't change the robot's system configuration** (the course forbids it): no editing the Pi's `.bashrc`, passwords or network settings. Ask me first before installing anything on the Pi.
> 5. Before writing code against an API, check what's actually installed (`ros2 pkg list`, `ros2 interface show`, `ros2 topic info -v`).

---

> **CURRENT SETUP:** I'm working on a **uni lab PC with native Ubuntu 22.04 + Humble** (user `uowd`), not WSL. Skip Section 2 except 2.5 (clock sync). Don't change the lab PC's system configuration. Back up all work (workspace, map, bags) to USB or git at the end of every session, because lab PCs may be shared or reset.

## 0. Known facts

| Item | Value |
|---|---|
| ROS distro | **Humble** (course standard; check with `echo $ROS_DISTRO`) |
| My machine | Windows laptop → **WSL2 Ubuntu 22.04**; GUI apps through WSLg |
| Robot | TurtleBot3 **Burger**, Raspberry Pi 4B, USB camera |
| Robot IP | **192.168.0.37** (changed robot again; earlier .229, .141) |
| Robot login | `ssh ubuntu@192.168.0.37` (password from the course manual) |
| Lab network | **CSLAB / CSLAB2** (Wi-Fi or wired). My laptop must be on it. |
| ROS_DOMAIN_ID | **8** for the current robot (.37) (check on the Pi with `grep ROS_DOMAIN_ID ~/.bashrc`) |
| SLAM tool (course) | Cartographer: `ros2 launch turtlebot3_cartographer cartographer.launch.py` |
| Camera driver (on Pi) | `ros2 run v4l2_camera v4l2_camera_node` → `/image_raw` |
| Required file names | `destination_selector.py`, `construction_robot.launch.py` |
| Required node count | **≥ 3 nodes** started by my launch file |

### 0.1 The saved map (M2 done)
- Files: `tb3_project_map.yaml` + `tb3_project_map.pgm`. Keep that name; reference it in config instead of `site_map`.
- 55×44 px at 0.05 m/px, so the whole image covers **2.75 × 2.20 m**. The usable floor inside the walls is only about **2.0 × 1.5 m**. Origin `[-0.523, -0.65, 0]`.
- Layout: an enclosed rectangle with three partition walls making bays. The robot started at map `(0, 0)`, facing +x, in the bottom-left bay. That spot is only about 15 cm from the left wall and 25 cm from the bottom wall.
- **Consequences for Nav2:** the TurtleBot3 Burger default config (`burger.yaml`) is tuned for big rooms:
  - `inflation_radius: 1.0` covers almost this whole arena with high cost;
  - `xy_goal_tolerance: 0.25` is huge compared with 0.5 m bays.

  Copy `burger.yaml` into `config/nav2_params.yaml` and tune it. Suggested starting points, to be tested and justified in the report: inflation radius about 0.25–0.30 m, cost_scaling_factor about 5–8, xy tolerance about 0.10 m, yaw tolerance about 0.15 rad. Increase `robot_radius` if the camera mount sticks out beyond the base.

### 0.2 Recorded zone goal poses (map frame, from `tf2_echo map base_link`)
| Zone | Colour | x | y | yaw (deg) |
|---|---|---|---|---|
| Electrical | blue | 0.443 | 0.065 | 179.5 (re-recorded further back; old 0.065, 0.027 was too close to the wall) |
| Mechanical | pink | 0.084 | 0.806 | 112.1 |
| Material Storage | yellow | 1.618 | 0.775 | 97.2 |

Stored in `config/zones.yaml`. Nav2 goals to these were tested in RViz and reported "Goal succeeded" with 0 recoveries.
- Blue is in the start (0, 0) bay, about 0.2 m from the left wall and facing it. It's tight, so check the camera framing on arrival.
- The demo start spot still needs choosing and taping. Prefer one outside the blue bay, so every zone needs a real route.
- The robot now in use has `/image_raw/compressed`. The lab PC has no image_transport compressed plugin, so view `/image_raw` in rqt; the nodes decode JPEG themselves with `cv2.imdecode`.

### 0.3 What the zone markers actually look like (from my phone photos)
Each zone is an A4 sheet on the brown MDF wall, at about camera height.
- **Electrical (blue):** 4 large **blue squares**, plus distractors on the same sheet: 2 pink squares and 2 pink circles. Requirement 13 says count only the zone colour, so the expected output is **blue squares = 4** with the pink ignored. The zone decision uses colour dominance: blue has the largest area on this sheet.
- **Mechanical (pink):** all pink: **1 square, 1 triangle, 3 circles**.
- **Material Storage (yellow):** one big **yellow circle** with **2 light-bulb pictures** inside. The CNN must recognise and count **light_bulb = 2**. The bulbs are greyish photos, not coloured shapes.
- Backgrounds: brown MDF wall, white paper margins, grey floor. The brown wall has low-saturation orange hues, so keep the saturation floors high in the HSV ranges.
- Pink here is magenta (bright, saturated), which is close to red. That's another reason red-sheet detection needs the area/shape gating and must be disabled during verification.

### 0.4 Code progress
- **Navigation verified on the real robot (.37):** with the tuned `config/nav2_params.yaml` (inflation 0.28, cost_scaling 6.0, xy tol 0.10, yaw tol 0.15, AMCL update_min_d/a 0.10), the robot reached all 3 zones in every order tested, with correct camera framing. Lesson: always check that the scan matches the walls before sending a goal.
- A copy of the working `nav2_params.yaml` is kept in `construction_robot_v1/config/`. Notes on that file:
  - It's an older-format TB3 file: `recoveries_server`, not Humble's `behavior_server`. Recoveries still ran, using defaults.
  - `use_sim_time: True` under global_costmap and recoveries_server is overridden by the launch file's `use_sim_time:=False` rewrite, so it's harmless.
  - If the robot gets stuck in corners again, the next knob is `progress_checker.required_movement_radius` 0.5 → 0.2 with `movement_time_allowance` 15.0. At 0.5 m in 10 s, slow turning in tight bays counts as "no progress" and triggers recoveries.
- `construction_robot/destination_selector.py` **v1** is done: menu → Nav2 `/navigate_to_pose` using `config/zones.yaml`, feedback, Ctrl+C cancels. It's installed by `install_v1.sh`. The next refactor moves the Nav2 logic into `mission_controller.py`.

### 0.5 v2: full system (built 2026-11-18)
Delivered as `construction_robot_v2.tar.gz`; install with `bash ~/construction_robot_v2/setup_pc.sh <DOMAIN_ID>`.
- Nodes: `destination_selector` (menu + Nav2 action client + RED stop / GREEN resume + verification), `sheet_detector` (4 Hz, 160 px, 3-of-4 debounce, SetBool enable service), `zone_verifier` (Trigger service, 7 fresh frames, colour dominance, shapes in the zone colour only, CNN objects, saves an annotated image to ~/cr_reports).
- Interfaces are standard only (std_srvs SetBool/Trigger, std_msgs String), so there's no custom interface package to build.
- The CNN is a custom 3-conv network trained in NumPy (`training/train_cnn.py`), exported to ONNX, and run with cv2.dnn. The dataset seed comes from phone photos (only 2 bulb instances, heavily augmented). Validation accuracy is 98% on augmented data. Add robot-camera crops and retrain.
- Tested off-robot: pytest on marker photos (blue 4 squares, pink 3 circles/1 triangle/1 square, yellow 2 bulbs), and node logic against a fake ROS layer (pause/resume, latched red, fallback from compressed to raw, verifier match/mismatch).
- NOT yet tested on the real robot. Red/green HSV ranges come from synthetic tests only, so check them with `snapshot` + `test_vision`.

### 0.6 FINAL v4: unique names (2026-11-19)
Shared lab PCs had other packages called `construction_robot` (e.g. in ~/turtlebot3_ws) that shadowed ours. So the package is now **`ecte477_cr_final`**, in its own workspace **`~/cr_final_ws`**, with its own settings file **`~/cr_final_env.sh`**, installed by `bash ~/cr_final_v4/setup_cr_final.sh <DOMAIN>`. The required file names `destination_selector.py` and `construction_robot.launch.py` are unchanged. The menu prints "ecte477_cr_final (FINAL v4)" at startup. Menu option `s` sets the AMCL initial pose to the taped START spot (map origin, camera facing up the arena toward the yellow end). The arena did NOT change; earlier trouble was mislocalisation.

---

## 1. What the project asks for

1. Map the arena (Cartographer + teleop), then save the map.
2. Run one launch file. The robot **stays still** until the user picks a zone in a Python program (`destination_selector.py`).
3. Nav2 plans and drives to the zone. **No hard-coded movement sequences.** Nav2 avoids obstacles using the LiDAR.
4. While driving, the camera watches for an A4 sheet. **Red seen consistently → stop and stay stopped. Green seen consistently → resume to the same zone**, without asking the user again.
5. On arrival, wait for the view to settle, then analyse **fresh** frames:
   - **Zone identity from colour** using OpenCV (not a CNN): yellow = Material Storage, pink = Mechanical, blue = Electrical.
   - **Count the shapes** of the zone's colour only, using OpenCV contours.
   - **Identify and count objects** (e.g. light bulbs) with a **CNN**.
6. Display: requested zone, detected zone, counts, match or mismatch, and an **honestly described** confidence (Section 6.5).
7. Camera processing at roughly **3–5 FPS on resized images, with a justification**.
8. **Bonus:** services/actions instead of only topics; a **custom-trained CNN**; an open-ended extra.

---

## 2. WSL2 setup (do this first; nothing else works without it)

### 2.1 Why WSL needs special setup
ROS 2 machines find each other with DDS, which uses **UDP multicast** discovery. By default WSL2 sits behind a virtual network (NAT) inside Windows. Multicast doesn't cross it, and the robot can't reach WSL directly. **Symptom:** `ssh` and `ping` work, but `ros2 topic list` doesn't show the robot's `/scan` or `/odom`.

### 2.2 Fix: mirrored networking (Windows 11 22H2 or later)
In `C:\Users\<me>\.wslconfig` on Windows:
```ini
[wsl2]
networkingMode=mirrored
```
In an **admin PowerShell**, allow inbound traffic to WSL:
```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```
Then restart WSL:
```powershell
wsl --shutdown
```
Check that `ip a` inside WSL now shows the same `192.168.0.x` address as Windows when the laptop is on CSLAB.

### 2.3 Laptop `~/.bashrc` (WSL)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=<same as robot>   # read it from the Pi
export ROBOT_IP=192.168.0.37
# NOT ROS_LOCALHOST_ONLY=1 when using the robot (only for pure Gazebo work at home)
```

### 2.4 Connectivity test (do before any project work)
```bash
ping -c 3 $ROBOT_IP
```
```bash
ssh ubuntu@$ROBOT_IP 'grep ROS_DOMAIN_ID ~/.bashrc; echo $ROS_DISTRO'
```
Then, in an SSH session on the Pi:
```bash
ros2 run demo_nodes_cpp talker
```
In WSL:
```bash
ros2 run demo_nodes_cpp listener      # must print "I heard: Hello World ..."
```
If the listener hears nothing: check the domain IDs, Windows Firewall (try briefly allowing UDP for the private network), and that mirrored mode is actually on. **Fallback:** use a lab PC (credentials are in the course manual).

### 2.5 Clock sync (a common cause of Nav2 failures)
Compare `date` on the laptop and on the Pi. If they're more than a few hundred ms apart, Nav2 and TF throw `extrapolation into the future` / `transform timeout` errors. Ask the tutor how the lab robots get their time; don't change the Pi's configuration yourself. Section 6.1 shows how my vision code avoids depending on the Pi's timestamps.

### 2.6 Packages to install in WSL
```bash
sudo apt update && sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot3* ros-humble-cartographer-ros \
  ros-humble-image-transport-plugins ros-humble-rqt-image-view \
  ros-humble-v4l2-camera xterm python3-opencv
```
```bash
pip install --user "numpy<2"
```
**Keep `numpy<2`.** Humble's Python packages (cv_bridge and others) are built against NumPy 1.x. `pip install torch` or `ultralytics` can silently upgrade NumPy to 2.x and break ROS Python nodes with confusing import errors.

---

## 3. Bringing up the real robot (each session)

**Pi, SSH terminal 1:**
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Pi, SSH terminal 2 (camera).** Shrink the image **at the source**:
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[320,240]"
```
**Why shrink at the source:** raw 640×480 RGB at 30 FPS is about **27 MB/s** over shared Wi-Fi. That chokes DDS, and `/scan` and TF start arriving late, so **navigation breaks because of the camera**. At 320×240 the camera stream is a quarter of that. If `/image_raw/compressed` exists (`ros2 topic list | grep compressed`), **subscribe to the compressed topic** on the laptop, since JPEG is about 10–20× smaller. Check `ros2 param list /v4l2_camera` for a frame-rate parameter (`time_per_frame`) to lower the FPS too.

**Laptop:**
```bash
ros2 topic list                 # expect /scan /odom /tf /image_raw ...
```
```bash
ros2 topic hz /scan             # should be ~5 Hz steady
```
```bash
ros2 run rqt_image_view rqt_image_view
```

---

## 4. Mapping (do once per arena)

1. **Mark the robot's start spot and heading with tape on the floor.** Cartographer makes the start pose the map origin `(0, 0, 0°)`. If the robot always starts on the tape, its initial pose on the map is known, so my code can set it automatically (Section 6.3) and I don't have to click 2D Pose Estimate in the demo.
2. Run Cartographer (laptop):
   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py
   ```
3. Drive (laptop):
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```
   Drive slowly, turn slowly, cover every aisle, and return to the start to close the loop.
4. Save the map **while Cartographer is still running**:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/construction_robot/maps/site_map
   ```
5. Check the `.pgm` image: walls should be crisp and there should be no doubled walls. Remap if it's bad. A bad map ruins everything after it.

**LiDAR blind spot:** the LDS scans one horizontal plane near the top of the Burger. Obstacles **lower than that plane** don't appear in the map or the costmaps, and the robot will hit them. Tell the tutor if the demo obstacles are very low. In that case, enlarge the inflation radius and test it.

---

## 5. System architecture

### 5.1 Packages
```
~/ros2_ws/src/
├── construction_robot_interfaces/      # ament_cmake: custom action + service (bonus 17a)
│   ├── action/GoToZone.action
│   └── srv/VerifyZone.srv
└── construction_robot/                 # ament_python: all nodes
    ├── construction_robot/
    │   ├── destination_selector.py     # REQUIRED NAME. UI: menu + action client
    │   ├── mission_controller.py       # action server, Nav2 client, state machine
    │   ├── sheet_detector.py           # red/green A4 detection → /sheet_signal
    │   ├── zone_verifier.py            # service: zone colour + shape count + CNN objects
    │   ├── vision_utils.py             # pure functions (HSV masks, shapes); unit-testable
    │   └── object_classifier.py        # loads ONNX CNN with cv2.dnn
    ├── launch/construction_robot.launch.py   # REQUIRED NAME
    ├── config/
    │   ├── zones.yaml                  # zone goal poses + colour + expected item types
    │   ├── vision.yaml                 # HSV ranges, area thresholds, rates
    │   └── mission.yaml                # settle time, initial pose, timeouts
    ├── maps/site_map.{yaml,pgm}
    ├── models/object_cnn.onnx, labels.txt
    ├── tools/
    │   ├── hsv_tuner.py                # trackbars on a bag/video/photos → prints ranges
    │   └── record_zone.py              # saves current map pose as a zone into zones.yaml
    ├── training/                       # CNN training + ONNX export (run outside ROS)
    └── test/test_vision_utils.py       # pytest on saved frames
```

### 5.2 Nodes and channels (put this table and an `rqt_graph` screenshot in the report)

```
 destination_selector ──(action /go_to_zone: GoToZone)──► mission_controller
        ▲   feedback: state, distance          │  ▲
        └───── result: requested/detected/     │  │ /sheet_signal (std_msgs/String, latched)
               counts/match/confidence         │  │
                                               │  sheet_detector ◄── /image_raw[/compressed]
                 (srv /sheet_detector/enable: std_srvs/SetBool) ──►  (on only while driving)
                                               │
                     (action /navigate_to_pose)▼
                                             Nav2 (bt_navigator, planner, controller, AMCL)
                                               │
                     (srv /verify_zone: VerifyZone)
 mission_controller ────────────────────────────► zone_verifier ◄── /image_raw[/compressed]
```

| Channel | Type | Why this type |
|---|---|---|
| `/go_to_zone` | **Action** (custom) | Long-running task that needs **feedback** (state, distance), a **final result** and **cancel**. That's exactly what an action is for. |
| `/navigate_to_pose` | Action (Nav2) | Same reason; it's Nav2's interface. |
| `/verify_zone` | **Service** (custom) | One request ("check zone X") → one answer, after a few seconds of frame analysis. |
| `/sheet_detector/enable` | Service (`std_srvs/SetBool`) | Switch the detector on only while driving. This saves CPU and Wi-Fi and stops pink zone markers from causing a false "red" during verification. |
| `/sheet_signal` | Topic (String: `RED`/`GREEN`/`NONE`), **transient-local** QoS | Continuous state that changes over time. A latched topic means a late subscriber still gets the current state. |

**Why not one big script:** the required ≥3 nodes come for free; each node can be tested alone (bag replay, or fake `/sheet_signal` messages); and the report's "purpose of each node and channel" section practically writes itself.

### 5.3 Interfaces
`GoToZone.action`
```
string zone                 # "material_storage" | "mechanical" | "electrical"
---
bool   reached
string requested_zone
string detected_zone        # or "unknown"
bool   match
string[] count_labels       # e.g. ["triangle","circle","light_bulb"]
int32[]  count_values
string confidence_text      # human-readable, honestly described (see 6.5)
string message
---
string state                # IDLE|NAVIGATING|PAUSED_RED|SETTLING|VERIFYING
float32 distance_remaining
int32  recoveries
```
`VerifyZone.srv`
```
string requested_zone
---
string   detected_zone
bool     match
string[] count_labels
int32[]  count_values
float32  colour_area_share   # fraction of zone-coloured pixels that belong to the winning colour
int32    frames_agreeing
int32    frames_used
string   summary
```

---

## 6. Node designs (the reasoning matters as much as the code)

### 6.1 Shared camera-handling pattern (sheet_detector and zone_verifier)
- Subscribe with `qos_profile_sensor_data` (best-effort). That matches the camera publisher, and a lost frame doesn't stall anything.
- Prefer **`sensor_msgs/CompressedImage`** and decode with `cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)`. That avoids cv_bridge and NumPy version problems. Make the topic and type a parameter (`camera_topic`, `use_compressed`) so the same node works with Gazebo (`/camera/image_raw` on `waffle_pi`).
- **Latest-frame-only processing:** the subscription callback just stores the newest message and the **laptop's receive time**. A separate **timer at `process_hz` (default 4 Hz)** processes only that newest frame. Old frames are dropped, never queued, so the robot always reacts to *now*, not to a backlog.
- Use the **receive time on the laptop** to decide whether a frame is fresh, not `header.stamp`, because the Pi's clock may be off (Section 2.5).
- Resize before processing: **160×120 for the sheet detector** (an A4 sheet held in front fills a large part of the frame, so fine detail isn't needed) and **320×240 for the zone verifier** (small shapes need more pixels).

**Justification for 4 Hz with a 3-of-4 debounce (put this in the report):**
- Burger top speed is about 0.22 m/s.
- With 3 of the last 4 frames required, the worst-case reaction time is about 4 frames ÷ 4 Hz = 1.0 s, so the robot travels about **22 cm** before it reacts. That's acceptable for a person holding up a sheet.
- Compared with 15 FPS at 640×480, the pixels processed per second fall by about 4 × 16 = 64×, which leaves CPU free for Nav2 on the same laptop.
- Measure the actual processing time per frame and the CPU use (`top`) and report the real numbers.

### 6.2 `sheet_detector.py`: red/green A4 detection
**Algorithm for each processed frame:**
1. BGR → HSV, then a light blur.
2. Masks from `vision.yaml`:
   - **Red** wraps around the hue circle, so it needs **two ranges** OR-ed together: H 0–10 and H 170–180 (OpenCV's H runs 0–180), with high S and V minimums.
   - **Green:** H about 40–85.
3. Clean up with a morphological open then close.
4. Take the largest contour of each colour. It counts as a **sheet candidate** only if:
   - its area is ≥ `min_area_frac` of the frame (start around 0.08–0.15; tune this so a sheet held about 0.5–1 m away triggers but a small red object doesn't), **and**
   - it's sheet-shaped: **solidity** (area / convex-hull area) ≥ 0.85, and the bounding-box aspect ratio is within a band around A4's 1.41 in either orientation. Don't require exactly 4 corners from `approxPolyDP`, because a hand covering a corner breaks that test.
5. The frame's label is `RED`, `GREEN` or `NONE`.

**Debounce ("consistently", as the spec says):** keep a deque of the last N labels (N = 4). Publish a **state change** only when ≥ K (K = 3) of them agree. Publish on change only, with transient-local QoS.

**Pitfall: pink zone markers can look red** (hue near 160–175 and lower saturation). Defences:
- a high saturation floor for red,
- the large-area and sheet-shape requirement,
- the detector is **disabled during verification** via the enable service.

Tune with real lab frames (Section 9, milestone M3).

### 6.3 `mission_controller.py`: state machine and Nav2
```
IDLE ──goal from selector──► NAVIGATING ──RED──► PAUSED_RED ──GREEN──► NAVIGATING (same goal re-sent)
                                │  ▲                     (RED is LATCHED: losing sight of red
                                │  └─ Nav2 recovering       does NOT resume; only GREEN does)
                     Nav2 SUCCEEDED
                                ▼
                            SETTLING (sleep settle_s, e.g. 1.5 s; detector disabled)
                                ▼
                            VERIFYING ──call /verify_zone──► DONE → action result to selector
            Nav2 FAILED (not caused by us) → DONE with reached=false + reason
```
**How "stop" works:**
- Nav2 has no "pause", so on RED the controller **cancels** the current NavigateToPose goal and publishes a zero `Twist` on `/cmd_vel` a couple of times for an immediate stop.
- On GREEN it **sends the same goal pose again**. Nav2 replans from wherever the robot is, which counts as "resume to the same destination".
- **Remember why the goal was cancelled** (a `paused_by_sheet` flag). Otherwise the CANCELED result from Nav2 looks like a failure.

**Other design points:**
- **Use `rclpy.action.ActionClient` for NavigateToPose directly, not BasicNavigator.** BasicNavigator spins its own internal node, which conflicts with an action server running in the same process (nested executors, deadlocks). Raw async action clients with callbacks are cleaner, and it's the Lab 5 material, which is good for the report.
- **Concurrency:** use a `MultiThreadedExecutor` with a `ReentrantCallbackGroup`. **Never** call `spin_until_future_complete` inside a callback. In the action server's execute callback, wait on `threading.Event`s set by other callbacks, or make the execute callback `async` and `await` futures.
- **Initial pose:** if `mission.yaml` has `auto_initial_pose: true`, publish `/initialpose` with the taped start pose (usually `(0, 0, 0)`) about 2 s after AMCL is active. Otherwise document using RViz 2D Pose Estimate.
- **Zone goals come from `zones.yaml`**, each with `x`, `y` and `yaw_deg`:
  - pick a viewing pose **about 0.6–0.8 m in front of the zone's markers, facing them**, so the camera sees all the markers straight on (straight on means fewer perspective errors in the shape counts);
  - use `tools/record_zone.py` to drive there with teleop and save the current `map → base_link` pose (same idea as `ros2 run tf2_ros tf2_echo map base_link`);
  - **re-recording zones takes about 2 minutes on demo day** if the environment changes.

  Storing destination poses is allowed. The **route** is planned by Nav2, which is what "not hard-coded" means.
- **If verification sees no zone colour:** optionally ask Nav2's Spin behaviour to rotate ±20° and verify again before giving up. Nav2 stops within its goal tolerance (about ±0.25 m / ±14°), so the markers may sit slightly out of view.
- **Feedback** to the selector at about 2 Hz: state, distance remaining, number of recoveries.

### 6.4 `zone_verifier.py`: zone identity, shape counts, objects
**Service `/verify_zone`:**
1. On request, note `t0` = now. Collect **M fresh frames** (e.g. M = 7) whose laptop receive time is after `t0`. That's the "fresh frames after settling" requirement.
2. **Zone identity (OpenCV only):**
   - For each zone colour (yellow, pink, blue; ranges in `vision.yaml`), build a cleaned HSV mask and take its pixel area.
   - **Winner** = largest area, provided it's above `min_zone_area_frac`; otherwise `unknown`.
   - `colour_area_share` = winner area ÷ sum of the three colour areas.
3. **Shape counting, only inside the winning colour's mask** (requirement 13):
   - `findContours` → drop tiny contours (`min_shape_area_px`) and **drop contours touching the image border**, since a cut-off shape can't be classified or counted reliably.
   - For each contour: `approxPolyDP(c, 0.03·perimeter)` gives the vertex count, and circularity = `4π·area / perimeter²`.
     - 3 vertices → triangle
     - 4 vertices → square or rectangle (aspect ratio of `minAreaRect`, with a tolerance band for perspective)
     - 5 → pentagon
     - circularity > ~0.8 with many vertices → circle
   - Map these to whichever shape set the arena actually uses. **Ask me for photos of each zone.**
4. **Objects (CNN):** for zones where objects must be identified (e.g. light bulbs), see 6.6.
5. **Combine the M frames:**
   - detected zone = the **mode** of the per-frame winners;
   - each count = the **mode** of the per-frame counts;
   - `frames_agreeing` = frames that match the mode.

   A median or mode across frames removes one-frame glitches such as motion blur or reflections.
6. `match = (detected_zone == requested_zone)`. Return everything, and log a clear summary block.

**Things I must give you:** photos or bag frames of each zone, the exact shapes/objects in each zone, and what "count as required" means per zone. The dataset link in the brief is a hyperlink I'll paste in.

### 6.5 Being honest about confidence (requirement 15, easy marks to lose)
- `colour_area_share` is a **relative colour dominance ratio**, not a probability. Name it that.
- `frames_agreeing / frames_used` is **agreement across frames** (e.g. "6/7 frames agree"), not a probability.
- CNN softmax outputs are **uncalibrated model scores**. Report them as "model score" unless the model was calibrated (e.g. temperature scaling on a validation set; that could count as bonus 17c).

Example display:
```
================ ZONE VERIFICATION ================
Requested zone : Electrical (blue)
Detected zone  : Electrical (blue)   colour dominance 0.91 (relative pixel share, not a probability)
Frame agreement: 6/7 frames
Shapes (blue only): circle=3, triangle=2
Objects (CNN)  : light_bulb=4 (mean model score 0.88, uncalibrated)
RESULT         : MATCH ✅
===================================================
```

### 6.6 CNN (pretrained model is the baseline; a custom model earns bonus 17b)
- **Approach:**
  1. **OpenCV proposes candidate regions** in the zone (contours from saturation/edge masks, filtered by size).
  2. The **CNN classifies each crop** (e.g. 64×64) into the object classes **plus a `background/other` class**. Without a background class, every random crop is forced into some object class and the counts inflate.
  3. Count by class with a score threshold, applying non-maximum suppression to overlapping boxes.
- **Custom CNN:** a small 3–4 block conv net in PyTorch, with augmentation (brightness, blur, rotation, scale) to match the robot camera's poor image quality. **Add crops taken with the robot's own camera** to the dataset. That's the biggest accuracy gain, because public datasets look nothing like a 320×240 webcam.
- **Train outside ROS** (WSL with CUDA if I have an NVIDIA GPU, otherwise Google Colab). **Export to ONNX** and run it in the node with `cv2.dnn.readNetFromONNX`, so the ROS node doesn't need torch (no NumPy 2 trouble and a light runtime).
- Report: dataset sizes, train/val split, accuracy, confusion matrix, and how the model was exported and run.
- Pretrained alternative: fine-tune MobileNet or YOLOv8n. Note whether it was used as a baseline compared with the custom net. A comparison table is good report material.

### 6.7 `destination_selector.py`
- **Terminal menu:** 1 = Material Storage, 2 = Mechanical, 3 = Electrical, q = quit. The names map to keys in `zones.yaml`.
- Sends a `GoToZone` action goal, prints feedback (`NAVIGATING 2.31 m`, `PAUSED_RED — show GREEN to resume`), then prints the result block. Ctrl+C or a menu key **cancels the action**, and the controller cancels Nav2, so the robot stops.
- **`input()` blocks**, so run the ROS executor in a **background thread** and keep the menu in the main thread. Otherwise feedback callbacks never run while waiting for input.
- **`ros2 launch` doesn't give nodes a keyboard (no stdin).** So in the launch file, start this node with `prefix='xterm -e'` (xterm works through WSLg) so it opens in its own window. As a backup, document running it with `ros2 run construction_robot destination_selector` in a separate terminal.

---

## 7. Launch file `construction_robot.launch.py` (outline)
- Launch arguments:
  - `map` (default: package's `maps/site_map.yaml`)
  - `use_sim_time` (default false)
  - `camera_topic`
  - `use_compressed`
- **Include** `turtlebot3_navigation2/launch/navigation2.launch.py` with `map` and `use_sim_time`. It starts Nav2, AMCL and RViz with the Burger's parameters.
- Start **4 nodes** (requirement ≥ 3): `mission_controller`, `sheet_detector`, `zone_verifier`, and `destination_selector` (with `prefix='xterm -e'`).
- Load parameters from `config/*.yaml`, located with `FindPackageShare` (no absolute paths).
- `setup.py` must install `launch/`, `config/`, `maps/` and `models/` into `share/` via `data_files`, plus `console_scripts` entry points for each node.
- Nothing moves after launch until a zone is selected (requirement 2). No node sends a goal at startup.

Demo run:
```bash
# Pi terminal 1
ros2 launch turtlebot3_bringup robot.launch.py
```
```bash
# Pi terminal 2
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[320,240]"
```
```bash
# Laptop (robot on its tape mark)
ros2 launch construction_robot construction_robot.launch.py
```

---

## 8. Testing at home without the robot
- **Record lab data once:**
  ```bash
  ros2 bag record -o lab_cam /image_raw/compressed /scan /tf /tf_static /odom
  ```
  Record red and green sheets at different distances and angles, partly covered by a hand, each zone's view, and a pink marker close up. Replay at home with `ros2 bag play lab_cam --loop` to tune HSV and test both vision nodes.
- **Gazebo for navigation logic:** `TURTLEBOT3_MODEL=waffle_pi` (it has a camera; the Burger doesn't), with `use_sim_time:=true` and `ROS_LOCALHOST_ONLY=1`.
- **Fake the sheet** to test the state machine without a camera:
  ```bash
  ros2 topic pub --once /sheet_signal std_msgs/msg/String "{data: RED}" --qos-durability transient_local
  ```
  Then publish `GREEN` the same way.
- **pytest** on saved frames for `vision_utils.py` (known image → expected zone and counts). That's solid evidence for the report.

---

## 9. Milestones (do them in order; each ends with a check)
| # | Milestone | Done when |
|---|---|---|
| M0 | WSL networking + domain ID | `listener` in WSL hears the Pi's `talker` |
| M1 | Bring-up, teleop, camera | `/scan` ~5 Hz; image in rqt_image_view at 320×240 |
| M2 | Map arena + tape the start spot | Clean `site_map.pgm` saved in the package |
| M3 | Record bag + tune HSV with `hsv_tuner.py` | Ranges in `vision.yaml` separate red, pink, green, yellow, blue on bag frames |
| M4 | Packages + interfaces build | `colcon build` OK; `ros2 interface show construction_robot_interfaces/action/GoToZone` works |
| M5 | `sheet_detector` | On the bag: RED/GREEN change within about 1 s, no false RED on pink markers |
| M6 | `zone_verifier` + `vision_utils` + tests | Correct zone and counts on saved frames; pytest passes |
| M7 | `mission_controller` + `destination_selector` | Gazebo: select → drive; faked RED stops; GREEN resumes same goal; result printed |
| M8 | Zone poses recorded on the real map | `zones.yaml` filled with `record_zone.py` |
| M9 | CNN trained + ONNX + `object_classifier` | Validation accuracy reported; counts correct on lab frames |
| M10 | Full real-robot run with the launch file | All 10 demo steps work 3 times in a row |
| M11 | Report evidence + video | Checklist in Section 11 complete |

---

## 10. Troubleshooting
| Symptom | Likely cause → fix |
|---|---|
| `ros2 topic list` in WSL lacks robot topics | NAT networking, domain-ID mismatch, firewall → Section 2 |
| Nav2 TF "extrapolation into the future" | Laptop/Pi clocks differ → Section 2.5 |
| Navigation gets jerky once the camera starts | Wi-Fi saturated by raw images → smaller `image_size`, compressed topic, lower FPS |
| Robot "resumes" when the red sheet is lowered | RED isn't latched → only GREEN may resume |
| False RED at the pink zone | Detector not disabled during verify; red saturation floor too low |
| Counts vary between runs | Not facing the markers (re-record the zone pose); use mode across frames; border-touching contours not filtered |
| `destination_selector` ignores keyboard under launch | No stdin under launch → `prefix='xterm -e'` or `ros2 run` |
| Node hangs after a service/action call | `spin_until_future_complete` inside a callback → events/async + MultiThreadedExecutor |
| `ImportError`/`numpy` errors in ROS nodes after pip installs | NumPy 2 installed → `pip install --user "numpy<2"` |
| Robot hits a low obstacle | Below the LiDAR plane → Section 4 note |

---

## 11. Submission and report checklist
- [ ] ROS 2 package(s): nodes, `config/`, `launch/construction_robot.launch.py`, `maps/`, `models/` (trained ONNX + labels), the interfaces package, and the training scripts
- [ ] `destination_selector.py` present under that exact name
- [ ] Report includes:
  - [ ] every ROS 2 command used
  - [ ] purpose of each node; the topics/services/actions table (5.2) and an `rqt_graph` screenshot
  - [ ] navigation algorithm: AMCL localisation, global planner on the global costmap, local controller on the local costmap, recoveries, and the cancel/resend pause
  - [ ] red/green algorithm: HSV dual-range red, area and shape gating, K-of-N debounce, latched state, rate and resize justification with **measured** numbers
  - [ ] verification: colour dominance, shape classification rules, cross-frame mode, CNN pipeline and metrics, honest confidence wording
  - [ ] evidence screenshots for each requirement 1–17
- [ ] Demo video covering demo steps 1–10 in order
