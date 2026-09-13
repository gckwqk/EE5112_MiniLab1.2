# EE5112 Mini-Lab README (Group Submission)

> **Mini-lab schedule:** Released 31 Aug 2026 (Sun) · Due **4 Oct 2026** · Canvas Student Submission: Mini-Lab  
> **GAs:** Mr. Huang Dong (dong.huang@u.nus.edu); Mr. Ying Zhuohang (E1554788@u.nus.edu)
>
> This README is a **group document**, not an individual log. One README covers all five tasks. Keep it short and tied to your code — not a generic software manual.

**Group index:** 13  
**Zip name:** `minilab_group_13.zip`  
**ROS version:** ROS 2 Humble  
**OS / environment:** Ubuntu 22.04 (native), Gazebo Classic 11

| Name | Matriculation No. | Led task(s) |
|------|-------------------|-------------|
| **Goh Chian Kai** | **A0330123B** | Task 2; Task 1 & Task 5 with the group |
| **Mohammad Asif Bin Abdul Sahid** | **A0313732M** | Task 3; Task 1 & Task 5 with the group |
| **Tan Chew Miang Edwin** | **A0201867A** | Task 4; Task 1 & Task 5 with the group |

## 1. Dependencies

Implemented Task 2 uses ROS 2 Humble and Gazebo Classic 11. Required packages include:

- `gazebo_ros` / Gazebo Classic 11
- `gazebo_ros2_control`
- `ros2_control`, `controller_manager`, `ros2_controllers`
- `joint_state_broadcaster`
- `forward_command_controller`
- `robot_state_publisher`, `joint_state_publisher`, `tf2_ros`
- `xacro`
- RViz 2 and `rqt_image_view` for verification
- `ackermann_msgs` is recommended for the future `/ackermann_cmd` Task 3 interface
- Task 3 perception/planning dependencies and Task 4 STT dependencies are **not yet implemented/selected**

Task 2 installation:

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-forward-command-controller \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools \
  ros-humble-xacro \
  ros-humble-rqt-image-view \
  ros-humble-rviz2 \
  ros-humble-ackermann-msgs
```

---

## 2. How to launch (reproduce)

Current working package: `ee5112_vehicle`.

```bash
cd ~/EE5112_MiniLab/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ee5112_vehicle arena.launch.py
```

This starts the prescribed three-room Gazebo arena, publishes the Task 2 robot description, spawns the Ackermann vehicle at START `(0.55, 0.35, 0)`, loads `gazebo_ros2_control`, and starts the joint-state, steering and rear-wheel controllers.

Verification:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic list | grep -E "camera|scan"
```

**World / map files used for marking:**

- World: `ee5112_vehicle/worlds/arena.world`
- Map / occupancy: no occupancy-grid file currently required; layout is known a priori
- Specs: `ee5112_vehicle/config/MiniLab1.2_platform_specs_5112.json`
- Generator: `ee5112_vehicle/scripts/generate_arena.py`

**Main launch file:** `ee5112_vehicle/launch/arena.launch.py` — currently starts the arena, Task 2 vehicle, robot-state publisher, camera/LiDAR and low-level controllers. Task 3 colour/mission nodes and Task 4 STT still need integration.

**Task 3 typed command:** **Pending Student B.** Final system must accept English commands such as `find red and blue`, start autonomous motion immediately, and reject non-English commands.

**Task 4 speech:** **Pending Student C.** Intended interface: `/speech_command` (`std_msgs/String`) into the same Task 3 mission logic.

---

## 3. Code structure

```text
ros2_ws/src/ee5112_vehicle/
├── config/
│   ├── controllers.yaml
│   └── MiniLab1.2_platform_specs_5112.json
├── ee5112_vehicle/__init__.py
├── launch/
│   ├── arena.launch.py
│   └── display.launch.py
├── resource/ee5112_vehicle
├── scripts/generate_arena.py
├── urdf/vehicle.urdf.xacro
├── worlds/arena.world
├── package.xml
├── setup.cfg
└── setup.py
```

| Path | Role | Owner |
|------|------|-------|
| `urdf/vehicle.urdf.xacro` | Task 2 chassis, wheels, steering, camera, LiDAR and `ros2_control` | Student A |
| `config/controllers.yaml` | Joint-state, steering-position and rear-wheel-velocity controllers | Student A |
| `worlds/arena.world` | 3-room arena + 7 coloured blocks | Student A |
| `scripts/generate_arena.py` | Rebuild arena from supplied JSON | Student A |
| `launch/arena.launch.py` | Launch arena and spawn vehicle at START | Student A |
| `launch/display.launch.py` | URDF/TF/RViz inspection | Student A |
| **[Task 3 colour node: pending]** | Colour detection → `/detected_colours` | Student B |
| **[Task 3 controller/planner: pending]** | Autonomous motion; intended `/ackermann_cmd` | Student B |
| **[Task 3 mission/parser: pending]** | Typed command/search; intended `/mission_status` | Student B |
| **[Task 4 STT node: pending]** | STT → `/speech_command` → Task 3 | Student C |

---

## 4. TF tree screenshot (required for Task 2 / Task 5)

`base_link` → `camera_link` and `base_link` → `laser_link` are fixed joints.

- Camera: translation `(0.14, 0.00, 0.08) m`, RPY `(0,0,0)`.
- LiDAR: translation `(0.00, 0.00, 0.18) m`, RPY `(0,0,0)`.

TF has been verified with TF2 tools.

![TF tree](figures/tf_tree.png)

> **TODO before submission:** add the final TF-tree screenshot at `figures/tf_tree.png`.

---

## 5. Task 1 — Four-wheel-drive locomotion modes and scenarios (ALL)

**Status:** **Group write-up still required.**

The implemented Task 2 platform uses Ackermann steering: front wheels steer and rear wheels provide propulsion. It has car-like motion with a finite turning radius rather than in-place rotation, so door approaches and corridor manoeuvres must respect the wheelbase, track and ±35° steering limit.

**TODO:** expand this section to about one page comparing Ackermann with at least two other relevant modes, including kinematic idea, command inputs and suitable scenarios.

**References used for Task 1:**

- **[TODO: add at least two references actually used by the group.]**

No implementation is required for Task 1.

---

## 6. Task 2 — How the vehicle and the map is modelled

**What was done:**  
We modelled the required rectangular four-wheel vehicle in Xacro with an Ackermann-style front-steer/rear-drive architecture. The chassis is `0.30 × 0.20 × 0.12 m`, with `0.20 m` wheelbase, `0.16 m` track, `0.04 m` wheel radius, `0.03 m` wheel width, maximum steering magnitude `35°`, and required maximum vehicle speed `0.50 m/s`. The front steering joints use position command interfaces and the rear wheels use velocity command interfaces through `gazebo_ros2_control`; the front rolling joints are free rolling. The prescribed RGB camera and 2D LiDAR are rigidly attached and verified in Gazebo. The three-room arena and seven coloured blocks are generated from the supplied JSON, and the vehicle spawns at START `(0.55, 0.35, 0)`. This exact vehicle/map is to be reused by Tasks 3 and 4.

**Key files:**

- Model / URDF: `ee5112_vehicle/urdf/vehicle.urdf.xacro`
- Controllers: `ee5112_vehicle/config/controllers.yaml`
- World / map: `ee5112_vehicle/worlds/arena.world`
- Specs: `ee5112_vehicle/config/MiniLab1.2_platform_specs_5112.json`
- Arena generator: `ee5112_vehicle/scripts/generate_arena.py`
- Launch / spawn: `ee5112_vehicle/launch/arena.launch.py`
- Owner: Student A **[add name/matriculation number]**

**Implemented platform:**

- Front-wheel steering / rear-wheel drive
- Steering limit: `±35°`
- Camera TF: `(0.14, 0.00, 0.08)`, RPY `(0,0,0)`
- LiDAR TF: `(0.00, 0.00, 0.18)`, RPY `(0,0,0)`
- Gazebo RGB camera: image and camera-info streams verified
- Gazebo 2D LiDAR: `/scan` verified
- Arena: `4.20 × 2.60 m`, three rooms + corridor
- Seven static `0.08 m` coloured cubes using the mandatory RGBA values
- START: `(0.55, 0.35, 0)`

**Key snippet:**

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="front_left_steering_joint">
    <command_interface name="position"/>
  </joint>
  <joint name="rear_left_wheel_joint">
    <command_interface name="velocity"/>
  </joint>
</ros2_control>
```

**Verification performed:**

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic list | grep -E "camera|scan"
ros2 topic echo /scan --once
ros2 run rqt_image_view rqt_image_view
```

Verified controllers: `joint_state_broadcaster`, `steering_controller`, `rear_wheel_controller` — all active. Verified command interfaces are both front steering `position` interfaces and both rear wheel `velocity` interfaces. Open-loop tests verified straight propulsion, left/right steering and combined curved motion.

**Deliverables:**

- Modelling screenshot: **[TODO: add final path, e.g. `figures/task2_vehicle.png`]**
- TF tree: **[TODO: `figures/tf_tree.png`]**
- Video: `Video_Task2.[mp4/mkv/…]` — **[TODO: record arena + vehicle + visible camera/LiDAR + basic motion]**

---

## 7. Task 3 — How Student B searched for coloured blocks

**Vehicle:** reuse the exact Task 2 URDF, arena, kinematics and sensor mounts.  
**Commands:** English only.  
**Navigation:** autonomous immediately after a valid command; no teleoperation/manual RViz goal in the graded video.

**Current status:** **Pending Student B implementation.** Task 2 has delivered the simulation platform, camera, `/scan`, known arena, seven blocks and low-level steering/drive controllers.

**Integration information:**

- Launch: `ros2 launch ee5112_vehicle arena.launch.py`
- Camera: verified; inspect exact topic with `ros2 topic list | grep camera`
- LiDAR: `/scan` (`sensor_msgs/msg/LaserScan`)
- Steering: `/steering_controller/commands` (`std_msgs/msg/Float64MultiArray`)
- Rear drive: `/rear_wheel_controller/commands` (`std_msgs/msg/Float64MultiArray`)
- Limits: `|v| ≤ 0.50 m/s`, `|δ| ≤ 35°`
- Recommended high-level command: `/ackermann_cmd` (`ackermann_msgs/AckermannDrive`)
- Known map: `config/MiniLab1.2_platform_specs_5112.json`
- START: `(0.55, 0.35, 0)`
- SLAM not required

**Supported English commands** (final implementation must cover 1–4 distinct colours):

| N | Example to test |
|---|-----------------|
| 1 | `find red` |
| 2 | `find red and blue` |
| 3 | `find red, blue and yellow` |
| 4 | `find red, blue, yellow and green` |

**Key files:**

- Colour node: **[pending Student B]**
- Controller / planner: **[pending Student B]**
- Mission / parser: **[pending Student B]**
- Owner: Student B **[add name/matriculation number]**

```python
# Required logic:
# "find red and blue" -> ["Red", "Blue"]
# Found only if camera detects colour AND planar distance <= 0.50 m
# AND a [FOUND] line is printed.
```

**Deliverables:**

- `Video_Task3.*` — **pending**; typed English command + autonomous search + visible terminal `[CMD]` / `[FOUND]` / `[MISSION]`.

---

## 8. Task 4 — How Student C connected speech to Task 3

**Current status:** **Pending Student C implementation.** No STT library/API has yet been selected in the work completed so far.

The final node must perform English speech-to-text, print `[STT]` output, validate/parse the seven supported colours, and publish/pass the command through `/speech_command` (`std_msgs/String`) into the same Task 3 mission logic. The vehicle must then start the same autonomous search without another manual trigger.

**Key files:**

- STT node: **[pending Student C]**
- Interface into Task 3: **[pending Student C]**
- Owner: Student C **[add name/matriculation number; or state shared ownership for a 2-member group]**

```python
print("[STT] text=...")
print("[STT] colours=Red, Blue")
# Then pass the validated command to the Task 3 mission logic.
```

**Deliverables:**

- `Video_Task4.*` — **pending**; spoken English command + `[STT]` + Task 3 autonomous search + `[FOUND]`.

---

## 9. Task 5 — Videos and README checklist (ALL)

This section is marked for the **whole group**. Confirm every item before you zip.

| File | Linked task | Content |
|------|-------------|---------|
| `README.md` | 5 | **In progress:** Task 2 documented; add member details and final Tasks 1/3/4 write-ups |
| `Video_Task2.*` | 2 | **To record:** vehicle/map, visible camera/LiDAR and basic motion |
| `Video_Task3.*` | 3 | **Pending Task 3** |
| `Video_Task4.*` | 4 | **Pending Task 4** |

> In `Video_Task3` and `Video_Task4`, the terminal or rosout log **must remain visible throughout**. A video without that output is incomplete.

---

## Found criterion (course-defined — do not change)

A block is **found** only when all three hold at the same time:

1. **C1 Camera:** the onboard camera pipeline correctly labels that colour (do not use only the JSON ground-truth pose).
2. **C2 Proximity:** planar Euclidean distance from the `base_link` origin to the block centre is **≤ 0.50 m**.  
   \(d = \sqrt{(x_{\mathrm{base}}-x_{\mathrm{block}})^2 + (y_{\mathrm{base}}-y_{\mathrm{block}})^2}\)  
   The \(z\) coordinate is ignored.
3. **C3 Log:** print one line to the terminal/rosout, e.g.  
   `[FOUND] colour=Blue t=12.3s x=3.40 y=2.10`

Colours must be found **in the order named in the English command**.

---

## Block colours (mandatory Gazebo `<visual>` RGBA)

Put the **same** four numbers in both `<ambient>` and `<diffuse>`.

| Colour | `r g b a` |
|--------|-----------|
| Red | `0.90 0.10 0.10 1.00` |
| Orange | `0.95 0.50 0.05 1.00` |
| Yellow | `0.95 0.90 0.10 1.00` |
| Green | `0.10 0.75 0.15 1.00` |
| Blue | `0.15 0.25 0.90 1.00` |
| Purple | `0.55 0.15 0.70 1.00` |
| Black | `0.08 0.08 0.08 1.00` |

---

## Recommended topics (not marked, strongly recommended)

| Role | Topic | Type |
|------|-------|------|
| Speed / steering | `/ackermann_cmd` | `ackermann_msgs/AckermannDrive` |
| Colour detection (optional) | `/detected_colours` | `std_msgs/String` or custom |
| Speech command (Task 4 → Task 3) | `/speech_command` | `std_msgs/String` |
| Mission status (optional) | `/mission_status` | `std_msgs/String` |

---

## 10. Limitations

- Task 2 vehicle, controllers, RGB camera, 2D LiDAR and prescribed arena are implemented and verified in Gazebo Classic.
- Task 3 colour detection, high-level Ackermann command node, planner/controller and autonomous mission parser are not yet integrated.
- Task 4 speech-to-text is not yet implemented.
- Camera throughput can fall below its nominal configured rate depending on Gazebo rendering/simulation load; perception performance should be rechecked with the final Task 3 stack.
- Ackermann steering cannot rotate in place; door/corridor trajectories must respect the wheelbase and ±35° steering limit.
- Colour-threshold/lighting limitations and STT noise/accent limitations must be documented after Tasks 3 and 4 are implemented.

---

## 11. References / third-party code

- EE5112 Mini-Lab specification and `MiniLab1.2_platform_specs_5112.json` — authoritative vehicle dimensions, sensor transforms, arena geometry, block positions/colours and START pose.
- ROS 2 Humble `ros2_control` / `ros2_controllers` — simulated steering and rear-wheel control.
- `gazebo_ros` and `gazebo_ros2_control` — Gazebo Classic integration and simulated hardware.
- `robot_state_publisher`, Xacro and TF2 — robot description and TF publication/verification.
- **[TODO Task 1: add at least two locomotion/kinematics references actually consulted.]**
- **[TODO Tasks 3/4: add third-party perception/planning/STT packages or APIs actually used.]**
