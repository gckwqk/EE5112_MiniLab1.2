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
| `urdf/vehicle.urdf.xacro` | Task 2 chassis, wheels, steering, camera, LiDAR and `ros2_control` | Goh Chian Kai |
| `config/controllers.yaml` | Joint-state, steering-position and rear-wheel-velocity controllers | Goh Chian Kai |
| `worlds/arena.world` | 3-room arena + 7 coloured blocks | Goh Chian Kai |
| `scripts/generate_arena.py` | Rebuild arena from supplied JSON | Goh Chian Kai |
| `launch/arena.launch.py` | Launch arena and spawn vehicle at START | Goh Chian Kai |
| `launch/display.launch.py` | URDF/TF/RViz inspection | Goh Chian Kai |
| **[Task 3 colour node: pending]** | Colour detection → `/detected_colours` | Mohammad Asif Bin Abdul Sahid |
| **[Task 3 controller/planner: pending]** | Autonomous motion; intended `/ackermann_cmd` | Mohammad Asif Bin Abdul Sahid |
| **[Task 3 mission/parser: pending]** | Typed command/search; intended `/mission_status` | Mohammad Asif Bin Abdul Sahid |
| **[Task 4 STT node: pending]** | STT → `/speech_command` → Task 3 | Tan Chew Miang Edwin |

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

The implemented Task 2 platform uses Ackermann steering: front wheels steer and rear wheels provide propulsion. It has a finite turning radius rather than a pivot-based rotation, so door approaches and corridor manoeuvres must respect the wheelbase, track and ±35° steering limit.

## Comparison of three wheeled locomotion modes

Three wheeled locomotion configurations relevant to a four-wheel drive vehicle are Ackermann steering, differential drive, and omnidirectional drive. They differ mainly in how steering is produced, the motion constraints imposed by the wheels, the commands required from the controller, and the environments in which they are most effective \[1\], \[2\], \[3\], \[4\].

## Ackermann steering

**Kinematic idea.** 
Ackermann steering is configuration in which the front wheels change their steering angle while the rear wheels provide propulsion. During a turn, the inner front wheel steers more sharply than the outer front wheel such that the wheel axes intersect at a common instantaneous centre of rotation, reducing lateral tyre scrubbing. 

A common low-speed approximation is the bicycle model,

\[ $\dot{x}$=v$\cos$$\theta$,$\qquad$
$\dot{y}$=v$\sin$$\theta$,$\qquad$
$\dot{\theta}$=$\frac{v}{L}$$\tan$$\delta$, \]

where (v) is longitudinal speed, ($\theta$) is heading, (L) is wheelbase, and ($\delta$) is the equivalent steering angle \[1\], \[3\]. The vehicle is non-holonomic: it cannot translate directly sideways or rotate in place.

**Typical command inputs.**
The natural high level inputs are forward and reverse speed (v) and steering angle ($\delta$). These high level inputs are then converted into steering-joint angles and driven-wheel velocities for control inputs. 

**Suitable applications.**
Ackermann steering is suitable for applications with low-to-moderate speed applications where rolling efficiency and tire wear matter more than high-speed cornering dynamics. Passenger cars and light trucks are the typical use case where the vehicles speed range minimizes tire scrubbing and allows for stable maneuvering. Mobile robotic platforms such as warehouse robots are suitable as well as it gives predictable and calculable turning radii, for ease of implementation of path-planning algorithm.

### Differential drive

**Kinematic idea.**
A differential-drive platform uses two independently powered wheels or tracks on a common axis, with steering achieved by varying the relative speed of each side rather than turning wheels. No separate steering mechanism is needed, and the vehicle can rotate in place (zero turning radius).

For an ideal two-wheel differential model,

\[ v=$\frac{v_R+v_L}{2}$,$\qquad$
$\omega$=$\frac{v_R-v_L}{W}$, \]

where (v_L) and (v_R) are the left/right wheel linear velocities and (W) is the track width \[1\]. Equal velocities produce straight motion; unequal velocities produce a turn; opposite velocities allow
approximately zero-radius rotation. 

Four-wheel skid-steer vehicles use the same principle but rely on lateral tyre slip during turning.

**Typical command inputs.**
The controller normally commands left and right wheel velocities ((v_L,v_R)), or equivalently a desired linear velocity (v) and yaw rate ($\omega$) that are converted into wheels angular velocity.

**Suitable applications.**
Differential drive is suitable for applications that operates in areas with space constraints due to its ability to rotate in place and the need for compactness due to mechanical simplicity. Mobile robots such as vacuum and warehouse robots are suitable as they are cheap and mechanically simple to implement, and allows for maneuvering in tight spaces. Track vehicles such as tanks are ideal as well as the differential drive provides excellent traction and ability to pivot in place, as they are often deployed in rough, soft or unstable terrains.

### Omnidirectional drive

**Kinematic idea.**
Omnidirectional drive allows a vehicle to translate in any directions. Each wheel consists of both longitudinal and lateral motion components. By coordinating all four wheel speeds, the platform can independently produce forward/backward velocity (v_x), lateral velocity (v_y), and yaw rate ($\omega$) \[2\], \[4\]. Unlike Ackermann and differential drive, an ideal omnidirectional platform is holonomic in planar motion and can translate sideways without first changing its heading.

**Typical command inputs.**
The high-level controller typically commands (v_x), (v_y), and ($\omega$). Inverse kinematics converts these three commands into the four individual wheel angular velocities.

**Suitable applications.**
Omnidirectional platforms are useful for warehouses, factories, mobile manipulators and parking/alignment tasks where precise lateral repositioning is valuable. Their disadvantages are greater mechanical/control complexity and increased sensitivity to roller contact, wheel slip and uneven or low-traction surfaces.

### Choice of Ackermann platform this MiniLab

The use front-wheel Ackermann steering with rear-wheel drive in this MiniLab makes navigation more constrained as compared to differential or omnidirectional drive as the robot is unable rotate in place or correct its position by moving sideways. For the implemented vehicle, the wheelbase is (L=0.20) m and the maximum steering angle is (35\^$\circ$). Therefore, when crossing door openings, considerations of suitable positions and headings must be taken into account to generate a feasible curved path rather than relying on an in-place rotation or lateral correction. This makes Ackermann steering a useful platform for demonstrating realistic non-holonomic path-planning and steering-control constraints.

**References used for Task 1:**

\[1\] R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, *Introduction to
Autonomous Mobile Robots*, 2nd ed. Cambridge, MA, USA: MIT Press, 2011.

\[2\] K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning,
and Control*. Cambridge, U.K.: Cambridge University Press, 2017.

\[3\] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed. New York, NY,
USA: Springer, 2012, doi: 10.1007/978-1-4614-1433-9.

\[4\] G. Wampfler, M. Salecker, and J. Wittenburg, "Kinematics,
dynamics, and control of omnidirectional vehicles with Mecanum wheels,"
*Mechanics of Structures and Machines*, vol. 17, no. 2, pp. 165--177,
1989, doi: 10.1080/15397738909412814.
---

## 6. Task 2 — How the vehicle and the map is modelled

**What was done:**  
The required rectangular four-wheel vehicle in Xacro with an Ackermann-style front-steer/rear-drive architecture was modelled. The chassis is `0.30 × 0.20 × 0.12 m`, with `0.20 m` wheelbase, `0.16 m` track, `0.04 m` wheel radius, `0.03 m` wheel width, maximum steering magnitude `35°`, and required maximum vehicle speed `0.50 m/s`. The front steering joints use position command interfaces and the rear wheels use velocity command interfaces through `gazebo_ros2_control`; the front rolling joints are free rolling. The prescribed RGB camera and 2D LiDAR are rigidly attached and verified in Gazebo. The three-room arena and seven coloured blocks are generated from the supplied JSON, and the vehicle spawns at START `(0.55, 0.35, 0)`. This exact vehicle/map will be reused in Tasks 3 and 4.

## Ackermann vehicle model

## States and inputs

For low-speed planar motion, the vehicle state is

\[ $\mathbf{x}$=\[x,;y,;$\theta$\]\^T, \]

where (x,y) are the planar position and ($\theta$) is yaw/heading.

The high-level control input is

\[ $\mathbf{u}$=\[v,;$\delta$\]\^T, \]

where (v) is longitudinal speed and ($\delta$) is the equivalent front steering angle. In Gazebo, these correspond to front-left/right steering position commands and rear-left/right wheel velocity commands. For straight rolling, rear-wheel angular speed is approximately ($\omega$\_w=v/r).

#### Kinematics

For low operating speed (max 0.50 m.s), the four-wheel vehicle is represented by the kinematic bicycle approximation.

Assuming pure rolling and negligible lateral slip,

\[ $\dot{x}$=v$\cos$$\theta$,$\qquad$
$\dot{y}$=v$\sin$$\theta$,$\qquad$
$\dot{\theta}$=$\frac{v}{L}$$\tan$$\delta$. \]

The centreline turning radius is

\[ R=$\frac{L}{\tan\delta}$. \]

With (L=0.20) m and ($\delta$\_{$\max$}=35\^$\circ$),

\[
R\_{$\min$}=$\frac{0.20}{\tan35^\circ}$$\approx0.286$$\text{ m}$.
\]

"Note: this R_min figure is the bicycle-model (single-track, centreline) result, obtained by treating δ_max = 35° as the equivalent centreline steering angle. If the physical front-wheel joints are each independently limited to ±35°, the inner wheel which must steer more sharply than the centreline angle in a turn, reaches its 35° limit first. Solving tan(35°) = L/(R − W/2) for R gives R_min ≈ 0.366 m as the true achievable minimum radius under a genuine per-wheel joint limit."

For ideal four-wheel Ackermann geometry, the inner and outer front wheels require different angles:

\[
$\tan$$\delta$*{$\mathrm{inner}$}=$\frac{L}{R-W/2}$,$\qquad$
$\tan$$\delta$*{$\mathrm{outer}$}=$\frac{L}{R+W/2}$.
\]

Hence the inner wheel steers more sharply than the outer wheel so that the wheel axes approximately meet at a common instantaneous centre of rotation.

## Dynamics / Gazebo physics

Gazebo does not move the robot by using the kinematic model. Instead, the Xacro defines physical masses and inertias, collision geometry, revolute/continuous joints, damping/friction, and wheel-ground contact
properties. `gazebo_ros2_control` applies steering-position and rear-wheel-velocity commands, while Gazebo's rigid-body/contact physics determines the realised motion. Thus the bicycle model is used for motion reasoning and controller design, while Gazebo captures non-ideal physical effects such as inertia, contact forces and wheel slip.

## Parameters

  Parameter                                         Symbol                  Value
  --------------------- ---------------------------------- ----------------------
  Chassis (L_c × W_c × H)                                  ---   0.30 × 0.20 × 0.12 m
  Wheelbase                                          \(L\)                 0.20 m
  Track width                                        \(W\)                 0.16 m
  Wheel radius                                       \(r\)                 0.04 m
  Wheel width                                          ---                 0.03 m
  Maximum steering        ($\delta$\_{$\max$})      ±35° (±0.611 rad)
  Maximum speed                        (v\_{$\max$})               0.50 m/s
  Steering                                             ---            Front-wheel
  Drive                                                ---             Rear-wheel

## Assumptions

1.  Motion is planar, and roll and pitch are neglected in the analytical
    model.
2.  Vehicle speed is low enough for the kinematic model to be appropriate.
3.  Ideal kinematics assume pure wheel rolling and negligible lateral slip.
4.  The chassis is rigid and the ground is locally level.
5.  The bicycle model replaces each axle by an equivalent centreline wheel.
6.  Steering/drive actuator response is assumed sufficiently fast for path-level modelling.
7.  Aerodynamic effects are neglected because of the small scale and low maximum speed.
8.  Inertia, damping, friction, contact forces and slip are handled by Gazebo rather than explicitly added to the analytical bicycle equations.

## Effect of vehicle geometry and physical parameters on motion

**Chassis size.**
The (0.30$\times0.20$$\times0.12$) m chassis determines the physical footprint that must clear walls, door frames and coloured blocks. Although the bicycle model often treats the vehicle as a point at its reference position, the planner must account for the complete rectangular footprint. A larger or wider chassis reduces clearance through narrow openings and increases the risk that a collision-free centreline trajectory is not collision-free for the actual body. The chassis dimensions therefore directly affect feasible doorway approaches and the safety margin required around obstacles.

**Wheelbase.**
The wheelbase (L=0.20) m directly affects curvature through

\[ R=$\frac{L}{\tan\delta}$. \]

For a fixed steering angle, increasing (L) increases the turning radius and produces a wider, less agile turn. A shorter wheelbase allows tighter turns but generally produces faster heading change for the same
speed and steering command. The MiniLab wheelbase must therefore be considered when generating paths between rooms and aligning the vehicle with door openings.

**Steering limit.**
The front steering joints are limited to ($\lvert$$\delta$$\rvert$$\leq35$\^$\circ$). This limits the maximum achievable curvature given by,

\[ $\kappa$\_{$\max$}=$\frac{\tan\delta_{\max}}{L}$,\]

and gives a bicycle-model centreline minimum turning radius of approximately 0.286 m (the corresponding minimum radius for the physical inner front wheel, accounting for the track width, is approximately 0.366 m). Commands requesting greater curvature are physically infeasible. Unlike a differential-drive or Mecanum robot, the
Ackermann vehicle cannot rotate in place or translate sideways, so a poor doorway approach cannot be corrected instantaneously. The planner/controller must begin turning early enough to enter an opening with an appropriate position and heading.

**Speed and dynamic parameters.**
The prescribed vehicle speed is limited to (0.50) m/s. At higher speed, the same steering angle produces a larger yaw rate magnitude given by,

\[ $\dot{\theta}$=$\frac{v}{L}$$\tan$$\delta$,
\]

so steering and path-tracking errors can develop more quickly. The Gazebo model also includes chassis/wheel masses and inertias, joint damping/friction, wheel-ground friction and contact properties. These parameters do not change the ideal geometric turning-radius equation, but they affect the transient response and realised trajectory. Greater mass/inertia resists rapid changes in motion; damping suppresses joint oscillation; and insufficient tyre-ground friction can cause wheel slip so that the simulated path departs from the ideal no-slip Ackermann model. Because the MiniLab operates at low speed, the kinematic model is used for path-level reasoning while Gazebo accounts for these physical effects.

**Key files:**

Owner: Goh Chian Kai **A0330123B**

- Model / URDF: `ee5112_vehicle/urdf/vehicle.urdf.xacro`
- Controllers: `ee5112_vehicle/config/controllers.yaml`
- World / map: `ee5112_vehicle/worlds/arena.world`
- Specs: `ee5112_vehicle/config/MiniLab1.2_platform_specs_5112.json`
- Arena generator: `ee5112_vehicle/scripts/generate_arena.py`
- Launch / spawn: `ee5112_vehicle/launch/arena.launch.py`

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

Verified controllers: `joint_state_broadcaster`, `steering_controller`, `rear_wheel_controller`
Verified command interfaces are both front steering `position` interfaces and both rear wheel `velocity` interfaces. Open-loop tests verified straight propulsion, left/right steering and combined curved motion.

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

------------------------------------------------------------------------

## 12. Task 2 --- Trajectory-validation additions

> **Additive update:** This section records the trajectory-validation
> work completed after the Task 2 material above. The existing README
> content has been retained unchanged.

### Additional Task 2 implementation

A controlled open-world trajectory-validation environment was added so
that the same verified Ackermann vehicle can be tested without
collisions with the walls of the prescribed three-room arena. The
prescribed `arena.world` remains the Task 2 map and the platform that
Tasks 3 and 4 must use; the open world is used only for controlled Task
2 motion-model experiments.

Additional files:

-   `ee5112_vehicle/worlds/trajectory_test.world` --- unobstructed
    Gazebo Classic world for controlled motion tests.
-   `ee5112_vehicle/launch/trajectory_test.launch.py` --- launches the
    verified vehicle in the open test world and preserves the working
    ROS 2 Humble/Gazebo Classic Xacro-processing and controller-startup
    sequence.
-   `ee5112_vehicle/ee5112_vehicle/trajectory_experiment.py` ---
    automated Task 2 trajectory experiment, data recorder and plot
    generator.
-   `task2_results/csv/` --- experiment CSV files and numerical
    summaries.
-   `task2_results/figures/` --- generated trajectory, heading, speed,
    steering, lateral-displacement and position-error plots.

The vehicle Xacro was extended with Gazebo ground-truth odometry using
`libgazebo_ros_p3d.so`. The resulting `/odom` topic publishes
`nav_msgs/msg/Odometry` in the `world` frame for recording the simulated
vehicle pose and velocity during Task 2 validation. This ground-truth
odometry is for model verification and does not replace the onboard
camera/LiDAR sensing required by later tasks.

### Automated trajectory experiments

  Experiment        Equivalent steering   Commanded speed   Duration
  --------------- --------------------- ----------------- ----------
  `straight`                    `0 deg`        `0.12 m/s`      `4 s`
  `gentle_left`               `+15 deg`        `0.12 m/s`      `5 s`
  `sharp_left`                `+30 deg`        `0.12 m/s`      `4 s`
  `right_turn`                `-20 deg`        `0.12 m/s`      `5 s`

For each experiment, the script converts the equivalent bicycle steering
angle into separate left/right Ackermann steering commands, commands the
rear-wheel velocity controller, records `/odom` and `/joint_states`, and
compares:

1.  **Command-input bicycle model:** bicycle equations using requested
    speed and equivalent steering.
2.  **Measured-input bicycle model:** the same equations using realised
    translational speed from Gazebo `/odom` and the commanded equivalent
    steering.
3.  **Gazebo ground truth:** recorded `/odom` pose.

The measured-input comparison is an **input-conditioned kinematic
validation**. It tests whether the bicycle kinematics reproduce pose
evolution once the realised Gazebo speed is supplied; it is not an
independent prediction of drivetrain speed.

### Trajectory-validation observations

The tests exposed a consistent difference between the nominal `0.12 m/s`
command and the lower realised Gazebo speed. `/odom` speed and
wheel-derived speed closely agree, showing that recorded vehicle motion
is consistent with measured wheel motion.

For straight motion, the measured-input bicycle trajectory almost
overlaps the Gazebo trajectory while the command-input model accumulates
longitudinal error. Heading and lateral displacement remain essentially
constant.

For gentle-left and right-turn motion, the measured-input model follows
Gazebo heading closely and produces much smaller position error than the
command-input model. Residual Cartesian differences remain because the
bicycle model is a single-track, no-slip approximation while Gazebo
simulates four wheel contacts and rigid-body/contact physics.

For sharp-left motion, the discrepancy increases. With an equivalent
`30 deg` bicycle command, ideal Ackermann conversion requires the inner
wheel to steer beyond the physical joint limit. The implementation
clamps each front steering joint to `+/-35 deg`, so the physical
steering geometry cannot realise the ideal equivalent command exactly.
This directly demonstrates the effect of the steering constraint on
achievable curvature.

For clean final comparisons, restart `trajectory_test.launch.py` before
each manoeuvre so all tests begin from approximately the same initial
pose.

### Additional Task 2 installation requirements

The trajectory experiment uses Matplotlib and ROS 2 odometry/message
packages. Install these if they are not already present:

``` bash
sudo apt update
sudo apt install \
  python3-matplotlib \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-gazebo-plugins
```

`ros-humble-gazebo-plugins` supplies the Gazebo Classic P3D plugin used
for `/odom`. The previously listed Gazebo, `gazebo_ros2_control`,
`ros2_control`, controller, Xacro, TF, RViz and camera-verification
dependencies remain required.

### Build and verify the updated package

``` bash
cd ~/EE5112_MiniLab/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select ee5112_vehicle --symlink-install
source install/setup.bash

ros2 pkg executables ee5112_vehicle
```

The package should include `ee5112_vehicle trajectory_experiment`.

The prescribed project arena remains:

``` bash
ros2 launch ee5112_vehicle arena.launch.py
```

Verify the main interfaces with:

``` bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic list -t | grep -E "odom|camera|scan"
```

### Run the controlled trajectory validation

Terminal 1:

``` bash
cd ~/EE5112_MiniLab/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ee5112_vehicle trajectory_test.launch.py
```

Verify before running:

``` bash
ros2 control list_controllers
ros2 topic echo /odom --once
```

Terminal 2:

``` bash
source /opt/ros/humble/setup.bash
source ~/EE5112_MiniLab/ros2_ws/install/setup.bash

ros2 run ee5112_vehicle trajectory_experiment --ros-args -p experiment:=straight
```

Other experiments:

``` bash
ros2 run ee5112_vehicle trajectory_experiment --ros-args -p experiment:=gentle_left
ros2 run ee5112_vehicle trajectory_experiment --ros-args -p experiment:=sharp_left
ros2 run ee5112_vehicle trajectory_experiment --ros-args -p experiment:=right_turn
```

Restart the trajectory-test simulation between final report runs if a
common initial pose is required.

### Generated Task 2 evidence

Each experiment writes a CSV file, text summary and six plots showing:

-   command-input bicycle model vs measured-input bicycle model vs
    Gazebo trajectory;
-   commanded, `/odom` and wheel-derived speed;
-   heading comparison;
-   command-input and measured-input position error;
-   equivalent bicycle and individual Ackermann steering angles;
-   lateral displacement.

Use the generated numerical summaries for final RMS, maximum and
final-error values rather than estimating values visually from plots.

### Updated Task 2 verification status

The following are now implemented and verified in addition to the
earlier Task 2 items:

-   `/odom` Gazebo ground-truth pose/velocity output.
-   Automated straight, gentle-left, sharp-left and right-turn
    manoeuvres.
-   Planned-versus-recorded trajectory plotting.
-   Commanded-versus-realised speed comparison.
-   Wheel-derived speed cross-check using `/joint_states`.
-   Heading and lateral-displacement comparison.
-   Quantitative command-input and measured-input trajectory-error
    calculation.
-   Demonstration of the `+/-35 deg` physical steering-joint limit
    during high-curvature motion.
-   Open trajectory-validation world while retaining `arena.world` as
    the Task 2/3/4 platform.

### Task 2 deliverable update

For final Task 2 evidence, retain the required modelling screenshot with
the camera and LiDAR mounts clearly visible and record `Video_Task2`
using the prescribed Task 2 vehicle. The trajectory plots and summaries
provide the required recorded `x`, `y`, `theta`, `v` and steering
analysis. The prescribed three-room arena remains the project map; the
open trajectory-test world is supporting validation infrastructure only.
