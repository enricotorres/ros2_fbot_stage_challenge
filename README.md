# 🤖 Behavioral Navigator — Stage Navigation Challenge

A ROS 2 node for autonomous navigation based on reactive behaviors, developed for the **Stage** simulator. The robot autonomously navigates to a goal while avoiding obstacles detected via simulated LiDAR.

---

## 📋 Description

The `BehavioralNavigatorNode` implements a simple behavioral state machine with three operating states:

| State | Description |
|-------|-------------|
| **SEEK** | The robot aligns itself and moves directly toward the goal |
| **AVOID** | An obstacle is detected ahead — the robot rotates toward the freest side |
| **BYPASS** | The front is clear, but the direct path to the goal is still blocked — the robot moves straight while bypassing the obstacle |

State transitions use **hysteresis** to prevent rapid oscillation between states.

---

## 🗂️ Structure

```
navigation_core/
├── navigation_core/
│   ├── __init__.py
│   └── behavioral_navigator.py   # Main node
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── README.md
├── package.xml
├── setup.cfg
└── setup.py
```

---

## ⚙️ Dependencies

- ROS 2 (Humble or later)
- **Stage** (`ros-<distro>-stage-ros2` or built from source)
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`
- `scipy`
- `numpy`

Install Python dependencies:
```bash
pip install scipy numpy
```

---

## 🖥️ Simulator — Stage

[Stage](https://github.com/rtv/Stage) is a lightweight 2D simulator widely used in mobile robotics. It publishes the topics consumed by this node — odometry via `/ground_truth` and laser scan via `base_scan`.

To launch Stage with the challenge world (`cave`):
```bash
ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true
```

---

## 🚀 How to run

```bash
# Build the workspace
colcon build --packages-select navigation_core
source install/setup.bash  # use setup.zsh if you are using the zsh shell

# Terminal 1: launch Stage
ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true

# Terminal 2: launch the navigator
ros2 run navigation_core behavioral_navigator
```

---

## 📡 ROS 2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/ground_truth` | `nav_msgs/Odometry` | Subscriber | Robot position and orientation (published by Stage) |
| `base_scan` | `sensor_msgs/LaserScan` | Subscriber | Simulated LiDAR data (published by Stage) |
| `/cmd_vel` | `geometry_msgs/Twist` | Publisher | Velocity commands sent to Stage |

---

## 🎯 Configuration Parameters

The parameters below can be adjusted directly in the node constructor:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_x / goal_y` | `5.0 / 4.0` | Goal coordinates |
| `angular_tolerance` | `3.0°` | Angular alignment tolerance |
| `goal_tolerance` | `1.0 m` | Distance to consider the goal reached |
| `path_safety_dist` | `1.1 m` | Minimum frontal safety distance |
| `safe_zone` | `1.5 m` | Zone close to goal where obstacle avoidance is disabled |
| `angular_speed` | `0.5 rad/s` | Rotation angular speed |
| `linear_speed` | `2.0 m/s` | Forward linear speed |
| `hysteresis_threshold` | `5 cycles` | Consecutive cycles before switching state |

---

## 🔄 State Machine

```
          obstacle detected
SEEK ──────────────────────► AVOID
  ▲                              │
  │                        front clear
  │                              ▼
  │    path to goal clear   BYPASS
  └───────────────────────       │
                                 │ obstacle detected
                                 └──────────► AVOID
```

---

## 📌 Notes

- The robot stops completely upon reaching the goal and publishes a zero `Twist`.
- Obstacle detection uses a frontal cone of **30°** total (±15° relative to the forward axis).
- Goal path checking uses an angular window of **±10°** around the direction to the goal.
- The turn direction in AVOID is chosen dynamically based on the average LiDAR readings on the left vs. right side.
- The `enforce_prefixes:=false` and `one_tf_tree:=true` flags ensure the topics (`/ground_truth`, `base_scan`) are published without a namespace prefix, which is required for the node to communicate correctly with the simulator.
- When the goal is behind the robot (angle outside the LiDAR range), `is_goal_path_clear` returns `False` — the robot stays in **BYPASS** until it rotates enough for the goal to enter the sensor's field of view.

---

## 🎬 Demo

https://github.com/user-attachments/assets/820b8336-6fd7-44cd-a066-fadd068f9e8c
