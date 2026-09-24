# Black Bee Challenge 2026

Repository with the autonomous drone missions built for the **Black Bee Challenge 2026** (UNIFEI), a competition held from **August 28 to 30**, aimed at getting the team's rookies ready for the official competitions.

Each mission is a standalone ROS 2 package, orchestrated as a state machine with **[YASMIN](https://github.com/uleroboticsgroup/yasmin)** and built on top of the team's own SDK, **`nectar`** (drone control via MAVROS/MAVLink, plus vision/detection). Each folder's own README covers that mission's full FSM architecture, state/outcome table, blackboard keys, and how to run it.

## Missions

| Mission | Folder | What it does |
|---|---|---|
| 🎯 **[Precision Landing](./precision_landing/README.md)** | `precision_landing/` | Lands the drone autonomously on the correct base in an arena full of decoy targets, identifying it by the combination of shape + number (ArUco marker). |
| 📦 **[Package Delivery](./package_delivery/README.md)** | `package_delivery/` | Takes off, flies to the GPS coordinates of each target box, locates and centers over it via YOLO, drops the package with a servo, and repeats the cycle until every box is delivered. |
| 🪝 **[Hang The Hook](./hang_the_hook/README.md)** | `hang_the_hook/` | Follows a blue line on the ground until it finds a red hose, aligns and descends over it, and actuates a servo to hang a hook on it. |
| 🗺️ **[Mapping](./mapping/README.md)** | `mapping/` | Flies a coverage grid over the arena, photographs and detects bases (shape/color markers) via OpenCV or YOLO, converts their positions to GPS, and publishes a results report. |

## Shared stack

- **ROS 2** (`rclpy`) as the communication layer between nodes
- **YASMIN** / `yasmin_ros` (and `yasmin_viewer` in some missions) to orchestrate each mission as a hierarchical state machine instead of a linear script
- **`nectar`** — the Black Bee's own SDK, split into drone-control (`nectar.control`: MAVROS/MAVLink, PID, movement), vision (`nectar.vision`: a real camera via OpenCV or a simulated one via a ROS topic, ArUco/line detection), and AI (`nectar.ai` / YOLO models for detecting boxes, shapes, and bases) modules — not the `nectar` package on PyPI, this is distributed internally by the team
- **OpenCV** (`cv2`), used across all missions for capture, correction, and/or debugging of images
- **Gazebo (SITL)**, used as an alternative to real hardware in every mission, toggled by a configuration flag (`SIM_MODE` / `sim_mode`)

## Repository structure

```
.
├── precision_landing/     # precision landing on an ArUco base (shape + number)
├── package_delivery/      # package delivery to GPS coordinates
├── hang_the_hook/         # follow a blue line and hang a hook on the hose
└── mapping/               # coverage-grid survey and base detection
```

Each folder above has its own `README.md` with the full mission architecture (FSM diagram, state/outcome table, blackboard keys, and how to run it).
