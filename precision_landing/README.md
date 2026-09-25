# Precision Landing — Black Bee Challenge 2026

ROS 2 package responsible for autonomously landing the drone on the correct base in an arena containing several decoy targets. The correct base is identified by the combination **geometric shape + number (ArUco marker)**, and the entire flight is orchestrated by a state machine built with **yasmin**, using the **Nectar** SDK to abstract the drone, camera, and detection.

## Stack and libraries

- **ROS 2 (`rclpy`)** — communication between nodes and package execution.
- **`yasmin` / `yasmin_ros`** — state machine framework: organizes the flight as a sequence of states with transitions (`SUCCEED`, `ABORT`, `FAIL`, `TIMEOUT`), instead of a linear script.
- **`nectar`** (the team's SDK), split into three modules:
  - `nectar.control` — abstracts the real/simulated drone via MAVROS (`DroneFactory`, `MavrosDrone`), the `PIDController`, and movement (`MoveReference`).
  - `nectar.vision` — camera (`ImageHandler`, with `ROSConfig` for Gazebo and `OpenCVConfig` for real flight) and ArUco marker detection (`Aruco`).
  - `nectar.ai` — `Detector`, which runs a YOLO model (`best_detector1.pt`) to recognize the shapes (triangle, hexagon, star) and numbers in the arena.
- **OpenCV + `cv_bridge`** — used in the `view_camera` utility to visualize the camera feed.
- **Gazebo (SITL)** — simulation, with the base/marker models in `simulation/gazebo_models`.

## Structure

```
precision_landing/
├── mangalarga.py         # entry point: assembles the main state machine
├── constants.py          # all adjustable parameters (PID, timings, waypoints...)
├── states/                # states of the main state machine
│   ├── initialize.py
│   ├── takeoff.py
│   ├── precision_landing.py
│   └── land.py
├── findSM/                # sub-state-machine for locating the target
│   ├── findSM.py
│   └── states.py          # Search and FindTargetBase
└── utils/
    └── view_camera.py     # debug helper node (displays the camera feed)
```

## State machine flow

```
INITIALIZE → TAKEOFF → FIND_SM → PRECISION_LANDING → SUCCEED
     ABORT↴      ABORT↴    FAIL/ABORT/TIMEOUT↴   ABORT/TIMEOUT↴
                                 LAND
```

Inside `FIND_SM` there is a sub-state-machine: `SEARCH → FIND_TARGET_BASE` (each one retries on `FAIL`, until time runs out or the target is found).

## Logic of each state

**INITIALIZE** — creates the drone (`DroneFactory`, with simulation or real MAVROS/GPS config), loads the `Detector` (YOLO), and opens the camera, registering `camera_callback` to run detection on every frame. Aborts if any of the three steps fails.

**TAKEOFF** — simple takeoff command up to `TAKEOFF_HEIGHT`.

**SEARCH** (inside `FIND_SM`) — traverses a fixed list of `WAYPOINTS` until an ArUco marker is spotted. Upon finding it, it calculates the yaw from the marker's corners (`calculateYawFromCorners`) and identifies the shape around it (`get_aruco_shape`), saving `aruco_shape` + `aruco_id` to the blackboard — this combination is the "address" of the correct base. A buffer (`FIND_BUFFER` / `ADD_FINDING_BUFFER`) stores shape+number combinations already seen at previous waypoints, avoiding repeated scanning if the correct target has already passed through the camera.

**FIND_TARGET_BASE** — returns to the waypoints (or goes directly if the target has already been pre-found) until it frames, in the same shot, the shape and number saved as the target.

**PRECISION_LANDING** — the core of the fine control:
- On every frame, it measures the pixel error between the target's center and the image's center;
- `PIXEL_POR_METRO()` converts this error from pixels to meters, using the camera's field of view and the current altitude (the higher the drone, the more meters each pixel represents);
- The error in meters feeds two `PIDController` instances (X and Y), which generate the lateral velocities applied to the drone;
- Descent (Z) is conditional: it only descends when the lateral pixel error is within a tolerance (tighter closer to the ground);
- If it loses sight of the target, it keeps the last velocity for a while ("drift") waiting to reacquire it, or already finalizes the landing if it is low and aligned enough;
- Returns `TIMEOUT` if the maximum time runs out without completing the landing.

**LAND** — triggers `drone.land()`, both on the success path and on any abort in the middle of the process.

## How to run

```bash
ros2 run precision_landing mangalarga      # full flight (main state machine)
ros2 run precision_landing view_camera     # debug window with the camera feed
```

`constants.py` gathers what usually changes between simulation and real flight: `SIM_MODE`, camera source, PID gains, scan waypoints, and landing tolerances.
