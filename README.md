# Robust Docking with LiDAR-Based Online Pose Refinement

A ROS 2 (Humble) docking layer for a TurtleBot3 that refines the final approach to
a docking station using **laser geometry** instead of trusting SLAM/odometry pose.
It fits the dock face and side wall from the scan, derives the relative pose error
`(range, lateral, yaw)`, and drives a stable, rate-limited control law to converge —
with a finite state machine handling search, approach, a **two-stage** dock
(pre-dock line-up of Y/yaw, then a straight push-in for X), and safety
(hold / abort) behaviour. A separate, configurable **disturbance injector**
stresses the system with scan dropout, scan corruption, pose noise and a moving
obstacle.

## Packages

| Package               | Purpose |
|-----------------------|---------|
| `docking_sim`         | Gazebo world with the L-corner dock, movable obstacle, TB3 spawn, RViz. |
| `docking_controller`  | `docking_node` (FSM + RANSAC/TLS geometry + control law), `line_fit` lib. |
| `docking_disturbance` | `disturbance_node` — programmable scan/pose/obstacle disturbances. |
| `docking_bringup`     | `demo.launch.py`, `dock_eval.py`. |

## Prerequisites

| Requirement        | Version / Notes                                |
| ------------------ | ---------------------------------------------- |
| Docker             | 20.10+ with `docker compose` v2                |
| X11 forwarding     | For RViz/Gazebo GUI (`xhost +local:root`)      |
| VS Code (optional) | With **Dev Containers** extension for Option C |

> **Note:** All ROS 2 dependencies (Gazebo, TurtleBot3 packages, build tools) are
> installed inside the Docker image. No host ROS installation is needed.

### Docker image & display

Do this once before any option. Pull the prebuilt image (or build it) and allow
GUI access:

```bash
xhost +local:root                              # allow RViz/Gazebo GUI
docker pull jeyakaranm/autonomy_dev:humble or make build-docker
```


---

## Repository Structure

```
├── Makefile                          # Build & run commands (host targets)
├── Dockerfile                        # ROS 2 Humble image with all deps
├── docker-compose.yml                # Development (source-mounted, dev container)
├── docker-compose-prod.yml           # Production (built image, no source mounts)
├── .devcontainer/devcontainer.json   # VS Code Dev Container config
├── scripts/
│   └── run_demo.sh                   # One entry point: baseline / disturbed / eval
└── src/
    ├── docking_sim/                  # Gazebo L-corner dock, obstacle, TB3, RViz
    ├── docking_controller/           # docking_node (FSM + RANSAC/TLS + control)
    ├── docking_disturbance/          # disturbance_node (scan/pose/obstacle faults)
    └── docking_bringup/              # demo.launch.py, dock_eval.py
```

---

## Launching Docking 

### Option A — Production (Docker one-command)

One command brings up the stack and docks automatically (~20 s delay to trigger docking for sim to launch):

```bash
make run-baseline-docking-autostart            # baseline, auto-docks
make run-disturbed-docking-case1-autostart     # scan corruption + pose noise + obstacle
make run-disturbed-docking-case2-autostart     # scan stale/dropout + pose noise + obstacle
```

### Option B — Production Container (Manual Launch)

Start the stack in Terminal 1, then trigger docking from Terminal 2. Three
scenarios:

```bash
# Scenario 1 — baseline (no disturbances)
make run-baseline-docking            # T1: Gazebo + RViz
make run-start-docking               # T2: trigger docking

# Scenario 2 — case 1: scan corruption (forward sector -> inf, 5 s) + pose noise + obstacle
make run-disturbed-docking-case1     # T1
make run-start-docking               # T2

# Scenario 3 — case 2: scan dropout/stale (5 s) + pose noise + obstacle
make run-disturbed-docking-case2     # T1
make run-start-docking               # T2
```

### Option C — Dev Container (VS Code)

Source-mounted environment: edit on the host, build/test inside the container.

```
1. Open this repository in VS Code
2. Install the "Dev Containers" extension
3. F1 → "Dev Containers: Reopen in Container"
```

Build, then run the script directly:

```bash
colcon build --base-paths src/autonomy_dev/src --symlink-install

/root/src/autonomy_dev/scripts/run_demo.sh baseline  --rviz
/root/src/autonomy_dev/scripts/run_demo.sh disturbed --case 1 --rviz   # scan corruption + pose noise + obstacle
/root/src/autonomy_dev/scripts/run_demo.sh disturbed --case 2 --rviz   # scan stale/dropout + pose noise + obstacle
```

These wait for a manual start. In a second container shell, trigger docking:

```bash
ros2 service call /docking/start std_srvs/srv/Trigger
```

Or skip the manual step by adding `--autostart` to dock ~20 s after launch:

```bash
/root/src/autonomy_dev/scripts/run_demo.sh disturbed --case 1 --rviz --autostart
```

> **Native (no Docker/dev container):** needs ROS 2 Humble installed. From the
> workspace root run `rosdep update && rosdep install --from-paths src/autonomy_dev/src
> --ignore-src -r -y`, then `colcon build --base-paths src/autonomy_dev/src --symlink-install`,
> source `install/setup.bash`, and use the same `run_demo.sh` commands.
## Key topics & services

- `/scan`, `/odom`, `/cmd_vel` — robot I/O (disturbed variants: `/scan_dock`, `/odom_dock`).
- `/docking/status` — human-readable state and error string.
- `/docking/markers` — RViz visualization of fitted lines, corner and target.
- `/docking/start`, `/docking/cancel` — `std_srvs/Trigger` services.

## Goal pose offsets

The final dock pose is defined relative to the detected dock corner in
[src/docking_controller/config/docking_params.yaml](src/docking_controller/config/docking_params.yaml):

- `longitudinal_offset` (0.45 m) — standoff from the dock face along its normal.
- `lateral_offset` (0.50 m) — offset along the dock face to the goal point.
- `predock_pose_offset_longitudinal` (1.0 m) — stage-1 line-up distance from the dock face.
- `xy_tolerance` / `yaw_tolerance` — success thresholds at the goal.

Adjust these to retarget the dock/predock poses without code changes.


