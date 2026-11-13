# Hexapod Project

A six-legged robot simulated in Gazebo with ROS 2 Humble. The project includes the main simulation, standalone gait nodes (open-loop and IMU-assisted closed-loop), and a reinforcement learning (RL) setup.

## Hexapod Simulation Demo

[![Hexapod Robot Simulation](https://img.youtube.com/vi/K-DYaAvGT-s/maxresdefault.jpg)](https://youtu.be/K-DYaAvGT-s)

[Watch the full video on YouTube](https://youtu.be/K-DYaAvGT-s)
## Project Structure

```text
hexapod_ws/src/
	|-hexa_description/
	|-hexa_simulation/
```

Standalone gait nodes live in `hexa_description/hexa_description/`.

## Setup

**Build the workspace and source it (required after cloning or after any code changes):**

```bash
cd ~/hexapod_ws
colcon build
source install/setup.bash
```

In every new terminal, source ROS 2 and the workspace first:

```bash
source /opt/ros/humble/setup.bash
source ~/hexapod_ws/install/setup.bash
```

## 1. Main Simulation

**Option 1 — Normal launch:**

```bash
ros2 launch hexa_simulation simulation_V1.launch.py
```

**Option 2 — If you get mesh-loading errors (e.g. `Unable to find file with URI [model://...]`), run these export commands first, then launch:**

```bash
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix hexa_description)/share
export GZ_SIM_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH
ros2 launch hexa_simulation simulation_V1.launch.py
```

On my system, this was needed to get meshes to load correctly, likely related to having multiple Gazebo Sim versions installed at one point (check with `gz sim --versions`). If Option 1 works cleanly for you, you can skip Option 2.

## 2. Standalone Gait Nodes

The main launch file is not needed to try out individual gaits. Each gait is its own ROS 2 node that publishes joint trajectories to `/hexapod_joint_trajectory_controller/joint_trajectory`.

**Step 1 — Start the simulation:**

```bash
ros2 launch hexa_simulation simulation.launch.py
```

**Step 2 — In a second terminal (sourced as shown in Setup), run a gait node:**

```bash
ros2 run hexa_description forward
```

Available gaits:

| Command | Gait | Control |
|---------|------|---------|
| `ros2 run hexa_description forward` | Tripod gait, walks forward | Open loop |
| `ros2 run hexa_description forward_closed` | Tripod gait, walks forward | Closed loop (IMU yaw hold) |
| `ros2 run hexa_description ripple` | Ripple gait (two legs airborne at a time) | Open loop |
| `ros2 run hexa_description ripple_closed` | Ripple gait | Closed loop (IMU yaw hold) |
| `ros2 run hexa_description sideways` | Tripod gait, walks sideways | Open loop |
| `ros2 run hexa_description sideways_closed` | Tripod gait, walks sideways | Closed loop (IMU yaw hold) |

Notes:

- Run only one gait node at a time, since they all command the same controller.
- The `_closed` versions subscribe to `/imu` and use a PID loop to hold the heading (`yaw_target`). Make sure `/imu` is being published before starting them.
- Gait parameters (`body_vx`, `body_vy`, `cycle_time`, `step_height`, `neutral_reach`, and the yaw-hold gains `kp`, `ki`, `kd`) are at the top of each node's `__init__`. Rebuild after editing them, unless you built with `colcon build --symlink-install`.

## 3. Reinforcement Learning Environment

The RL setup runs in its own Gazebo partition so it does not clash with other simulations on the same network. It loads `world2.sdf` (robot included statically) and starts the `/odom` and pose bridges automatically.

**Set the environment variables and launch the RL simulation:**

```bash
export ROS_DOMAIN_ID=0
export IGN_PARTITION=hexa_sim_0
ros2 launch hexa_simulation simulation_rl.launch.py
```

The pose bridge between Gazebo and ROS 2 starts automatically with the launch file, so no second terminal is needed.

### GUI mode vs headless mode

Gazebo can run with its GUI window (good for visual debugging) or headless, server-only (best for training, since there is no rendering load and you get the highest simulation speed). Switch between them in `hexa_simulation/launch/simulation_rl.launch.py` by commenting and uncommenting the matching `gz_args` line:

```python
launch_arguments={
    # GUI mode: use for visual debugging
    "gz_args":f"-r {world}",

    # Headless mode: use for PPO training (-s = server only, no render window)
    #"gz_args":f"-s -r {world}",
}.items(),
```

For training, comment out the first line and uncomment the second. Rebuild afterwards (`colcon build`) unless you built with `--symlink-install`.

Notes:

- `ROS_DOMAIN_ID` and `IGN_PARTITION` isolate this simulation. To run several environments in parallel, give each one a different value (for example `ROS_DOMAIN_ID=1` and `IGN_PARTITION=hexa_sim_1`).
- The pose bridge reads `pose_bridge.yaml` from the workspace root (`~/hexapod_ws/pose_bridge.yaml`), so keep the file there.
