# Hexapod Project

## Project Structure

```text
hexapod_ws/src/
	|-hexapod_description/
	|-hexapod_simulation/
```

## How to Run

**Build the workspace and source it (required after cloning or after any code changes):**

```bash
cd ~/hexapod_ws
colcon build
source install/setup.bash
```

**Option 1 — Normal launch:**

```bash
ros2 launch hexa_simulation simulationv1.launch.py
```

**Option 2 — If you get mesh-loading errors (e.g. `Unable to find file with URI [model://...]`), run these export commands first, then launch:**

```bash
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix hexa_description)/share
export GZ_SIM_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH
ros2 launch hexa_simulation simulationv1.launch.py
```

On my system, this was needed to get meshes to load correctly — likely related to having multiple Gazebo Sim versions installed at one point (check with `gz sim --versions`). If Option 1 works cleanly for you, you can skip Option 2.
