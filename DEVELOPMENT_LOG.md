# Hexapod Project – Development Log

## 1. Created `hexapod_simulation` Package
A new package named `hexapod_simulation` was created to separate the simulation environment from the robot description.

---

## 2. Added Simulation World
A simulation world file named `world.sdf` was created inside the `hexapod_simulation` package.

---

## 3. Added `start_simulation` Launch File
A launch file named `start_simulation.launch.py` was added. It launches the Gazebo world and invokes the launch file from the `hexapod_description` package to spawn the robot.

---

## 4. Added Material Definitions
Material/color tags were added to the URDF/Xacro files in `hexapod_description` to improve the robot's visual appearance in Gazebo.


---

---

## 5. Fixed Controller Loading & Activation Failures
The `joint_state_broadcaster` and `hexapod_joint_trajectory_controller` were failing to load and activate in simulation. Root causes and fixes, in order:

- **Unresolved config path**: The `<parameters>$(find hexa_description)/config/controller.yaml</parameters>` tag in the xacro's `gz_ros2_control` plugin block wasn't being resolved by the installed `gz_ros2_control` build, so `controller_manager` never received `controller.yaml`. Fixed by loading `controller.yaml` directly via each spawner's `--param-file` flag in `spawn_hexa.launch.py`, instead of relying on the URDF's `<parameters>` tag.

- **Missing spawner nodes**: The `joint_state_broadcaster` and `hexapod_joint_trajectory_controller` spawner nodes had been dropped from `spawn_hexa.launch.py` (left over from an old stale build). Re-added both as `TimerAction`-delayed `Node` entries.

- **Joint name mismatch**: `controller.yaml` listed joint names without the `hexa_` prefix (e.g. `Joint1_1`), while the actual robot joints are prefixed (`hexa_Joint1_1`) per the xacro's `prefix` arg. Updated all joint names in the `joints:` list and `gains:` keys to include the `hexa_` prefix.

- **No physics stepping**: `world.sdf` had no `<physics>` block and no Gazebo system plugins at all (`gz-sim-physics-system`, `gz-sim-user-commands-system`, `gz-sim-scene-broadcaster-system`, `gz-sim-sensors-system`). Without these, Ignition/Gazebo Fortress wasn't stepping physics, which caused `controller_manager`'s activation switch to time out after 5 seconds. Added an explicit `<physics>` block and the required system plugins to `world.sdf`.

- **Simulation starting paused**: `simulation.launch.py` passed `gz_args` as just the world path, causing Gazebo to launch paused. Changed to `gz_args: f"-r {world}"` so the simulation runs immediately on launch.

**Result**: Both controllers now load, configure, and activate successfully (`ros2 control list_controllers` shows both `active`).
