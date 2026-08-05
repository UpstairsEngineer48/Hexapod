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
