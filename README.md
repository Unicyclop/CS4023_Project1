# CS4023 Project 1

## Reactive TurtleBot 4 Simulation

This repository contains our ROS 2 and Gazebo implementation of a reactive controller for a simulated TurtleBot 4. The robot moves through a custom room and hallway environment, reacts to sensor input according to a fixed priority order, accepts human keyboard commands, and constructs an occupancy grid while it moves.

## Current Implementation

The controller implements the six required behaviors in the following priority order:

1. **Bumper collision halt** - Stops the robot when a physical bumper collision is detected.
2. **Human keyboard control** - Allows a user to manually control the robot unless the higher-priority bumper behavior is active.
3. **Symmetric obstacle escape** - When obstacles within approximately one foot are roughly symmetric in front of the robot, it performs a fixed escape turn of approximately 175 degrees.
4. **Asymmetric obstacle avoidance** - Turns away from nearby asymmetric obstacles detected by the LiDAR.
5. **Random turning** - After approximately one foot of forward travel, the robot randomly turns about 15 degrees left or right.
6. **Forward movement** - Drives forward when no higher-priority behavior is active.

The robot uses LiDAR, odometry, keyboard commands, and Gazebo bumper contact information. A separate occupancy-grid mapper uses LiDAR and odometry data to construct a map while the robot explores.

## Reactive Architecture

The controller uses a priority-based reactive architecture. Sensor callbacks maintain the current sensor and command information, and the control loop checks the behaviors in priority order. An `if` / `else if` structure ensures that the highest-priority active behavior controls the robot and suppresses lower-priority behaviors.

The symmetric escape is a fixed-action behavior. Once triggered, the robot continues turning until the escape angle is reached even if the original sensor condition changes. Asymmetric avoidance instead acts as a reflex and continues only while the obstacle condition remains.

The occupancy grid is created alongside the controller, but the robot does not use the map to choose its movement.

## Occupancy Grid Mapping

The occupancy-grid mapper is implemented in:

`src/occupancy_grid_mapper.cpp`

It publishes a `nav_msgs/msg/OccupancyGrid` on:

`/map`

The grid uses:

- `-1` for unknown cells
- `0` for free cells
- `100` for occupied cells

The mapper has been added to `CMakeLists.txt` and has been tested with the simulation. The resulting occupancy grid has also been viewed in RViz.

## Gazebo Environment

The custom Gazebo world is:

`worlds/reactive_room.sdf`

The launch file is:

`launch/reactive_room.launch.py`

The world contains the room and backward-L-shaped hallway used for the project. The launch file starts the TurtleBot 4 in this environment and connects the required Gazebo and ROS 2 topics.

Important sensor topics include LiDAR, odometry, and bumper contact data. In the namespaced TurtleBot 4 simulation these can appear as:

```text
/robot1/scan
/robot1/odom
/robot1/bumper_contact
```

The launch configuration/remappings connect these topics to the project nodes.

The TurtleBot 4 simulation also creates its standard dock. During development the dock was moved away from the main testing area because it could interfere with autonomous movement near the robot's starting position.

## Important Project Files

```text
reactive_robot/
|
|-- CMakeLists.txt
|-- package.xml
|-- README.md
|
|-- launch/
|   `-- reactive_room.launch.py
|
|-- src/
|   |-- reactive_turtlebot.cpp
|   `-- occupancy_grid_mapper.cpp
|
`-- worlds/
    `-- reactive_room.sdf
```

- `reactive_turtlebot.cpp` - Main priority-based reactive controller.
- `occupancy_grid_mapper.cpp` - Occupancy-grid mapping node.
- `reactive_room.sdf` - Custom Gazebo environment.
- `reactive_room.launch.py` - Launches the simulation and required bridges.
- `CMakeLists.txt` - Builds and installs the controller and mapper.

## Build and Run

### 1. Build the Package

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select reactive_turtlebot
source ~/ros2_ws/install/setup.bash
```

### 2. Launch the Simulation

```bash
ros2 launch reactive_turtlebot reactive_room.launch.py
```

Wait for Gazebo and the TurtleBot 4 to finish loading.

The launch file starts the simulation, required bridges, reactive controller, and occupancy-grid mapper. The controller terminal output displays LiDAR and odometry information along with messages showing active behaviors such as obstacle avoidance, escape turns, random turns, and bumper responses.

### 3. Run Keyboard Control

To manually control the robot, open another terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -r cmd_vel:=/keyboard_cmd_vel
```

Common controls are:

```text
i = forward
, = backward
j = turn left
l = turn right
k = stop
```

Keyboard commands pass through the reactive controller so that higher-priority safety behavior can override manual movement when necessary.

### 4. View the Occupancy Grid

The occupancy-grid mapper is started automatically by `reactive_room.launch.py`.

To verify that the map is being published:

```bash
ros2 topic info /map
ros2 topic echo /map --once
```

To visualize the map:

```bash
rviz2
```

In RViz:

1. Set **Fixed Frame** to `robot1/odom`.
2. Add a **Map** display and select `/map`.
3. Optionally add a **LaserScan** display and select `/robot1/scan`.

The occupancy grid will update as the TurtleBot explores the environment.

## Testing Status

The following have been tested in the current simulation:

- TurtleBot 4 spawning in the custom world
- LiDAR and odometry input
- Default forward movement
- Random turns after approximately one foot
- Asymmetric obstacle avoidance
- Symmetric fixed-action escape
- Human keyboard control
- Bumper collision detection and highest-priority halt
- Recovery after bumper contact ends
- Autonomous movement through the room/hallway environment
- Occupancy-grid publication on `/map`
- Occupancy-grid visualization in RViz

