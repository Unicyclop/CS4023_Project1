# CS4023 Project 1
## Reactive TurtleBot 4 Simulation

This repository contains our ROS 2 and Gazebo implementation of a
priority-based reactive controller for a simulated TurtleBot 4.

The project uses a custom Gazebo environment and a C++ ROS 2 controller
to implement the required reactive behaviors. Development and testing
have focused on connecting TurtleBot 4 sensor information to the
controller, implementing the required behavior priorities, and testing
the robot's responses in simulation.

---

# Development Progress

## Initial Implementation

A custom Gazebo world named `reactive_room` was created for testing the
reactive TurtleBot 4 behavior.

The initial implementation established the following functionality:

- TurtleBot 4 successfully spawns in the custom Gazebo world.
- LiDAR data is available through the `/scan` ROS 2 topic.
- The controller monitors left, front, and right LiDAR regions.
- The robot normally travels forward when no higher-priority behavior
  is active.
- After traveling approximately one foot, the robot performs a random
  15-degree turn.
- Obstacles within approximately one foot can trigger avoidance behavior.
- Asymmetric obstacles cause the robot to turn away from the closer side.
- Symmetric or front-facing obstacles can trigger an approximately
  175-degree escape turn.
- LiDAR distances are printed to the terminal during testing to help
  observe what the controller detects.

---

## Obstacle Detection Improvement

During initial testing, obstacle avoidance only checked:

`front_distance_ <= OBSTACLE_DISTANCE`

This allowed the robot to become extremely close to a wall on its left
or right side while continuing its normal movement.

The obstacle condition was updated to consider all three LiDAR regions:

- `front_distance_`
- `left_distance_`
- `right_distance_`

Testing after this change showed the robot detecting nearby walls,
performing asymmetric avoidance when appropriate, triggering the
symmetric escape behavior when appropriate, and then returning to normal
movement.

---

# Current Implementation

The controller currently implements the six required priority-based
reactive behaviors.

The behaviors are evaluated according to their required priority so that
a higher-priority behavior can override a lower-priority behavior when
necessary.

The current priority order is:

1. Bumper collision halt
2. Human keyboard control
3. Symmetric obstacle escape
4. Asymmetric obstacle avoidance
5. Random turn after forward travel
6. Default forward movement

---

## 1. Bumper Collision Halt

Gazebo contact sensing has been enabled in the custom world.

Bumper contacts are published in Gazebo and bridged to ROS 2 through:

`/bumper_contact`

The controller subscribes to this topic and checks whether physical
contact has been detected.

When a bumper collision is detected:

- `bumper_detected_` becomes active.
- Linear movement is set to zero.
- Angular movement is set to zero.
- The robot halts.
- This behavior overrides all lower-priority behaviors, including an
  active human keyboard command.

Testing confirmed that a physical TurtleBot 4 bumper collision with a
wall is detected by Gazebo, transferred through the ROS-Gazebo bridge,
received by the C++ controller, and used to halt the robot.

Contact timing is also used to prevent an old bumper state from leaving
the controller permanently halted. Each detected contact updates the
time of the most recent bumper contact. If contact messages stop for the
configured timeout period, the bumper state is cleared and normal
controller operation can resume.

Bumper messages are throttled during testing so that continuous Gazebo
contact messages do not unnecessarily flood the terminal.

---

## 2. Human Keyboard Control

Human movement commands are accepted by the controller through:

`/keyboard_cmd_vel`

The `teleop_twist_keyboard` ROS 2 node can be remapped to publish its
commands to this topic.

When a nonzero keyboard command is received, keyboard control becomes
active and overrides lower-priority autonomous behaviors such as:

- symmetric obstacle escape,
- asymmetric obstacle avoidance,
- random turning, and
- default forward movement.

Keyboard control does not override the higher-priority bumper collision
halt.

Testing confirmed the intended priority relationship by manually driving
the robot toward a wall. The controller continued accepting the human
movement command until the physical bumper detected a collision. The
bumper behavior then overrode the active keyboard command and halted the
robot.

---

## 3. Symmetric Obstacle Escape

LaserScan data from `/scan` is divided into left, front, and right
regions.

When an obstacle is detected within approximately one foot and the
obstacle measurements are sufficiently symmetric, the robot begins its
escape behavior.

The escape behavior:

- stops forward movement,
- begins a fixed turn,
- continues turning even after the original obstacle stimulus changes,
- and ends after the robot has turned approximately 175 degrees.

This behavior is implemented as a fixed-action response rather than a
simple reflex because the robot continues the turn after the condition
that originally triggered the behavior may no longer be present.

Testing showed the controller entering the symmetric escape behavior,
continuing the turn, completing the escape, and returning to lower-
priority behaviors afterward.

---

## 4. Asymmetric Obstacle Avoidance

The controller also uses the left, front, and right LiDAR regions to
detect asymmetric obstacles within approximately one foot of the robot.

When an asymmetric obstacle is detected:

- forward movement stops,
- the controller compares the left and right detected distances,
- and the robot turns away from the closer obstacle.

For example, if the left side is closer than the right side, the robot
turns toward the right.

Unlike the symmetric escape behavior, asymmetric avoidance is a reflex.
It is active only while the nearby asymmetric obstacle condition exists.

Testing showed the robot detecting nearby side obstacles and turning away
from the closer side.

---

## 5. Random Turning

The controller tracks the robot's position using odometry from:

`/odom`

After the robot travels approximately one foot forward, it begins a
random turn.

The turn direction is selected randomly between:

- approximately 15 degrees to the left, or
- approximately 15 degrees to the right.

After completing the turn, the robot resets its forward-distance
reference and resumes normal behavior unless a higher-priority behavior
is active.

---

## 6. Default Forward Movement

Forward movement is the controller's lowest-priority behavior.

When none of the higher-priority conditions are active, the TurtleBot 4
moves forward.

This allows the other behaviors to interrupt normal movement whenever
sensor input or a human command requires a different response.

---

# Gazebo Environment

The custom Gazebo world is located at:

`worlds/reactive_room.sdf`

The ROS 2 launch file is located at:

`launch/reactive_room.launch.py`

The custom world contains the room and hallway environment used for
development and testing.

The launch file starts the TurtleBot 4 simulation using the custom world.

Gazebo contact sensing is enabled in the world so that physical
collisions involving the TurtleBot 4 bumper can be detected and
published.

The Gazebo contact information is bridged to ROS 2 through:

`/bumper_contact`

The TurtleBot 4 LiDAR is available to the controller through:

`/scan`

Odometry information is available through:

`/odom`

---

# Building and Running

The following commands describe the current development setup.

## Terminal 1 - Build and Launch the Simulation

From the ROS 2 workspace:

```bash
cd ~/ros2_ws

colcon build --packages-select reactive_turtlebot

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch reactive_turtlebot reactive_room.launch.py
```

Wait for Gazebo and the TurtleBot 4 simulation to finish loading before
starting the controller.

---

## Terminal 2 - Start the Reactive Controller

Open another terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run reactive_turtlebot reactive_turtlebot
```

During testing, the controller prints diagnostic information such as:

```text
LiDAR | LEFT: ... m | FRONT: ... m | RIGHT: ... m
```

Additional messages indicate which reactive behavior is currently
occurring, such as:

```text
Avoiding asymmetric obstacle
Starting 180 degree escape
Escaping symmetric obstacle
Escape complete
Starting random 15 degree turn
Performing random 15 degree turn
Keyboard control active
Bumper collision detected
BUMPER ACTIVE - ROBOT HALTED
Bumper contact cleared
```

---

## Terminal 3 - Human Keyboard Control

To test human keyboard control, open a third terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -r cmd_vel:=/keyboard_cmd_vel
```

Common keyboard commands include:

```text
i = move forward
, = move backward
j = turn left
l = turn right
k = stop
```

The keyboard node publishes to `/keyboard_cmd_vel` rather than directly
controlling `/cmd_vel`. This allows the reactive controller to receive
the human command and preserve the required behavior priorities.

---

# Testing Completed

The following functionality has been observed and tested in simulation:

- TurtleBot 4 spawning in the custom Gazebo world.
- LiDAR data being received through `/scan`.
- Left, front, and right LiDAR region monitoring.
- Default forward movement.
- Random turns after approximately one foot of forward travel.
- Asymmetric obstacle detection.
- Asymmetric obstacle avoidance.
- Symmetric obstacle detection.
- Fixed-action symmetric escape turn.
- Return to normal behavior after completing an escape.
- Human keyboard movement commands.
- Keyboard commands overriding lower-priority autonomous behaviors.
- Gazebo physical bumper collision detection.
- Gazebo `/bumper_contact` contact generation.
- ROS 2 `/bumper_contact` receiving the Gazebo contact information.
- Physical bumper collisions reaching the C++ controller.
- Highest-priority bumper halt overriding an active keyboard command.
- Release of the bumper halt after contact messages stop.
- Repeated bumper collisions being detected after the previous bumper
  state has cleared.

LiDAR diagnostic output is currently printed by the controller during
testing to show the minimum detected distances in the left, front, and
right regions.

---

# Current Reactive Behavior Priority

The controller currently follows this priority structure:

```text
Highest Priority

1. Bumper collision detected
        |
        +--> HALT

2. Human keyboard command active
        |
        +--> Follow human movement command

3. Symmetric obstacle detected
        |
        +--> Perform fixed escape turn

4. Asymmetric obstacle detected
        |
        +--> Turn away from closer obstacle

5. Approximately one foot traveled
        |
        +--> Perform random 15-degree turn

6. No higher-priority condition
        |
        +--> Drive forward

Lowest Priority
```

This structure allows higher-priority safety and human-control behaviors
to override lower-priority autonomous movement.

---

# Project File Structure

Important project files currently include:

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
|   `-- reactive_turtlebot.cpp
|
`-- worlds/
    `-- reactive_room.sdf
```

`src/reactive_turtlebot.cpp` contains the main C++ reactive controller.

`worlds/reactive_room.sdf` defines the custom Gazebo testing
environment.

`launch/reactive_room.launch.py` launches the TurtleBot 4 simulation
using the custom world.

---

# Remaining Work

The following work remains before the project is considered complete:

- Occupancy-grid mapping.
- Additional robustness testing with different TurtleBot 4 starting
  positions.
- Additional testing with changes to the environment configuration.
- Testing on the CSN Linux machines.

---

# Development Notes

The project is being developed incrementally so that individual reactive
behaviors can be tested before final integration.

Important issues identified and corrected during development include:

1. Obstacle avoidance originally depended too heavily on the front LiDAR
   region. The condition was expanded to account for left and right
   proximity as well.

2. The custom Gazebo world initially did not produce the required bumper
   contact information. Gazebo contact sensing was enabled and verified
   before testing the ROS 2 bridge.

3. Gazebo bumper contact was verified first on the Gazebo side and then
   through the ROS 2 `/bumper_contact` topic.

4. Continuous contact originally produced excessive terminal messages.
   Bumper collision logging was throttled.

5. Bumper contact handling was updated so that an old contact state does
   not permanently leave the controller halted after contact messages
   have stopped.

6. Priority testing confirmed that a physical bumper collision overrides
   an active human keyboard movement command.

These changes were made as part of testing and improving the robustness
of the reactive controller.