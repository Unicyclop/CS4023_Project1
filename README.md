# CS4023_Project1
## Reactive TurtleBot Simulation Progress

A custom Gazebo world (`reactive_room`) was added for testing the reactive
TurtleBot behavior.

### Current functionality

- TurtleBot4 spawns successfully in the custom Gazebo world.
- LiDAR data is available through `/scan`.
- The controller monitors left, front, and right LiDAR regions.
- The robot normally travels forward.
- After traveling approximately one foot, it performs a random 15-degree turn.
- Obstacles within one foot can trigger avoidance.
- Asymmetric obstacles cause the robot to turn away from the closer side.
- Symmetric/front-facing obstacles can trigger an approximately 175-degree
  escape turn.
- LiDAR distances are printed to the terminal for debugging.

### Important fix

Originally, obstacle avoidance only checked:

`front_distance_ <= OBSTACLE_DISTANCE`

This allowed the robot to get extremely close to a wall on its left or right
side while continuing normal movement.

The asymmetric obstacle condition was updated to check all three LiDAR
regions:

`front_distance_`, `left_distance_`, and `right_distance_`.

Testing showed the robot detecting a nearby wall, performing obstacle
avoidance, triggering the symmetric escape behavior when appropriate, and
then returning to normal movement.

### Running the simulation

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch reactive_turtlebot reactive_room.launch.py
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run reactive_turtlebot reactive_turtlebot
```