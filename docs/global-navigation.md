# Stretch - Global Localization and Navigation

This document describes how to perform global navigation for a Stretch robot
within our lab environment.

## Assumptions

- The environment is indoors, static, and can be fully described using 2D poses
  (x, y, theta).
  - In practice, 3D obstacles in the environment are projected to 2D.
- An operator may provide an initial localization estimate to the robot such
  that it does not need a robust solution to the "wake-up" robot problem.

## Functionality Requirements

- A user should be able to specify an arbitrary 2D goal pose in the map
  coordinate frame. Assuming a valid path exists to this goal pose, the robot
  should plan a path and navigate to the goal position.
- The robot should be localized at all times with respect to the map frame.

## Design Decisions

Necessary subcomponents include mapping, localization, and path planning.

We opt to use the frameworks included by hello-robot in their stretch_ros2
repository. Mapping and localization are performed by the RTAB-Map SLAM
framework. Basic global path planning is performed with Nav2.

We adjust a number of parameters improve functionality in our lab environment.
These parameters are noted in **`bold monospace`** in the next section.

## Workflow

This workflow is a verbose extension of the [instructions provided by hello-robot](/stretch_rtabmap/README.md).

We define two workflow stages:

- "Mapping", in which the robot is teleoperated to generate a map of the
  environment.
- "Navigation", in which the robot performs point-to-point navigation while
  localizing against a previously generated map.

### Mapping

In this mode, RTAB-Map performs 6DoF pose estimation along with 3D point cloud
mapping.

- Home and stow the robot by running the following commands:
  - `stretch_robot_home.py`
  - `stretch_robot_stow.py`
- Go to the root of the ROS 2 workspace containing this package and run the
  following commands:
  - `colcon build --symlink-install`
  - `source install/setup.zsh` (or the appropriate alternative).
- Run `ros2 launch stretch_rtabmap visual_mapping.launch.py`
  - Important arguments and parameters to be aware of:
    - **`--delete_db_on_start`**: Deletes any previous RTAB-Map "database"
      stored at `~/.ros/rtabmap.db`
    - **`Grid/MaxObstacleHeight`**: Sets the upper bound of points that are
      considered obstacles. This is necessary to generate a proper 2D occupancy
      map projection from 3D point clouds.
  - This will launch the `/rtabmap` node responsible for mapping and the
    `/stretch_driver` node responsible for control and wheel odometry.
  - At the time of writing, teleoperation is performed by holding the left
    trigger while moving the right stick.
  - Recall that we are assuming a static environment, so try not to move within
    the robot's field of view during mapping.
- You can visualize the mapping process on a remote machine by running
  `rviz2 -d stretch_rtabmap/rviz/rtabmap.rviz` from the root of this package.
  Topics to keep an eye on:
  - `/cloud_map`: The 3D point cloud map (primarily for visualization).
    - `/cloud_obstacles`: Subset of 3D points associated with obstacles.
    - `/cloud/ground`: Subset of 3D points associated with the ground plane.
  - `/map`: The 2D occupancy grid map used for downstream navigation tasks.
  - `/localization_pose`: The robot pose estimate and associated covariance.
- When you are satisfied with the map, run the following commands to save a
  representation of the map for Nav2.
  - `mkdir -p ${HELLO_FLEET_PATH}/maps`
  - `ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/<map_name>`
    - Use some unique identifier for `<map_name>`, e.g., the current yyyy-mm-dd.
- After the nodes are shut down, the RTAB-Map database is located at
  `~/.ros/rtabmap.db`. We recommend making a backup file in case, e.g.,
  - `cp ~/.ros/rtabmap.db ~/.ros/rtabmap.db.bak-yyyy-mm-dd`.

### Navigation

If you are in a new workflow session and did not just run Mapping, remember to
initialize the robot and your shell environment:

- Home and stow the robot by running the following commands:
  - `stretch_robot_home.py`
  - `stretch_robot_stow.py`
- Navigate to the root of the ROS 2 workspace containing this package and run
  the following commands:
  - `colcon build --symlink-install`
  - `source install/setup.zsh` (or the appropriate alternative).

Before running navigation, recall that two map representations exist: one used
by RTAB-Map located at `~/.ros/rtabmap.db` and another used by Nav2 in
`${HELLO_FLEET_PATH}/maps`. Make sure that `~/.ros/rtabmap.db` matches the Nav2
representation in the following instructions, restoring it from a backup file if
necessary. For example,

```shell
# If the nav2 map representation is ${HELLO_FLEET_PATH}/maps/2024-02-23.{pgm,yaml}

cd ~/.ros
cp rtabmap.db.bak-2024-02-23 rtabmap.db
```

Then,

- Run `ros2 launch stretch_rtabmap visual_navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/yyyy-mm-dd.yaml`
  - Nav2 uses YAML configuration files to specify various planning plugins and
    navigation behavior. These are located in `stretch_nav2/config`. If you decide
    to create a new configuration file, you must also update the **`params_file`**
    parameter in this launch file.
  - Alternatively, the map yaml file path can be specified in the above
    `params_file` with the **`map_server/ros__parameters/yaml_filename`**
    property. Keep in mind that the `${HELLO_FLEET_PATH}` expansion will not
    work in that file.
  - This will launch the `/rtabmap` node responsible for localization and the
    `stretch_driver` node responsible for control and wheel odometry.
- Currently, goal poses can only be specified in RViz using the `2D Goal Pose`
  tool. Run RViz on a remote machine with `rviz2 -d stretch_rtabmap/rviz/rtabmap.rviz`
  from the root of this package. Topics to keep an eye on:
  - `/map`: The 2D occupancy grid map used for navigation tasks.
    - `/global_costmap` (off by default)
    - `/local_costmap` (off by default)
  - `/localization_pose`: The robot pose estimate and associated covariance.
  - `/plan`: The planned path to the goal.
- Relocalizing the robot
  - `/localization_pose` includes a visualization of the associated covariance.
    If the covariance is very large, the entire screen may be within the
    covariance ellipsoid.
  - Sometimes RTAB-Map's relocalization pipeline "just works".
  - To manually relocalize the robot, provide a rough `2D Pose Estimate` vector.
    Then, teleoperate the robot in a couple of loops, ideally to areas that have
    rich visual features, until the localization covariance is small.

### Additional Notes

During navigation, the system seems to handle antagonistic occlusions (e.g.,
covering the camera lens). We hypothesize that the system is simply falling back
on wheel odometry, but this has not been confirmed. The system is also drives
very slowly, and we have not validated this observation over long deployments.

The Nav2 framework seems to be using the 2D Lidar at the base of the Stretch to
perform some basic obstacle avoidance.

There is another launch file in the `stretch_rtabmap` package,
`visual_odometry.launch.py`, but at the time of writing it did not seem too
useful.

## Limitations and Known Issues

- It seems that the stretch driver node and the rtabmap node both publish a
  transform from the odom frame to the base link frame. Occasionally, the two
  nodes will "disagree", causing the robot to teleport repeatedly between the
  two poses.
- The map coordinate frame is not necessarily grounded to the same fixed
  position in the world. Instead, the origin of the map frame is the position at
  which the robot was located when mapping was started.
  - This is not currently implemented, but a proposed solution is to ground the
    map coordinate frame using one or more fiducial markers taped to the walls
    of the environment.
- Naturally, the static environment assumption does not match the real world.
  - The environment needs to be remapped whenever there are major changes.
  - The Nav2 framework is using the 2D Lidar at the base of Stretch to perform
    some limited obstacle avoidance.
- We have been focused on deployment with Stretch. There may be a substantial
  amount of overhead to get this same system working on another robot.
- Cannot map glass and struggles with reflections in general.
- May be sensitive to lighting changes (e.g., the map is generated during the
  day, but navigation is being run at night).
