# RoboND Catkin Workspace

Step into the ROS Kinetic Docker container by running

```bash
./run-nvidia.sh
```

Note that `run-nvidia.sh` makes use of the `sunside/ros-gazebo-gpu:udacity-robond`
Docker image, because the original `desktop-full` ROS installation doesn't contain some
required packages, such as

- `controller_manager`

You can either have the required image pulled automatically, or build it yourself from
the provided [Dockerfile](Dockerfile) by running

```bash
docker build --no-cache -t sunside/ros-gazebo-gpu:udacity-robond -f Dockerfile .
```

## Setting up a Catkin workspace (once)

Ensure the `src` directory exists, then `cd` into it and call `catkin_init_workspace`
to initiate a new Catkin workspace:

```bash
mkdir -p src && cd src
catkin_init_workspace
```

This should create a result similar to the following:

```
Creating symlink "/workspace/src/CMakeLists.txt" pointing to "/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake"
```

If we run `ls -l`, we find that a symbolic link to Catkin's top-level `CMakeLists.txt` was created:

```
total 0
lrwxrwxrwx 1 ros ros 50 May 21 19:10 CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
```

## Building the workspace

Change back to the root directory of the repository (the one _containing_ `src/`) and call

```bash
catkin_make
```

If we run `ls -l` again we will find, that the following directories exist:

- `build`: The CMake build directory
- `devel`: ROS and Catkin resources, such as `setup.bash`

## Simple Arm

![The Simple Arm simulation environment](src/simple_arm/images/simulation.png)

Ensure that all packages are installed as required:

```bash
rosdep install -i simple_arm
```

If a package is missing, this will ask you for permission to
install it. Note that unless you run `sudo apt update` in the
container at least once, these updates will fail. For this project,
the required packages are included in the [`Dockerfile`](Dockerfile)
used to run th environment with.

Build the project via

```bash
catkin_make
```

then run `roslaunch` as described below to run the project:

```
source devel/setup.bash
roslaunch simple_arm robot_spawn.launch
```

If you encounter a warning

```
[WARN] [1590093853.282634, 0.000000]: Controller Spawner couldn't find the expected controller_manager ROS interface.
```

… that probably just means you started Gazebo for the first time in this Container
and it takes a bit too long to do anything.


### Fixing `legacyModeNS` errors for Kinetic

Originally, the code produced the following output:

> ```
> [ERROR] [1590091780.648877316, 261.772000000]: GazeboRosControlPlugin missing <legacyModeNS> while using DefaultRobotHWSim, defaults to true.
> This setting assumes you have an old package with an old implementation of DefaultRobotHWSim, where the robotNamespace is disregarded and absolute paths are used instead.
> If you do not want to fix this issue in an old package just set <legacyModeNS> to true.
> ```

To fix, add the `<legacyModeNS>true</legacyModeNS>` attribute to the appropriate node of the `.gazebo.xacro` file that is affected. You can
use the following command to pinpoint its location:

```bash
./src/simple_arm/urdf/simple_arm.gazebo.xacro
```

See the [`src/simple_arm/README.md`](src/simple_arm/README.md) for more information.
