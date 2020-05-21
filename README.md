# RoboND Catking Workspace

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
docker build -t sunside/ros-gazebo-gpu:udacity-robond -f Dockerfile .
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
