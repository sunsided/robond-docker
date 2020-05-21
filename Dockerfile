FROM sunside/ros-gazebo-gpu:kinetic-nvidia

# Install required ROS packages.
USER root
RUN apt-get update && apt-get install -y \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-effort-controllers \
    ros-kinetic-joint-state-controller \
 && rm -rf /var/lib/apt/lists/*

# Switch back to the ROS user.
USER ros
