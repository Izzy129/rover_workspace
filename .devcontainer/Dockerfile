FROM ros:jazzy-ros-base

# Arguments for user creation
ARG USERNAME=robotics
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete existing user with same UID to avoid conflicts
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel $(id -un $USER_UID); fi

# Create a non-root user with sudo access
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the non-root user
USER $USERNAME

# Add user to video group (GUI apps, camera access)
RUN sudo usermod --append --groups video $USERNAME

# Set default shell
ENV SHELL=/bin/bash

# Update system and install common tools
RUN sudo apt-get update && sudo apt-get upgrade -y \
    && sudo apt-get install -y git python3-pip python3-colcon-common-extensions

# Source ROS environment automatically
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

# Install proper OpenCV version for Aruco API
RUN pip install opencv-contrib-python>=4.8.0

