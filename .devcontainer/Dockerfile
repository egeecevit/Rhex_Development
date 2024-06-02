FROM osrf/ros:humble-desktop-full


# Set non-interactive frontend to prevent interactive prompts during package installations
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

# Set the working directory
WORKDIR /usr/src

# Update system and install necessary packages in a single RUN statement to reduce image layers
# Install General Dependencies
RUN apt-get update -y && \
    apt-get install -qq -y --no-install-recommends \
        build-essential \
        sudo \
        git \
        wget \
        python3 \
        python3-dev \
        python3-pip
    

#Install ROS packages
RUN apt install ros-humble-cv-bridge -y \
    ros-humble-gazebo-* -y \
    ros-humble-turtlebot3* -y \
    ros-humble-teleop-twist-keyboard -y \
    ros-humble-joint-state-publisher* -y \
    ros-humble-robot-state-publisher* -y &&\
    # Clean up APT cache to reduce image size
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Add user so the files created in the container are not owned by root
# Following operations needs sudo to be installed

# Add a new user 'dev' to use instead of 'root' for better security practices
# Configure user with no-password sudo access
ARG USERNAME=dev-ros2
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a group and user with specified UID/GID and add user to sudoers with no password entry
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Set the default user to the newly created user
USER $USERNAME



