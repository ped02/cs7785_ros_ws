ARG FROM_IMAGE=ros:humble-perception-jammy

FROM $FROM_IMAGE

RUN . /opt/ros/humble/setup.sh

## Update and Upgrade
RUN apt update && apt upgrade -y

RUN apt install iputils-ping net-tools -y

## Install ROS Components
RUN apt install ros-humble-rqt* -y

RUN apt install ros-humble-rviz2 -y

RUN apt install ros-dev-tools -y

## Install Gazebo
# RUN apt install ros-humble-gazebo-* -y
RUN apt install ros-humble-ros-gz -y

## Install ROS2 Utilities
RUN apt install ros-humble-image-transport-plugins

## Opencv Utils
RUN apt install -y libcanberra-gtk-module libcanberra-gtk3-module

# Cartographer
RUN apt install -y ros-humble-cartographer

## Remove apt list
RUN rm -rf /var/lib/apt/lists/*

## Setup turtlebot package
RUN . /opt/ros/humble/setup.sh && \
    mkdir -p /turtlebot3_ws/src && \
    cd /turtlebot3_ws/src && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
    # && \
    # cd /turtlebot3_ws && \
    # rosdep update && \
    # rosdep install -i --from-path src --rosdistro humble -y && \
    # colcon build --symlink-install

## Copy Setup Files
WORKDIR /
COPY setup  /setup

RUN echo "source /setup/entrypoint.sh" >> /root/.bashrc

## Dev container arguments (https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/)
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

## Dev Packages
RUN apt update && apt install gdb gdbserver nano -y

# Gazebo (Optional)

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
apt update && \
apt install gz-harmonic -y

ENV GZ_VERSION harmonic
RUN . /opt/ros/humble/setup.sh && \
    mkdir -p /gz_ros_ws/src && \
    cd /gz_ros_ws/src && \
    git clone -b humble https://github.com/gazebosim/ros_gz.git && \
    cd /gz_ros_ws && \
    rosdep install -r --from-paths src -i -y --rosdistro humble && \
    colcon build --symlink-install

# RUN . /opt/ros/humble/setup.sh && \
#     cd /turtlebot3_ws/src && \
#     # git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
#     git clone -b new_gazebo https://github.com/azeey/turtlebot3_simulations.git && \
#     cd /turtlebot3_ws && \
#     colcon build --symlink-install

# # VSs code
# # RUN curl -fsSL https://code-server.dev/install.sh | sh
# RUN curl -Lk 'https://code.visualstudio.com/sha/download?build=stable&os=cli-alpine-x64' --output vscode_cli.tar.gz && tar -xf vscode_cli.tar.gz


# RUN code-server --install-extension ms-python.debugpy
# RUN code-server --install-extension ms-python.python
# # RUN code-server --install-extension ms-python.vscode-pylance
# RUN code-server --install-extension ms-vscode.cmake-tools
# RUN code-server --install-extension ms-vscode.cpptools
# RUN code-server --install-extension ms-vscode.cpptools-extension-pack
# RUN code-server --install-extension ms-vscode.cpptools-themes

RUN groupadd --gid $GID $USERNAME \
    && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME}

RUN groupadd --gid 110 render \
    && usermod -aG video ${USERNAME} \
    && usermod -aG 110 ${USERNAME}

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}

RUN . /opt/ros/humble/setup.sh && \
#     mkdir -p /turtlebot3_ws/src && \
    cd /turtlebot3_ws && \
#     git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
#     git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
#     git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git && \
#     cd /turtlebot3_ws && \
    rosdep update

USER root
RUN . /opt/ros/humble/setup.sh && \
    cd /turtlebot3_ws && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build --symlink-install

RUN echo "source /setup/entrypoint.sh" >> /home/${USERNAME}/.bashrc

# Entry
SHELL ["/bin/bash"]
ENV SHELL /bin/bash
ENTRYPOINT [ "/setup/entrypoint.sh" ]
CMD [ "bash" ]
