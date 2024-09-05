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

## Remove apt list
RUN rm -rf /var/lib/apt/lists/*

## Setup turtlebot package
RUN . /opt/ros/humble/setup.sh && \
    mkdir -p /turtlebot3_ws/src && \
    cd /turtlebot3_ws/src && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    cd /turtlebot3_ws && \
    colcon build --symlink-install

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

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /setup/entrypoint.sh" >> /home/${USERNAME}/.bashrc

# Entry
SHELL ["/bin/bash"]
ENV SHELL /bin/bash
ENTRYPOINT [ "/setup/entrypoint.sh" ]
CMD [ "bash" ]
