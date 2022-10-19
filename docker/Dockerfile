FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
RUN chmod 1777 /tmp

RUN apt-get update && \
    apt-get -y install --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends git \
                       python3-pip \
                       ros-noetic-smach-viewer && \
    rm -rf /var/lib/apt/lists/*

ARG USER=docker_user
ARG UID=1000
ARG GID=1000
# default password
ARG PW=docker_user

# Add user and his password
RUN useradd -m ${USER} --uid=${UID} \
    && echo "${USER}:${PW}" | chpasswd \
    && usermod -s /bin/bash ${USER} \
    && usermod -a -G sudo ${USER}

RUN pip3 install pre-commit pylint && \
    echo PATH=$PATH:/home/docker_user/.local/bin >> ~/.bashrc

RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-noetic-actionlib-tools && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install flask

WORKDIR /home/${USER}
