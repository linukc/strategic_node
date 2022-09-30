FROM osrf/ros:noetic-desktop-full

ARG USER=docker_user
ARG UID=1000
ARG GID=1000
# default password
ARG PW=docker_user

# Add user and his password
RUN useradd -m ${USER} --uid=${UID} \
    && echo "${USER}:${PW}" | chpasswd \
    && usermod -s /bin/bash ${USER}

WORKDIR /home/${USER}
