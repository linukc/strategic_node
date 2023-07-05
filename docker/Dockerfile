FROM osrf/ros:foxy-desktop

ARG DEBIAN_FRONTEND=noninteractive
RUN chmod 1777 /tmp

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
        tzdata \
        nano \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y ros-noetic-desktop && rm -rf /var/lib/apt/lists/*

WORKDIR /home
