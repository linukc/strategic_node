# ROS1 (melodic / noetic) - ROS2 (foxy) bridge with ROS2 State Machine.

### Installation

1. Clone this repo
2. Build docker
```bash
./docker/build.sh
./docker/start.sh
```
You can open whatever number of terminals you want with ```./docker/into.sh```

3. Install and update pip3
```bash
apt-get update
apt-get install -y python3-pip
python3 -m pip install --upgrade pip
```

4. Based on [this](https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html) tutorial add communication_msgs
```bash
mkdir -p ros1_msgs_ws/src
git clone -b pick_and_place_srv_only https://github.com/linukc/communication_msgs ros1_msgs_ws/src
mkdir -p ros2_msgs_ws/src
git clone -b pick_and_place_only_srv_ros2 https://github.com/linukc/communication_msgs ros2_msgs_ws/src
mkdir -p ros_bridge_ws/src
git clone -b foxy https://github.com/ros2/ros1_bridge
```

5. Build messages
```bash
source /opt/ros/noetic/setup.bash
cd /home/ros1_msgs_ws
catkin_make_isolated --install
source /opt/ros/foxy/setup.bash
cd /home/ros2_msgs_ws
colcon build --packages-select communication_msgs
```

6. Build bridge ~30 minutes
```bash
source /home/ros1_msgs_ws/install_isolated/setup.bash
source /home/ros2_msgs_ws/install/local_setup.bash
cd /home/ros_bridge_ws
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
source install/setup.bash
```
7. Run bridge with ```ros2 run ros1_bridge dynamic_bridge```

8. Install yasmin dependencies (suggesting to use another terminal)
```bash
source /opt/ros/foxy/setup.bash
cd /home/yasmin_state_machine_ws/src/
git clone https://github.com/uleroboticsgroup/simple_node.git
git clone https://github.com/uleroboticsgroup/yasmin.git
git clone -b pick_and_place_only_srv_ros2 https://github.com/linukc/communication_msgs
cd yasmin
pip3 install -r requirements.txt
cd /home/yasmin_state_machine_ws/
colcon build
source install/setup.bash
```

9. Run state machine as ```ros2 run yasmin_sm sm_talker```

10. (optionally) You can run on melodic/noetic host services from scripts folder to emulate server behaviour for the state machine.

### Notes

1. I use --net flag in docker to communicate with melodic roscore.
2. Do not forget to mount /shm volume inside docker for ros2 ([see](https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/)).
3. Inside docker default foxy ros2 version of ROS is sourced only.
4. Bridge will pass messages to a topic if only threre are both subsciber and publisher.
5. How to add custom messages in bridge - https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst
6. After build you can check ```ros2 run ros1_bridge dynamic_bridge --print-pairs``` for cusom messages