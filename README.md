Пример манипулирования объектом с помощью behaviour_tree.

### Установка

Пример для ros-noetic (смотри таблицу версий https://github.com/splintered-reality/py_trees_ros)

1) pip install git+https://github.com/splintered-reality/py_trees/@release/0.7.x
2) git clone -b release/0.3.x https://github.com/splintered-reality/py_trees_msgs/ to a project workspace
3) git clone https://github.com/ros-geographic-info/unique_identifier
3) pip install git+https://github.com/splintered-reality/py_trees_ros/@release/0.6.x
4) git clone -b behaviour_tree_example https://github.com/linukc/communication_msgs to a project workspace
5) pip install termcolor
6) clone move_base_msgs from https://github.com/ros-planning/navigation_msgs
7) (optionally) apt install ros-noetic-rqt-py-trees (https://github.com/splintered-reality/rqt_py_trees/tree/release/0.4.x)

### Особенности

Статус выполняющего процесса ACTIVE или PENDING приравнивается к RUNNING

(см. 
https://roslibpy.readthedocs.io/en/latest/_modules/roslibpy/actionlib.html

http://docs.ros.org/en/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html)
