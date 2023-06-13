## Experiment

### Desciption

1) Берем объект (pick_object)
2) Доезжаем с объектом до стола (move_to_point_with_orientation)
3) Кладем объектом на стол (place_object)

### Commands

```python
cd docker
./start.sh
./into.sh
```

(optionaly) clone to src proper experiment branch from [communication_msgs](https://github.com/linukc/communication_msgs)

```python
catkin_make
source devel/setup.bash
cd src/state_machine/src/
python pick_and_place.py
```