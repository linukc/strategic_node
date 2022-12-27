#!/usr/bin/env python3
'''
3 floor -> 1 floor
'''

import rospy
import smach
from smach_ros import SimpleActionState, ServiceState

from std_msgs.msg import String
import communication_msgs.msg as Action
import communication_msgs.srv as Service


metadata = {"third_floor_lift_coord": {"x": 12, "y": 20.8},
            "322_door_type": "right",
            "322_door_action": "push",
            "button_sign_in_lift_hall": 'down',
            "button_position_in_lift_hall": 0,
            "floor_sign_in_lift": '1',
            "floor_sign_position_in_lift": 1,
            "first_floor_coord": {"x": -10, "y": -20}}

class Lift_door_bottleneck(smach.State):
    '''Lift_door_bottleneck state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['lift_passing'])

    def execute(self, ud):
        rospy.loginfo('Executing state Lift_door_bottleneck')
        rospy.wait_for_message('/lift_door_is_opened', String)
        return 'lift_passing'

# main
def main():
    '''main'''
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    state_machine = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with state_machine:

        smach.StateMachine.add('door_coord_ACTION_1',
                        SimpleActionState('door_coord',
                            Action.DoorCoordAction,
                            goal=Action.DoorCoordGoal(x1=-0.95, y1=2.37, x2=-0.08, y2=2.775)),   
                        transitions={'succeeded': 'navigation_ACTION_2',
                                        'preempted': 'finish',
                                        'aborted': 'finish'})

        smach.StateMachine.add('navigation_ACTION_1',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2DAction,
                                    goal=Action.Navigate2DGoal(goal="parkingin")),
                                transitions={'succeeded': 'open_the_door_SERVICE',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('open_the_door_SERVICE',
                                ServiceState('OpenDoor',
                                    Service.OpenDoor,
                                    request = Service.OpenDoorRequest(
                                        door_type=metadata.get("322_door_type"),
                                        action_type=metadata.get("322_door_action"))),
                                transitions={'succeeded': 'navigation_ACTION_2',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('navigation_ACTION_2',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2DAction,
                                    goal=Action.Navigate2DGoal(goal="opendoor")),
                                transitions={'succeeded': 'detach_gripper_SERVICE',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('detach_gripper_SERVICE',
                                ServiceState('DetachGripper',
                                    Service.DetachGripper),
                                transitions={'succeeded': 'navigation_ACTION_3',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('navigation_ACTION_3',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2DAction,
                                    goal=Action.Navigate2DGoal(goal="gotocoridour")),
                                transitions={'succeeded': 'move_to_point_ACTION_1',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('move_to_point_ACTION_1',
                                        SimpleActionState('move_to_point',
                                            Action.MoveToPointAction,
                                            goal=Action.MoveToPointGoal(x = 12 , y=20.8 ),
                                            result_slots=["x1", "y1", "x2", "y2"]),
                                        transitions={'succeeded': 'push_button_SERVICE',
                                                    'preempted': 'finish',
                                                    'aborted': 'move_to_point_ACTION_1'})

        smach.StateMachine.add('push_button_SERVICE',
                                        ServiceState('/elev/push_button',
                                            Service.PushButton,
                                            request = Service.PushButtonRequest(
                                                lmr=metadata.get("near_elevator_sign_pos"),
                                                symbol=metadata.get("down_sign_lift"))),
                                        transitions={'succeeded': 'door_coord_ACTION_2',
                                                    'preempted': 'finish',
                                                    'aborted': 'finish'})

        smach.StateMachine.add('door_coord_ACTION_2',
                                SimpleActionState('door_coord',
                                    Action.DoorCoordAction,
                                    goal_slots=["x1", "y1", "x2", "y2"]),
                                transitions={'succeeded': 'navigation_ACTION_4',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('navigation_ACTION_4',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2DAction,
                                    goal=Action.Navigate2DGoal(goal="liftparking")),
                                transitions={'succeeded': 'push_button_SERVICE_2',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('push_button_SERVICE_2',
                                ServiceState('PushButton',
                                    Service.PushButton,
                                    request = Service.PushButtonRequest(
                                        lmr=metadata.get("button_position_in_lift_hall"),
                                        symbol=metadata.get("button_sign_in_lift_hall"))),
                                transitions={'succeeded': 'navigation_ACTION_5',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('navigation_ACTION_5',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2DAction,
                                    goal=Action.Navigate2DGoal(goal="liftpassing")),
                                transitions={'succeeded': 'push_button_SERVICE',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('push_button_SERVICE',
                                ServiceState('PushButton',
                                    Service.PushButton,
                                    request = Service.PushButtonRequest(
                                        lmr=metadata.get("floor_sign_position_in_lift"),
                                        symbol=metadata.get("floor_sign_in_lift"))),
                                transitions={'succeeded': 'finish',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('Lift_door_bottleneck_2',
                                Lift_door_bottleneck(),
                                transitions={'lift_passing':'navigation_ACTION_6'})

        smach.StateMachine.add('navigation_ACTION_6',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2DAction,
                                    goal=Action.Navigate2DGoal(goal="liftpassing")),
                                transitions={'succeeded': 'move_to_point_ACTION_2',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('move_to_point_ACTION_2',
                                SimpleActionState('move_to_point',
                                    Action.MoveToPointAction,
                                    goal=Action.MoveToPointActionGoal(**metadata.get("first_floor_coord")),
                                transitions={'succeeded': 'finish',
                                             'preempted': 'finish',
                                             'aborted': 'finish'}))

    # Execute SMACH plan
    state_machine.execute()

if __name__ == '__main__':
    main()
