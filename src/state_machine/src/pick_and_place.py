#!/usr/bin/env python3

import rospy
import smach
from smach_ros import SimpleActionState, ServiceState
import communication_msgs.msg as Action
import communication_msgs.srv as Service


def main():
    rospy.init_node('pick_and_place_state_machine')
    state_machine = smach.StateMachine(outcomes=['finish'])

    with state_machine:
        smach.StateMachine.add('move_to_point_ACTION_1',
                                SimpleActionState('move_to_point_with_orientation',
                                    Action.MoveToPointWithOrientationAction,
                                    goal=Action.MoveToPointWithOrientationGoal(
                                        x=-2.35, y=-6.85, theta=0, use_orientation=False)),
                                    transitions={'succeeded': 'pick_object_SERVICE',
                                                 'preempted': 'finish',
                                                 'aborted': 'finish'})

        request = Service.PickObjectRequest(what="any_object", where="any_place")
        smach.StateMachine.add('pick_object_SERVICE',
                                ServiceState('pick_object',
                                    Service.PickObject,
                                    request=request),
                                transitions={'succeeded': 'move_to_point_ACTION_2',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('move_to_point_ACTION_2',
                                SimpleActionState('move_to_point_with_orientation',
                                    Action.MoveToPointWithOrientationAction,
                                    goal=Action.MoveToPointWithOrientationGoal(
                                        x=0, y=0, theta=1.57, use_orientation=True)),
                                transitions={'succeeded': 'place_object_SERVICE',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
        
        request = Service.PlaceObjectRequest(where="any_place")
        smach.StateMachine.add('place_object_SERVICE',
                                ServiceState('place_object',
                                    Service.PlaceObject,
                                    request=request),
                                transitions={'succeeded': 'finish',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

    state_machine.execute()

if __name__ == '__main__':
    main()
