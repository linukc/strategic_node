#!/usr/bin/env python3
'''
lift scenario without manipulator
'''

import rospy
import smach
from smach_ros import SimpleActionState
import communication_msgs.msg as Action

# main
def main():
    '''main'''
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    state_machine = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with state_machine:

        smach.StateMachine.add('door_coord_ACTION_2',
                                SimpleActionState('door_coord',
                                    Action.DoorCoordAction,
                                    goal=Action.DoorCoordGoal(x1=57.7, y1=-12.5, x2=56.7, y2=-12.5)),
                                transitions={'succeeded': 'navigation_ACTION_4',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('navigation_ACTION_4',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2DAction,
                                    goal=Action.Navigate2DGoal(goal="liftparking")),
                                transitions={'succeeded': 'navigation_ACTION_5',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('navigation_ACTION_5',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2DAction,
                                    goal=Action.Navigate2DGoal(goal="liftpassing")),
                                transitions={'succeeded': 'navigation_ACTION_6',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

        smach.StateMachine.add('navigation_ACTION_6',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2DAction,
                                    goal=Action.Navigate2DGoal(goal="liftexit")),
                                transitions={'succeeded': 'finish',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})

    # Execute SMACH plan
    state_machine.execute()

if __name__ == '__main__':
    main()
