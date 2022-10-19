#!/usr/bin/env python3
'''
state machine for offline scenario (not general enviroment -> 525 room)
1) Move to point (MoveToPoint.action)
2) Controller->parking (Navigate2D.action)
3) Open the Door (OpenDoor.srv)
4) Move with the door (Navigate2D.action)
5) Detach gripper (DetachGripper.srv)
6) Go trought the door (Navigate2D.action)
'''

import rospy
import smach
from smach_ros import SimpleActionState, ServiceState

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import communication_msgs.msg as Action
import communication_msgs.srv as Service

# define bottleneck state
class BottleNeck(smach.State):
    '''BottleNeck state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_to_point'])

    def execute(self, ud):
        rospy.loginfo('Executing state BottleNeck')
        command = rospy.wait_for_message('/commands_from_dream', String)
        rospy.loginfo(command.data)
        return 'move_to_point'

# main
def main():
    '''main'''
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    state_machine = smach.StateMachine(outcomes=['finish'])
    # (['succeeded','aborted','preempted'])

    # Open the container
    with state_machine:
        # Add states to the container
        smach.StateMachine.add('BottleNeck',
                                BottleNeck(),
                                transitions={'move_to_point':'move_to_point_ACTION'})

        smach.StateMachine.add('move_to_point_ACTION',
                                SimpleActionState('tx2_action_server',
                                    Action.MoveToPoint,
                                    goal=Action.MoveToPoineGoal(x=-87, y=-12)),
                                    transitions={'succeeded': 'navigation_ACTION_1',
                                                 'preempted': 'finish',
                                                 'aborted': 'finish'})

        smach.StateMachine.add('navigation_ACTION_1',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2D,
                                    goal=Action.Navigate2DGoal(goal="makeparking")),
                                    transitions={'succeeded': 'open_the_door_SERVICE',
                                                 'preempted': 'finish',
                                                 'aborted': 'finish'})

        smach.StateMachine.add('open_the_door_SERVICE',
                                ServiceState('OpenDoor',
                                    Service.OpenDoor,
                                    request = Service.OpenDoor('right',
                                                               'pull',
                                                                PointStamped(),
                                                               'some debug string')),
                                    transitions={'succeeded': 'navigation_ACTION_2'})

        smach.StateMachine.add('navigation_ACTION_2',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2D,
                                    goal=Action.Navigate2DGoal(goal="opendoor")),
                                    transitions={'succeeded': 'detach_gripper_SERVICE',
                                                 'preempted': 'finish',
                                                 'aborted': 'finish'})

        smach.StateMachine.add('detach_gripper_SERVICE',
                                ServiceState('DetachGripper',
                                    Service.DetachGripper),
                                    transitions={'succeeded': 'navigation_ACTION_3'})

        smach.StateMachine.add('navigation_ACTION_3',
                                SimpleActionState('navigate_2d',
                                    Action.Navigate2D,
                                    goal=Action.Navigate2DGoal(goal="gotoroom")),
                                    transitions={'succeeded': 'finish',
                                                 'preempted': 'finish',
                                                 'aborted': 'finish'})

    # Create and start the introspection server
    #sis = smach_ros.IntrospectionServer('my_smach_introspection_server', state_machine, '/SM_ROOT')
    #sis.start()

    # Execute SMACH plan
    state_machine.execute()

    # Wait for ctrl-c to stop the application
    #rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    main()
