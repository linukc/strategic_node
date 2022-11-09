#!/usr/bin/env python3
'''
custom scenario
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
        smach.State.__init__(self, outcomes=['simple_commands'],
                                   output_keys=['command'])

    def execute(self, ud):
        rospy.loginfo('Executing state BottleNeck')
        command = rospy.wait_for_message('/commands_from_dream', String)
        rospy.loginfo(command.data)
        ud.command = command.data
        return 'simple_commands'

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
                                transitions={'simple_commands':'simple_commands_ACTION'})

        smach.StateMachine.add('simple_commands_ACTION',
                                SimpleActionState('simple_move_server',
                                    Action.SimpleCommandAction,
                                    goal_slots=["command"]),
                                    transitions={'succeeded': 'BottleNeck',
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
