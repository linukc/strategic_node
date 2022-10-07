#!/usr/bin/env python3
'''
state machine
'''

import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
import communication_msgs.msg as Action
from std_msgs.msg import String

# define state Foo
class Foo(smach.State):
    '''Foo state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, ud):
        rospy.loginfo('Executing state FOO')
        #js = rospy.wait_for_message('/requestMotion01', String)
        #rospy.loginfo(js.data)
        return 'outcome1'


# main
def main():
    '''main'''
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    state_machine = smach.StateMachine(outcomes=['outcome2']) # (['succeeded','aborted','preempted']) 

    # Open the container
    with state_machine:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),
                               transitions={'outcome1':'ACTION'})

        smach.StateMachine.add('ACTION',
                         SimpleActionState('tx2_action_server',
                            Action.OpenDoorAction,
                            goal=Action.OpenDoorGoal(x=-87, y=12)),
                         transitions={'succeeded':'outcome2',
                                      'aborted': 'outcome2',
                                      'preempted': 'outcome2'})

    # asw = ActionServerWrapper('my_action_server_name', MyAction,
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
