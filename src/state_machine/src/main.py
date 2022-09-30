#!/usr/bin/env python3
'''
state machine
'''

import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
    '''Foo state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, ud):
        rospy.loginfo('Executing state FOO')
        if self.counter < 30:
            self.counter += 1
            return 'outcome1'
        return 'outcome2'


# define state Bar
class Bar(smach.State):
    '''Bar state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, ud):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'

# main
def main():
    '''main'''
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    state_machine = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with state_machine:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),
                               transitions={'outcome1':'BAR',
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'outcome2':'FOO'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', state_machine, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    state_machine.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
