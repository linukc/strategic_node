#!/usr/bin/env python3
'''
lift scenario without manipulator
'''
​
import rospy
import smach
from smach_ros import SimpleActionState, ServiceState
import communication_msgs.msg as Action
import communication_msgs.srv as Service
from std_msgs.msg import Bool, String
​
metadata = {"322_door_type": "right",
            "322_door_action": "push",
            "button_sign_in_lift_hall": 'down',
            "button_position_in_lift_hall": 0,
            "floor_sign_in_lift": '1',
            "floor_sign_position_in_lift": 1,
            "near_elevator_sign_pos" : 0,
            "down_sign_lift" : "down",
            "first_floor_coord": {"x": 24.5, "y": 18.6}}
​
class Lift_door_bottleneck(smach.State):
    '''Lift_door_bottleneck state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['lift_exit'])
​
    def execute(self, ud):
        rospy.loginfo('Executing state Lift_door_bottleneck')
        rospy.wait_for_message('/elev/door/open_event', Bool)
        return 'lift_exit'
​
class Speech_bottleneck(smach.State):
    ''' speech bottleneck state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['out'])
​
    def execute(self, ud):
        rospy.loginfo('Executing state speech bottleneck')
        rospy.wait_for_message('/asr_result', String)
        return 'out'
​
​
class Timeout(smach.State):
    '''timeout state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['out'])
​
    def execute(self, ud):
        rospy.loginfo('Executing state timeout')
        rospy.sleep(20)
        return 'out'
​
# main
def main():
    '''main'''
    rospy.init_node('smach_example_state_machine')
​
    # Create a SMACH state machine
    state_machine = smach.StateMachine(outcomes=['finish'])
​
    # Open the container
    with state_machine:
        # # smach.StateMachine.add('speech_bottleneck_1',
        # #                         Speech_bottleneck(),
        # #                         transitions={'out':'door_coord_ACTION_1'})
​
        smach.StateMachine.add('move_to_point_ACTION_1',
                                SimpleActionState('move_to_point',
                                    Action.MoveToPointAction,
                                    goal=Action.MoveToPointGoal(x = 12.98, y=-18.8 ),
                                    result_slots=["x1", "y1", "x2", "y2"]),
                                transitions={'succeeded': 'open_the_door_SERVICE',#'open_the_door_SERVICE',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
        smach.StateMachine.add('open_the_door_SERVICE',
                                ServiceState('OpenDoor',
                                    Service.OpenDoor,
                                    request = Service.OpenDoorRequest(
                                        door_type=metadata.get("322_door_type"),
                                        action_type=metadata.get("322_door_action"))),
                                transitions={'succeeded': 'move_to_point_ACTION_2',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})                                    
        smach.StateMachine.add('move_to_point_ACTION_2',
                                SimpleActionState('move_to_point',
                                    Action.MoveToPointAction,
                                    goal=Action.MoveToPointGoal(x = 12.4, y= -18.8 ),
                                    result_slots=["x1", "y1", "x2", "y2"]),
                                transitions={'succeeded': 'detach_gripper_SERVICE',#'detach_gripper_SERVICE',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
        smach.StateMachine.add('detach_gripper_SERVICE',
                                ServiceState('DetachGripper',
                                    Service.DetachGripper),
                                transitions={'succeeded': 'move_to_point_ACTION_3',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
        smach.StateMachine.add('move_to_point_ACTION_3',
                                SimpleActionState('move_to_point',
                                    Action.MoveToPointAction,
                                    goal=Action.MoveToPointGoal(x = 10.8 , y= -18.8 ),
                                    result_slots=["x1", "y1", "x2", "y2"]),
                                transitions={'succeeded': 'move_to_point_ACTION_4',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
        smach.StateMachine.add('move_to_point_ACTION_4',
                                SimpleActionState('move_to_point',
                                    Action.MoveToPointAction,
                                    goal=Action.MoveToPointGoal(x = 0.55, y= 0.15 ),
                                    result_slots=["x1", "y1", "x2", "y2"]),
                                transitions={'succeeded': 'push_button_SERVICE',  #push_button_SERVICE
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
​
        # # smach.StateMachine.add('Timeout1',
        # #                         Timeout(),
        # #                         transitions={'out':'push_button_SERVICE'})
​
​
​
​
        smach.StateMachine.add('push_button_SERVICE',
                                ServiceState('/elev/push_button',
                                    Service.PushButton,
                                    request = Service.PushButtonRequest(
                                        lmr=metadata.get("near_elevator_sign_pos"),
                                        symbol=metadata.get("down_sign_lift"))),
                                transitions={'succeeded': 'move_to_point_ACTION_5',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
​
        
        smach.StateMachine.add('move_to_point_ACTION_5',
                                 SimpleActionState('move_to_point',
                                     Action.MoveToPointAction,
                                     goal=Action.MoveToPointGoal(x = 1.98, y= -1.9 ),
                                     result_slots=["x1", "y1", "x2", "y2"]),
                                 transitions={'succeeded': 'push_button_SERVICE_2',
                                              'preempted': 'finish',
                                              'aborted': 'finish'})
    
        smach.StateMachine.add('push_button_SERVICE_2',
                                ServiceState('/elev/push_button',
                                    Service.PushButton,
                                    request = Service.PushButtonRequest(
                                        lmr=metadata.get("floor_sign_position_in_lift"),
                                        symbol=metadata.get("floor_sign_in_lift"))),
                                transitions={'succeeded': 'move_to_point_ACTION_6',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
​
        smach.StateMachine.add('move_to_point_ACTION_6',
                                SimpleActionState('move_to_point',
                                    Action.MoveToPointAction,
                                    goal=Action.MoveToPointGoal(x = 2.5, y= -2.05 ),
                                    result_slots=["x1", "y1", "x2", "y2"]),
                                transitions={'succeeded': 'Timeout1',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
​
        smach.StateMachine.add('Timeout1',
                                Timeout(),
                                transitions={'out':'move_to_point_ACTION_7'})
​
        # smach.StateMachine.add('Lift_door_bottleneck_1',
        #                         Lift_door_bottleneck(),
        #                         transitions={'lift_exit':'navigation_ACTION_7'})
​
        smach.StateMachine.add('move_to_point_ACTION_7',
                                SimpleActionState('move_to_point',
                                    Action.MoveToPointAction,
                                    goal=Action.MoveToPointGoal(x = -0.5, y= -1.5 ),
                                    result_slots=["x1", "y1", "x2", "y2"]),
                                transitions={'succeeded': 'move_to_point_ACTION_8',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
​
        smach.StateMachine.add('move_to_point_ACTION_8',
                                SimpleActionState('move_to_point',
                                    Action.MoveToPointAction,
                                    goal=Action.MoveToPointGoal(x = 7.5, y= 13.5 ),
                                    result_slots=["x1", "y1", "x2", "y2"]),
                                transitions={'succeeded': 'finish',
                                             'preempted': 'finish',
                                             'aborted': 'finish'})
​
​
​
        # smach.StateMachine.add('navigation_ACTION_10',
        #                         SimpleActionState('navigate_2d',
        #                             Action.Navigate2DAction,
        #                             goal=Action.Navigate2DGoal(goal="liftpassing")),
        #                         transitions={'succeeded': 'detach_gripper_SERVICE_2',
        #                                      'preempted': 'finish',
        #                                      'aborted': 'finish'})
​
        # smach.StateMachine.add('detach_gripper_SERVICE_2',
        #                         ServiceState('DetachGripper',
        #                             Service.DetachGripper),
        #                         transitions={'succeeded': 'finish',
        #                                      'preempted': 'finish',
        #                                      'aborted': 'finish'})
​
        # smach.StateMachine.add('navigation_ACTION_7',
        #                         SimpleActionState('navigate_2d',
        #                             Action.Navigate2DAction,
        #                             goal=Action.Navigate2DGoal(goal="liftexit")),
        #                         transitions={'succeeded': 'move_to_point_ACTION_2',
        #                                      'preempted': 'finish',
        #                                      'aborted': 'finish'})
                    
        # smach.StateMachine.add('move_to_point_ACTION_2',
        #                         SimpleActionState('move_to_point',
        #                             Action.MoveToPointAction,
        #                             goal=Action.MoveToPointGoal(**metadata.get("first_floor_coord"))),
        #                         transitions={'succeeded': 'finish',
        #                                      'preempted': 'finish',
        #                                      'aborted': 'finish'})
​
    # Execute SMACH plan
    state_machine.execute()
​
if __name__ == '__main__':
    main()
