#!/usr/bin/env python3

import time
import rclpy

from simple_node import Node

from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin import StateMachine

from communication_msgs.srv import PickObject, PlaceObject


def create_request_cb_pick(blackboard):
    request = PickObject.Request()
    request.what = "what"
    request.where = "where"
    return request

def create_request_cb_place(blackboard):
    request = PlaceObject.Request()
    request.where = "where"
    return request

class DemoNode(Node):

    def __init__(self):
        super().__init__("yasmin_node")

        # create a state machine
        sm = StateMachine(outcomes=[SUCCEED])

        # add states
        sm.add_state("pick_object_state",
                     ServiceState(self,
                                  PickObject,
                                  "pick_object",
                                  create_request_cb_pick),
                     {SUCCEED: "place_object_state"})
        sm.add_state("place_object_state",
                     ServiceState(self,
                                  PlaceObject,
                                  "place_object",
                                  create_request_cb_place),
                     {SUCCEED: SUCCEED})

        # execute
        outcome = sm()
        print(outcome)

# main
def main(args=None):

    print("yasmin_demo")
    rclpy.init(args=args)
    node = DemoNode()
    #node.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
