from communication_msgs.srv import PickObject, PickObjectResponse
import rospy
import time

def pick_object(req):
    print("Returning response")
    time.sleep(3)
    return PickObjectResponse(result="result")

def pick_server():
    rospy.init_node('pick_object_server')
    s = rospy.Service('pick_object', PickObject, pick_object)
    print("Ready to pick object.")
    rospy.spin()

if __name__ == "__main__":
    pick_server()