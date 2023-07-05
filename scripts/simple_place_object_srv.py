from communication_msgs.srv import PlaceObject, PlaceObjectResponse
import rospy
import time

def place_object(req):
    print("Returning response")
    time.sleep(3)
    return PlaceObjectResponse(result="result")

def place_server():
    rospy.init_node('place_object_server')
    s = rospy.Service('place_object', PlaceObject, place_object)
    print("Ready to place object.")
    rospy.spin()

if __name__ == "__main__":
    place_server()