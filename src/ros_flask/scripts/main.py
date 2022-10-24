#!/usr/bin/env python3
'''
communication node between ros and dream
'''
import threading
import rospy
from std_msgs.msg import String

from flask import Flask, request
from flask import jsonify

app = Flask(__name__)

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.

pub = rospy.Publisher('/commands_from_dream', String, queue_size=1)
threading.Thread(target=lambda: rospy.init_node('test_node_flask', disable_signals=True)).start()

@app.route('/robot_status', methods = ['GET'])
def default():
    '''return robot_status'''
    return jsonify({"status": "free"})


@app.route('/upload_intent', methods = ['POST'])
def info():
    '''intercept commands from dream intent catcher'''
    rospy.loginfo(str(request.data))
    pub.publish("go to 525 room")
    return jsonify({"status": "ok"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
