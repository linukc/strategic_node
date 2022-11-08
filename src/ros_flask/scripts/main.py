#!/usr/bin/env python3
'''
communication node between ros and dream
'''
import threading
import rospy
from std_msgs.msg import String
import json

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

def postprocess_movetopoint(command):
    '''extract data from move_to_point_<some_data> template'''
    template = len("move_to_point_")
    return command[template:]

@app.route('/upload_response', methods = ['POST'])
def info():
    '''intercept commands from dream intent catcher'''
    data = str(json.loads(request.get_data()).get("text"))
    rospy.loginfo(data)

    if data.startswith("move_to_point"):
        data = postprocess_movetopoint(data)
        rospy.loginfo(data)

    pub.publish(str(mapping.get(data)))
    return jsonify({"status": "ok"})

if __name__ == '__main__':
    mapping = {"аудитория525": "-89 -12"}
    app.run(host='0.0.0.0', port=5000, debug=True)

# to do:
# 1) several words in go to
# 2) forms of key in mapping
