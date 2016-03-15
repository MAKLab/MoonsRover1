#!/usr/bin/env python
import json
from flask import Flask, render_template, Response, request

try:
    from camera_pi import Camera
except ImportError:
    print("camera_pi module not found, falling back to camera_dev")
    from camera_dev import Camera

from rover_interface import RoverInterface

# Create our Flask app
app = Flask(__name__)

# Create the rover interface
rover = RoverInterface()

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/rover/api/v1.0/instructions', methods=['PUT'])
def postInstructions():
    """Instruction posting route."""
    # Check we got some kind of JSON
    if not request.json:
        return json.dumps({'success': False, 'error':'No instructions received'}), 400

    # Check the instructions
    if not rover.validateInstructions(request.json):
        return json.dumps({'success': False, 'error':'Invalid instructions'}), 400

    # Give the rover the instructions
    rover.addValidInstructions(request.json)
    
    return json.dumps({'success': True}), 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, threaded=True)
