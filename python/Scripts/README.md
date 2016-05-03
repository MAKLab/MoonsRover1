# shutdown_pi.py

Shut down the raspberry pi when a button connected to pin 21 is pressed.
Connect a momentary push-button between 21 and ground. Internal pull-up is enabled so no other hardware is required.

How to set up so this python script is run when Raspberry Pi is started:

> sudo nano /etc/rc.local

This file is what gets executed everytime your RPi boots up. We need to add our python command before the last line which closes the if loop. Therefore, add the following line before the #fi at the end of the file.

> sudo python /home/pi/MoonsRover1/python/Scripts/shutdown_pi.py &

Please note that everything is case sensitive so Scripts is not the same as scripts. The & at the end of the command tells it to run the process in the background. If you omit it, then your login prompt probably will not appear.

... while we're at it why not add a line to start the camera app.py too:

> sudo python /home/pi/MoonsRover1/python/camera-stream/app.py &
