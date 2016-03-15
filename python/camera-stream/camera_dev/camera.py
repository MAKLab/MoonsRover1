from time import time


class Camera(object):
    """An emulated camera implementation that streams a repeated sequence of
    files 1.jpg, 2.jpg and 3.jpg at a rate of one frame per second."""

    def __init__(self):
        print("Reading example frames...")
        self.frames = [open('camera_dev/' + f + '.jpg', 'rb').read() for f in ['1', '2', '3']]
        print("... done!")

    def get_frame(self):
        i = int(time()) % 3
        #print("Get frame {}".format(i))
        return self.frames[i]
