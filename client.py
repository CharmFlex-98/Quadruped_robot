import cv2
import zmq
import base64
import picamera
from picamera.array import PiRGBArray


class client:
    def __init__(self, width, height):
        self.IP=IP = '192.168.0.15'
        self.camera = picamera.PiCamera()
        self.camera.resolution = (width, height)
        self.camera.framerate = 30
        self.rawCapture = PiRGBArray(self.camera, size = (width,height))
        self.contest = zmq.Context()
        self.footage_socket = self.contest.socket(zmq.PUB)
        self.footage_socket.connect('tcp://{}:5555'.format(self.IP))
        print(IP)
        self.tracking_on=True


    def data_upload(self, frame):
        frame_image = frame.array
        encoded, buffer = cv2.imencode('.jpg', frame_image)
        jpg_as_test = base64.b64encode(buffer)
        self.footage_socket.send(jpg_as_test)
        self.rawCapture.truncate(0)

    def change_control_method(self):
        if self.tracking_on==False:
            self.tracking_on=True
            print(self.tracking_on)
            self.footage_socket.send_string('tracking on')
        else:
            self.tracking_on=False
            print(self.tracking_on)
            self.footage_socket.send_string('tracking off')