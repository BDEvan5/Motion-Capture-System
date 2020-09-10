import io
import socket
import struct
import time
import picamera
import logging
import multiprocessing as mp
from PIL import Image
import glob
import os

RIGHT = 'right'
LEFT = 'left'
STATIC = 'static'


class ClientSocket:
    def __init__(self, side, use='static', stream_number=1):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = '192.168.137.1'  # Enter the correct IP number of the server here.
        self.port = 8000
        self.side = side
        self.use = use
        self.camera = picamera.PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.rotation = 90
        self.camera.shutter_speed = 5000
        self.stream = io.BytesIO()

        logging.basicConfig(filename=('%sClientLogFile.log' % side), format='%(asctime)s %(message)s',
                            filemode='w')
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.INFO)

        self.stream_path = 'Stream%d/' % stream_number
        if not os.path.exists(self.stream_path):
            os.mkdir(self.stream_path)

        self.stream_list = []
        self.img_q = mp.Queue()

        self.filename = 'Stream_%s_%s.h264' % (self.use, self.side)

    def run_client_show(self):
        while True:
            print("client is searching")
            self.logger.info("client is searching")
            self.socket.connect((self.host, self.port))

            print('A connection is established: ' + str(self.socket.getsockname()))
            self.logger.info('A connection is established: ' + str(self.socket.getsockname()))
            self.receive_message()
            self.logger.info("Done receiving, restart")
            print("Done")

            self.socket.close()
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def receive_message(self):
        print("Receiving from: " + str(self.socket.getpeername()))
        self.logger.info("Receiving from: " + str(self.socket.getpeername()))
        while True:
            try:
                print("Waiting for data to receive")
                self.logger.info("Waiting for data to receive")
                msg = self.socket.recv(1024)
                msg.decode()
                msg = str(msg)
                msg = msg[2:len(msg) - 1]
                print(msg)
                self.logger.info(msg)
                self.process_msg(msg)
                if not msg:
                    break
            except Exception as e:
                print(e)
                print("msg exception")
                self.logger.info("msg exception")

    def process_msg(self, msg):
        self.logger.info("Checking msg")

        if msg == "t":
            msg_to_send = "Sending take"
            self.send_message(msg_to_send)
            self.send_single_take()
        elif msg == "f":
            self.send_single_take_fast()
        elif msg == "v":
            leng = self.socket.recv(1024)
            leng = leng.decode()
            leng = int(leng)
            print(leng)
            self.take_video(leng)
            self.send_video()
        elif msg == "i":
            msg_to_send = "This Pi is " + self.use + " " + self.side
            self.send_message(msg_to_send)

    def take_video(self, stream_length=5, framerate=5):
        self.camera.framerate = framerate
        self.camera.start_preview()
        time.sleep(1)
        print("Busy taking video")

        self.camera.start_recording(self.filename)
        self.camera.wait_recording(stream_length)
        self.camera.stop_recording()

    def send_video(self):
        self.send_message("Sending video")
        f = open(self.filename, 'rb')
        data = f.read(8192)
        while data:
            self.socket.send(data)
            print('Sent ')
            data = f.read(8192)
        f.close()

        print('Done sending')

    def send_single_take(self):
        # self.camera.resolution = (3280, 2464)
        stream = io.BytesIO()
        connection = self.socket.makefile('wb')
        self.camera.capture(stream, 'jpeg')
        print("Picture taken")
        leng = struct.pack('<L', stream.tell())
        connection.write(leng)
        connection.flush()
        stream.seek(0)
        connection.write(stream.read())
        print("Picture taken and sent: process complete")

    def send_message(self, msg):
        m = msg.encode('utf-8')
        self.socket.send(m)
        print("Message sent --> " + msg)
        self.logger.info("Message sent --> " + msg)
        time.sleep(2)
        # gives time for the message to be received


if __name__ == "__main__":
    myClient = ClientSocket(RIGHT, STATIC) # change this for different msgs
    myClient.run_client_show()


