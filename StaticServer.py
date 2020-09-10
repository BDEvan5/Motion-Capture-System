import socket
import cv2 as cv
import numpy as np
import time
import os
import multiprocessing
import struct
import io

STATIC = 'static'


class Server:
    def __init__(self, session=1, stream=1, calib=1):
        self.host = '192.168.137.1'
        self.port = 8000
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connections = []

        self.image_size = (640, 480)
        self.pattern_size = (8, 6)
        self.session_number = session
        self.stream_number = stream
        self.calib_num = calib

        self.session_path = 'StaticServerSession%d/' % session
        if not os.path.exists(self.session_path):
            os.mkdir(self.session_path)

        self.calibration_dp = self.session_path + "staticCalibrationData%d/" % calib
        if not os.path.exists(self.calibration_dp):
            os.mkdir(self.calibration_dp)

        self.triangulation_dp = self.session_path + "StreamData%d/" % stream
        if not os.path.exists(self.triangulation_dp):
            os.mkdir(self.triangulation_dp)

    def run_server(self):
        self.set_up_connection(2)
        self.run_range_check()
        self.take_calibration_imgs(20, 2)
        self.take_video(20)

    def set_up_connection(self, number_of_connections=2):
        # the point of this function is to allow 4 devices to connect then break
        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        print("Ready to roll: start client now")
        while True:
            conn, addr = self.socket.accept()
            connection = SocketConn(conn, addr[0], addr[1])
            self.connections.append(connection)
            connection_msg = "Connection has been established by: " + str(addr[0]) + " : " + str(addr[1])

            self.send_set_msg(connection_msg)

            if len(self.connections) == number_of_connections:
                print(str(number_of_connections) + " Connections have been made, warm up threads")
                time.sleep(4)
                print("Threads are hot --> lets go....")
                # this sleep is so the threads can be set up properly
                break
        self.identify_pi()

    def identify_pi(self):
        self.send_set_msg("i")
        time.sleep(4)
        for conn in self.connections:
            conn.name = conn.name_q.get()
            conn.use = conn.name_q.get()
            conn.side = conn.name_q.get()
            print("The following Pi has been identified as: " + conn.name)
        print("Queues have been assigned for connections")
        return 1

    def send_set_msg(self, msg):
        msg = msg.encode('utf-8')
        for conn in self.connections:
            conn.socket.send(msg)

        info = "Msg send -> : " + " : " + msg.decode()
        print(info)

    def run_range_check(self):
        while True:
            print("Checking img")
            self.send_set_msg("t")
            time.sleep(4)
            left_img, right_img = self.get_img_pair(9999, self.session_path)

            disp_img = np.concatenate((left_img, right_img), axis=1)
            scaling_factor = 1
            size = (int(disp_img.shape[1] / scaling_factor), int(disp_img.shape[0] / scaling_factor))
            disp_img = cv.resize(disp_img, size)
            cv.imshow("CheckImg", disp_img)
            cv.waitKey(4000)
            cv.destroyAllWindows()
            i = input("Image good? y/n --> ")
            if i == "y":
                break

    def take_calibration_imgs(self, range_to_take, time_between=2):
        print("Taking calibration images")
        for counter in range(range_to_take):
            print("Taking image: %d" % counter)
            self.send_set_msg("t")
            time.sleep(time_between)
            left_img, right_img = self.get_img_pair()

            cv.imwrite(self.calibration_dp + 'Left%d.jpg' % counter, left_img)
            cv.imwrite(self.calibration_dp + 'Right%d.jpg' % counter, right_img)
            disp = np.concatenate((left_img, right_img), axis=1)
            cv.imwrite(self.calibration_dp + 'JoinedDisplay%d.jpg' % counter, disp)
        print("Calibration images taken and received")
        cv.destroyAllWindows()

    def get_img_pair(self):
        for conn in self.connections:
            if conn.side == 'left':
                img_left = conn.q.get()
            elif conn.side == 'right':
                img_right = conn.q.get()

        return img_left, img_right

    def take_video(self, length=10):
        leng = "%s" % length
        print("Running Video")
        self.send_set_msg("v", None)
        self.send_set_msg(leng, None)
        time.sleep(1.5 * length)
        #TODO:this needs to be done differently somehow, it needs to know when to stop

        self.move_videos_taken()

    def move_videos_taken(self):
        print("Moving videos")
        static_right_name = 'static rightStreamedVideo.h264'
        video_right_taken = cv.VideoCapture(static_right_name)
        static_left_name = 'static leftStreamedVideo.h264'
        video_left_taken = cv.VideoCapture(static_left_name)

        code = cv.VideoWriter_fourcc('M', 'J', 'P', 'G')
        framerate = 10
        resolution = (620, 480)
        resolution2 = (resolution[0] * 2, resolution[1])

        name = self.triangulation_dp + 'CombinedVideo%d.avi' % self.stream_number
        video_combined = cv.VideoWriter(name, code, framerate, resolution2)
        name = self.triangulation_dp + 'LeftVideo%dClean.avi' % self.stream_number
        video_left_clean = cv.VideoWriter(name, code, framerate, resolution)
        name = self.triangulation_dp + 'RightVideo%dClean.avi' % self.stream_number
        video_right_clean = cv.VideoWriter(name, code, framerate, resolution)

        counter = 0
        while True:
            ret_l, img_static_left = video_left_taken.read()
            ret_r, img_static_right = video_right_taken.read()
            if ret_l is True and ret_r is True:
                video_left_clean.write(img_static_left)
                video_right_clean.write(img_static_right)

                joined_img = np.concatenate((img_static_left, img_static_right), axis=1)
                video_combined.write(joined_img)
                cv.imshow("Static Img Set", joined_img)
                cv.waitKey(1)

                counter += 1
            else:
                break
        print("finished moving videos")

        video_combined.release()
        video_right_clean.release()
        video_left_clean.release()


class SocketConn:
    def __init__(self, connection, ip, address):
        self.name = "Not set"
        self.use = ""
        self.side = ""
        self.address = address
        self.socket = connection
        self.ip = ip
        self.name_q = multiprocessing.Queue()
        self.error_counter = 0
        self.filename = ""

        self.q = multiprocessing.Queue()
        self.vid_q = multiprocessing.Queue()
        self.error_flag = False
        self.set_up_connection()

    def set_up_connection(self):
        connection_message = "Connection has been established by: " + str(self.ip) + " : " +str(self.address)
        # print(connection_message)
        recv_msg = multiprocessing.Process(target=self.receive_msg_process)
        recv_msg.start()

        # self.send_set_msg(connection_message)

    def receive_msg_process(self):
        print("Ready to receive from: " + str(self.address))
        while True:
            if self.error_flag is True:
                break
            msg = self.socket.recv(2048)
            try:
                msg.decode()
            except Exception as e:
                print(e)
                print("Unable to decode")
                self.report_error("%s : problem receiving message - unable to decode" % self.name)
            msg = str(msg)
            msg = msg[2:len(msg) - 1]

            print(str(self.name) + " : Message received -> " + str(msg))
            # self.mp_logger.info(str(name) + " : Message received -> " + str(msg))
            if len(msg) == 0:
                print("breaking")
            else:
                self.process_msg(msg)

    def process_msg(self, msg):
        flag = True
        if msg == "Sending take":
            self.receive_img()
        elif msg == "Sending video":
            self.get_video_from_client()
        else:
            if msg == "This Pi is static left":
                side = "left"
                use = "static"
            elif msg == "This Pi is static right":
                side = "right"
                use = "static"
            else:
                print("Message has not been recognised")
                flag = False

            #TODO: change this to local variables that get queued
            if flag is True:
                name = use + " " + side
                self.name_q.put(name)
                self.name_q.put(use)
                self.name_q.put(side)

                self.name = name
                self.use = use
                self.side = side
                # note, this only sets the variables in side the process, they are gloablly set via the q
                # print("The name has been set: " + self.name)

    def receive_img(self):
        connection = self.socket.makefile('rb')
        time.sleep(0.1)

        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        image_stream = io.BytesIO()
        stream_data = connection.read(image_len)
        image_stream.write(stream_data)
        image_stream.seek(0)
        file_bytes = np.asarray(bytearray(image_stream.read()), dtype=np.uint8)
        img = cv.imdecode(file_bytes, cv.IMREAD_COLOR)
        self.q.put(img)

    def get_video_from_client(self):
        BUFFER = 32768
        self.filename = self.name + 'StreamedVideo.h264'
        start_time = time.time()
        with open(self.filename, 'wb') as f:
            print('file opened: moving to receive data')
            while True:
                data = self.socket.recv(BUFFER)
                if data == b"DONE":
                    print("Data finished sending")
                    break
                f.write(data)
        end_time = time.time()
        f.close()
        print('Successfully got the file: %f' %(end_time - start_time))


        






