import socket
import _thread
import urllib.request
import cv2
import numpy as np
import time

from controller_1 import calculate_control_signal, send_control, set_control_ip, find_lane_lines


CONTROL_IP = "192.168.4.100"
CONTROL_PORT = 9999
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sk.settimeout(3000)

CAM_URL = "http://192.168.4.1:80"
stream = urllib.request.urlopen(CAM_URL)
bytes = bytes()


# Set control ip continuously to receive sensor params
def set_control_ip():
    time.sleep(2)
    set_control_ip()
_thread.start_new_thread(set_control_ip, ())


while True:
    bytes += stream.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        image = bytes[a:b + 2]
        bytes = bytes[b + 2:]
        try:
            image = cv2.imdecode(np.frombuffer(
                image, dtype=np.uint8), cv2.IMREAD_COLOR)
            image = cv2.resize(image,(320,160))
        except:
            continue
        
        cv2.imshow("Image", image)
        number_of_lines, left_points, right_points, im_center, draw = find_lane_lines(image, draw=True)
        cv2.imshow("Lane lines", draw)
        HaveNoneValue = False

        for value in left_points:
            if value == -1:
                HaveNoneValue = True
        for value in right_points:
            if value == -1:
                HaveNoneValue = True
        if HaveNoneValue == True:
            #send_control(80, 80)
            cv2.waitKey(1)
            continue

        left_point = (left_points[0] + left_points[1]) // 2
        right_point = (right_points[0] + right_points[1]) // 2
        print(left_points[0], left_points[1], right_points[0], right_points[1])
        left_motor_speed, right_motor_speed = calculate_control_signal(left_point, right_point, im_center) 
        print(left_motor_speed, right_motor_speed)
        send_control(left_motor_speed,right_motor_speed) 
        if cv2.waitKey(1) == 27:
            send_control(0,0)
            exit(0)