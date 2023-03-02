import cv2
import numpy as np
import socket
from config import CONTROL_IP, CONTROL_PORT
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sk.settimeout(3000)

def set_control_ip():
    ip = socket.gethostbyname(socket.gethostname())
    control_msg = "SET_CONTROL_IP {}".format(ip).encode('ascii')
    sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))


def send_control(left_motor_speed, right_motor_speed):
    """Convert steering and throttle signals to a suitable format and send them to ESP32 bot"""
    control_msg = "CONTROL_WHEEL {} {}".format(
        left_motor_speed, right_motor_speed).encode('ascii')
    sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))


def calculate_control_signal(left_point, right_point, im_center):
    """Calculate control signal"""
    
    if left_point == None or right_point == None:
        left_motor_speed = right_motor_speed = 80
        return left_motor_speed, right_motor_speed

    # Calculate difference between car center point and image center point
    center_point = (right_point + left_point) // 2
    center_diff = center_point - im_center

    # Calculate steering angle from center point difference
    steering = -float(center_diff * 0.03)
    steering = min(1, max(-1, steering))
    throttle = 0.8
    
    # From steering, calculate left/right motor speed
    left_motor_speed = 0
    right_motor_speed = 0

    if steering > 0:
        left_motor_speed = throttle * (1 - steering)
        right_motor_speed = throttle 
    else:
        left_motor_speed = throttle
        right_motor_speed = throttle  * (1 + steering)
    

    left_motor_speed = int(left_motor_speed * 100)
    right_motor_speed = int(right_motor_speed * 100)
    diff = abs(left_motor_speed - right_motor_speed)
    if left_motor_speed < 80:
        left_motor_speed = 80
        right_motor_speed+= diff / 4
    if right_motor_speed < 80:
        right_motor_speed = 80
        left_motor_speed+= diff / 4
 
    return int(left_motor_speed), int(right_motor_speed)


def grayscale(img):
    """Convert image to grayscale"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def canny(img, low_threshold, high_threshold):
    """Apply Canny edge detection"""
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):
    """Apply a Gaussian blur"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def birdview_transform(img):
    """Get birdview image"""
    IMAGE_H = 160
    IMAGE_W = 320

    src = np.float32([[0, IMAGE_H], [320, IMAGE_H], [
        0, IMAGE_H // 3], [IMAGE_W, IMAGE_H // 3]])
    dst = np.float32([[90, IMAGE_H], [230, IMAGE_H],
                      [-10, 0], [IMAGE_W + 10, 0]])
    M = cv2.getPerspectiveTransform(src, dst)  # The transformation matrix
    warped_img = cv2.warpPerspective(
        img, M, (IMAGE_W, IMAGE_H))  # Image warping
    return warped_img


def preprocess(img):
    """Preprocess image to get a birdview image of lane lines"""

    img = grayscale(img)
    img = gaussian_blur(img, 3)
    img = canny(img, 100, 200)

    img = birdview_transform(img)

    return img

number_of_lines = 2 
preLeft = [-1] * number_of_lines
preRight = [-1] * number_of_lines

def fix_point(type, curLeft, curRight, lanewidth):
    left = curLeft
    right = curRight
    if preLeft[type] != -1 and preRight[type] != -1:
        if curLeft != -1 and curRight != -1:
            if curRight - curLeft >= lanewidth - 50:
                left = curLeft
                right = curRight
            else:
                curMin = abs(curLeft - preLeft[type])
                left = curLeft
                right = left + lanewidth
                
                if abs(curRight - preRight[type]) < curMin:
                    curMin = abs(curRight - preRight[type])
                    right = curRight
                    left = right - lanewidth
                
                if abs(curLeft - preRight[type]) < curMin:
                    curMin = abs(curLeft - preRight[type])
                    right = curLeft
                    left = right - lanewidth
                
                if abs(curRight - preLeft[type]) < curMin:
                    curMin = abs(curRight - preLeft[type])
                    left = curRight
                    right = left + lanewidth
                    
        else:
            if curLeft == curRight == -1:
                left = preLeft[type]
                right = preRight[type]
            elif curLeft == -1 or curRight == -1:
                if curLeft == -1:
                    if (abs(curRight - preRight[type])) <= abs((curRight - preLeft[type])):
                        right = curRight
                        left = right - lanewidth
                    else:
                        left = curRight
                        right = left + lanewidth
                else:
                    if (abs(curLeft - preLeft[type])) <= abs(curLeft - preRight[type]):
                        left = curLeft
                        right = left + lanewidth
                    else:
                        right = curLeft
                        left = right - lanewidth
            else:
                left = curLeft
                right = curRight
    else:
        if curLeft == curRight == -1:
            return -1, -1
        
        if curLeft == -1 or curRight == -1:
            if curRight == -1:
                left = curLeft
                right = left + lanewidth
            else:
                right = curRight
                left = right - lanewidth
            
    
    preLeft[type] = left
    preRight[type] = right
    
    return left, right

def find_lane_lines(image, draw=False):
    """Find lane lines from color image"""
    image = preprocess(image)
    im_height, im_width = image.shape[:2]

    if draw:
        viz_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    # Interested line to determine lane center
    global number_of_lines
    height_of_interested_lines = []
    lane_width = []
    height = 0.75
    width = 160
    
    for line in range(number_of_lines):
        height_of_interested_lines.append(int(im_height * height))
        lane_width.append(width)
        height-= 0.25        
        width+= 16
        
    if draw:
        for height in height_of_interested_lines:
            cv2.line(viz_img, (0, height), (im_width, height), (0, 0, 255), 2)
    
    interested_lines = []
    for height in height_of_interested_lines:
        interested_lines.append(image[height, : ])
    

    # Determine left point and right point
    left_points = [-1] * number_of_lines
    right_points = [-1] * number_of_lines

    center = im_width // 2

    for it in range(number_of_lines):
        for x in range(center, 0, -1):
            if interested_lines[it][x] > 0:
                left_points[it] = x
                break
        for x in range(center + 1, im_width):
            if interested_lines[it][x] > 0:
                right_points[it] = x
                break
        left_points[it], right_points[it] = fix_point(it, left_points[it], right_points[it], lane_width[it])
         
    if draw:
        if left_points[0] != -1:
            viz_img = cv2.circle(viz_img, (left_points[0], height_of_interested_lines[0]), 7, (255, 255, 0), -1)
        if left_points[1] != -1:
            viz_img = cv2.circle(viz_img, (left_points[1], height_of_interested_lines[1]), 7, (255, 255, 0), -1)
        if right_points[0] != -1:
            viz_img = cv2.circle(viz_img, (right_points[0], height_of_interested_lines[0]), 7, (0, 255, 0), -1)
        if right_points[1] != -1:
            viz_img = cv2.circle(viz_img, (right_points[1], height_of_interested_lines[1]), 7, (0, 255, 0), -1)
 
    if draw:
        return number_of_lines, left_points, right_points, center, viz_img
    else:
        return number_of_lines, left_points, right_points, center