import pygame
import socket
import urllib.request
import cv2
import numpy as np

pygame.init()
pygame.font.init()
pygame.display.set_caption('VIABot controller')
window = pygame.display.set_mode((300, 300))
clock = pygame.time.Clock()

CONTROL_IP = "192.168.4.1"
CONTROL_PORT = 9999
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sk.settimeout(3000)

CAM_URL = "http://192.168.4.1:80"
stream = urllib.request.urlopen(CAM_URL)
bytes = bytes()

def set_speed(left_wheel, right_wheel):
    """Set speed for robot wheel

    Args:
        left_wheel (int): [-100, 100]. -100 -> 0: Reverse, 0 -> 100: Forward
        right_wheel (int): [-100, 100]. -100 -> 0: Reverse, 0 -> 100: Forward
    """
    control_msg = "CONTROL_WHEEL {} {}".format(
        left_wheel, right_wheel).encode('ascii')
    sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))

rect = pygame.Rect(0, 0, 20, 20)
rect.center = window.get_rect().center
origin_x = rect.x
origin_y = rect.y
run = True
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.KEYDOWN:
            print(pygame.key.name(event.key))

    keys = pygame.key.get_pressed()

    CONTROL_SPEED =  80  
    color = (255, 0, 0)
    if keys[pygame.K_UP] or keys[ord('w')]:
        rect.x = origin_x
        rect.y = origin_y - 50
        set_speed(CONTROL_SPEED, CONTROL_SPEED)
        color = (0, 0, 255)
    elif keys[pygame.K_DOWN] or keys[ord('s')]:
        rect.x = origin_x
        rect.y = origin_y + 50
        set_speed(-CONTROL_SPEED, -CONTROL_SPEED)
        color = (0, 0, 255)
    elif keys[pygame.K_LEFT] or keys[ord('a')]:
        rect.x = origin_x - 50
        rect.y = origin_y
        set_speed(-CONTROL_SPEED, CONTROL_SPEED)
        color = (0, 0, 255)
    elif keys[pygame.K_RIGHT] or keys[ord('d')]:
        rect.x = origin_x + 50
        rect.y = origin_y
        set_speed(CONTROL_SPEED, -CONTROL_SPEED)
        color = (0, 0, 255)
    else:
        rect.x = origin_x
        rect.y = origin_y
        set_speed(0, 0)
        color = (255, 0, 0)

    rect.centerx = rect.centerx % window.get_width()
    rect.centery = rect.centery % window.get_height()

    window.fill((60, 60, 60))

    pygame.draw.circle(window, (255, 255, 255), (window.get_width() // 2, window.get_height() // 2), 30)
    pygame.draw.rect(window, color, rect)
    
    font = pygame.font.SysFont('Comic Sans MS', 16)
    text = font.render('Use arrow keys or w,a,s,d to move', False, (0, 255, 0))
    window.blit(text, (10, 10))

    pygame.display.flip()

    # Get and display image
    bytes += stream.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        image = bytes[a:b+2]
        bytes = bytes[b+2:]
        try:
            image = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
        except:
            continue
        cv2.imshow("Image", image)
        cv2.waitKey(1)

pygame.quit()
exit()
