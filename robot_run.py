import robot_controller
import pygame
import time
from math import *
import client as _client
import threading

width=640
height=480

pygame.init()
window=pygame.display.set_mode((50, 50))
robot=robot_controller.robot()
client=_client.client(width, height)
cam_on=True

key_up=[pygame.K_w, pygame.K_x, pygame.K_a, pygame.K_d]


def detect_key():
    for event in pygame.event.get():
        pass

    key_press=pygame.key.get_pressed()
    if key_press[pygame.K_UP]:  # stand up
        robot.up_down(1)

    elif key_press[pygame.K_DOWN]:  # stand up
        robot.up_down(-1)

    elif key_press[pygame.K_LEFT]:  # stand up
        robot.front_back(1)

    elif key_press[pygame.K_RIGHT]:  # stand up
        robot.front_back(-1)

    elif key_press[pygame.K_RETURN]:
        if robot.status!='sleep':
            robot.rotate_to(range(8, 16), robot.origin, 70, pov=False)
            robot.status='sleep'

    elif key_press[pygame.K_SPACE]:
        robot.y_offset=-16      #-13
        robot.x_offset=-22     #-6
        robot.rad=0
        robot.gait(range(8, 16), robot.x_offset, robot.y_offset, 70)
        robot.status='normal'

    elif key_press[pygame.K_r]:
        robot.turning_body(radius=10, velocity=0.05)

    elif key_press[pygame.K_w]:
        if robot.status=='normal':
            robot.status='forward'
        if robot.status !='forward':
            robot.stand_reset(40)
            robot.status='forward'
        robot.walk(radius1=47.5, radius2=7.6, velocity=0.15)

    elif key_press[pygame.K_x]:
        if robot.status!='backward':
            robot.stand_reset(40, x=16, y=-3)
            robot.status='backward'
        robot.walk(radius1=47.5, radius2=7.6, velocity=-0.15)

    elif key_press[pygame.K_a]:
        if robot.status=='normal':
            robot.status='left'
        if robot.status!='left':
            robot.stand_reset(40)
            robot.status='left'
        robot.walk_turn(radius1=47.5, radius2=7.6, velocity=0.15, turn_rate=4, direction='left')

    elif key_press[pygame.K_d]:
        if robot.status=='normal':
            robot.status='right'
        if robot.status!='right':
            robot.stand_reset(40)
            robot.status='right'
        robot.walk_turn(radius1=47.5, radius2=7.6, velocity=0.15, turn_rate=4, direction='right')

    elif key_press[pygame.K_z]:
        robot.body_slant(0.007)

    elif key_press[pygame.K_c]:
        robot.body_slant(-0.007)

    elif key_press[pygame.K_m]:
        if robot.status!='sit':
            robot.x_offset=-22
            robot.y_offset=12
            robot.rad=0
            robot.rotate_to(range(8, 16), [52, 28, 0, 106, 132, 138, 180, 82], 70)
            robot.status='sit'

    elif key_press[pygame.K_q]:
        robot.jump(radius=30, velocity=3)

    elif key_press[pygame.K_e]:
        if robot.status in ['normal', 'reset']:
            return
        elif robot.status in ['forward', 'left', 'right']:
            robot.gait(range(8, 16), robot.x_offset, robot.y_offset, 20)
            robot.rad=0
            robot.status='normal'
        elif robot.status=='backward':
            robot.gait(range(8, 16), robot.x_offset, robot.y_offset, 20)
            robot.rad=0
            robot.status='reset'
        else:
            robot.y_offset=-16      #-13
            robot.x_offset=-22     #-6
            robot.rad=0
            robot.gait(range(8, 16), robot.x_offset, robot.y_offset, 70)
            robot.status='normal'

    elif key_press[pygame.K_s]:
        robot.check_pwm()
        print('x_offset={}, y_offset={}'.format(robot.x_offset, robot.y_offset))

    elif key_press[pygame.K_p]:
        global cam_on
        cam_on=False

    else:
        return

def capture():
    for frame in client.camera.capture_continuous(client.rawCapture, format='bgr', use_video_port=True):
        if not cam_on:
            break
        client.data_upload(frame)
    return

cam=threading.Thread(target=capture)
cam.start()
while True:
    if not cam_on:
        break
    detect_key()
cam.join()
print('all close!')