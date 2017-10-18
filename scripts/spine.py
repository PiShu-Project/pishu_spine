#!/usr/bin/env python
import sys
from threading import Lock

from time import sleep, clock

import numpy

import rospy
from std_msgs.msg import String
from pishu_msgs.msg import MultiFaces, FacePosition

import Adafruit_PCA9685

face_pos = numpy.array([0.0, 0.0])
face_pos_lock = Lock()

def big_face_callback(data):
    global face_pos  
    face_pos_lock.acquire()
    face_pos = numpy.array([data.x, data.y])
    face_pos_lock.release()

def main():
    global face_pos
    rospy.init_node('spine', anonymous=False)

    rospy.Subscriber("eye/biggest_face", FacePosition, big_face_callback)

    cam_pos = numpy.array([620.0, 570.0])

    pwm = Adafruit_PCA9685.PCA9685()

    pwm.set_pwm_freq(80)

    while not rospy.is_shutdown():
        face_pos_lock.acquire()
        cam_pos = cam_pos - 0.1*face_pos
        face_pos = 0.9*face_pos
        face_pos_lock.release()
        sleep(0.05)  

        #print cam_pos

        cam_pos[0] = max(min(cam_pos[0], 780), 460)
        cam_pos[1] = max(min(cam_pos[1], 650), 430)

        pwm.set_pwm(0, 0, int(cam_pos[0]))
        pwm.set_pwm(1, 0, int(cam_pos[1]))      

## Quitting

    # stop servomotors
    pwm.set_pwm(0, 0, 0)
    pwm.set_pwm(1, 0, 0)

if __name__ == '__main__':
    main()  

