import sys
import cv2
import numpy as np
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-l", "--left_output", required=True,            help="path to left output video file")
ap.add_argument("-r", "--right_output", required=True,            help="path to right output video file")
ap.add_argument("-f", "--fps",    type=int, default=30,     help="FPS of output video")
ap.add_argument("-c", "--codec",  type=str, default="MJPG", help="codec of output video")
args = vars(ap.parse_args())

fourcc = cv2.VideoWriter_fourcc(*args['codec'])
print("FOURCC = {:x}".format(fourcc))

a = (fourcc >> 24) & 0xff
b = (fourcc >> 16) & 0xff
c = (fourcc >> 8) & 0xff
d = fourcc & 0xff
print("{:x} {}".format(a, chr(a)))
print("{:x} {}".format(b, chr(b)))
print("{:x} {}".format(c, chr(c)))
print("{:x} {}".format(d, chr(d)))

camL = cv2.VideoCapture(1)
camR = cv2.VideoCapture(2)
ch = 0
imgL = np.array(1)
imgR = np.array(1)

fps = args['fps']
left_file  = cv2.VideoWriter(args['left_output'],  fourcc, fps, (640, 480), True)
right_file = cv2.VideoWriter(args['right_output'], fourcc, fps, (640, 480), True)

while ch != 27:
    camL.grab()
    camR.grab()

    retL, imgL = camL.retrieve()
    retR, imgR = camR.retrieve()

    if retL and retR:
        left_file.write(imgL)
        right_file.write(imgR)

        cv2.imshow('LEFT', imgL)

        ch = cv2.waitKey(1) & 0xFF
    else:
        break

left_file.release()
right_file.release()
camL.release()
camR.release()
cv2.destroyAllWindows()

print("DONE")
