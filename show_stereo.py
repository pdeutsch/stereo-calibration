import sys
import cv2
import numpy as np
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-l", "--left", required=True,            help="path to left output video file")
ap.add_argument("-r", "--right", required=True,            help="path to right output video file")
ap.add_argument("-f", "--fps",    type=int, default=30,     help="FPS of output video")
args = vars(ap.parse_args())

fps = 1.0 * args['fps']
wait_time = int((1000.0 / fps) - 1)

vid_l = cv2.VideoCapture(args['left'])
vid_r = cv2.VideoCapture(args['right'])

ch = 0
imgL = np.array(1)
imgR = np.array(1)
flag = True

while ch != 27:
    vid_l.grab()
    vid_r.grab()

    retL, imgL = vid_l.retrieve()
    retR, imgR = vid_r.retrieve()

    if retL and retR:
        cv2.imshow('LEFT', imgL)
        cv2.imshow('RIGHT', imgR)
        if flag:
            cv2.moveWindow('LEFT', 10, 10)
            cv2.moveWindow('RIGHT', 650, 10)
            flag = False

        ch = cv2.waitKey(wait_time) & 0xFF
    else:
        break

vid_l.release()
vid_r.release()
cv2.destroyAllWindows()

print("DONE")
