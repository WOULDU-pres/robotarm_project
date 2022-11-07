#!/usr/bin/env python
# coding: utf-8
import cv2
from color_grab import color_grab


if __name__ == '__main__':
    
    grab = color_grab()
    capture = cv2.VideoCapture(0)
    # capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # capture.set(cv.CAP_PROP_BRIGHTNESS, 30) 
    # capture.set(cv.CAP_PROP_CONTRAST, 50)
    # capture.set(cv.CAP_PROP_EXPOSURE, 156) 
    
    while capture.isOpened():
        _, img = capture.read()
        img = grab.start_grab(img)
        cv2.imshow("img", img)
        action = cv2.waitKey(10) & 0xff
        if action == ord('q') or action == 27:
            cv2.destroyAllWindows()
            capture.release()
            break

    cv2.destroyAllWindows()
    capture.release()
