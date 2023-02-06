import gi

gi.require_version('Gtk','2.0')

import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs

# Author: Michael Aboulhair 

pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(config)

frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
color_init = np.asanyarray(color_frame.get_data())

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
lineType               = 2

frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
depth_frame = frameset.get_depth_frame()

color = np.asanyarray(color_frame.get_data())
res = color.copy()
try:
  while(True):
        
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

    l_b = np.array([24, 133, 48])
    u_b = np.array([39, 200, 181])

    mask = cv2.inRange(hsv, l_b, u_b)
    color = cv2.bitwise_and(color, color, mask=mask)

    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)
    
    # Update color and depth frames:
    aligned_depth_frame = frameset.get_depth_frame()
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

    ### motion detector
    d = cv2.absdiff(color_init, color)
    gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
    (c, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(color, c, -1, (0, 255, 0), 2)
    color_init = color

    depth = np.asanyarray(aligned_depth_frame.get_data())



    cv2.namedWindow('RBG', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RBG', color)
    cv2.waitKey(1)
finally:
  pipe.stop