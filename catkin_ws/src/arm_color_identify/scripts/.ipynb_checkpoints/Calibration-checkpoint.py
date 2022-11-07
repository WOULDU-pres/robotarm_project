# !/usr/bin/env python
# coding: utf-8
import random
import cv2 as cv
import numpy as np
import Arm_Lib

class Arm_Calibration:
    def __init__(self):

        self.image = None
        self.threshold_num = 140

        self.xy = [90, 135]
        self.rect_init()
        self.index = 0
        self.GetPoints_status = "Runing"
        self.rect_A = (self.rect_init_x, self.rect_init_y)
        self.rect_B = (self.rect_end_x, self.rect_end_y)
        self.point_initX_list=[]
        self.point_initY_list=[]
        self.point_endX_list=[]
        self.point_endY_list=[]

        self.arm = Arm_Lib.Arm_Device()

    def calibration_map(self, image, xy=None, threshold_num=140):

        if xy != None: self.xy = xy

        joints_init = [self.xy[0], self.xy[1], 0, 0, 90, 30]

        self.arm.Arm_serial_servo_write6_array(joints_init, 1500)
        self.image = image
        self.threshold_num = threshold_num

        dp = []
        h, w = self.image.shape[:2]

        contours = self.Morphological_processing()

        for i, c in enumerate(contours):

            area = cv.contourArea(c)
   
            if h * w / 2 < area < h * w:

                mm = cv.moments(c)
                if mm['m00'] == 0:
                    continue
                cx = mm['m10'] / mm['m00']
                cy = mm['m01'] / mm['m00']

                cv.drawContours(self.image, contours, i, (255, 255, 0), 2)

                dp = np.squeeze(cv.approxPolyDP(c, 100, True))

                cv.circle(self.image, (np.int(cx), np.int(cy)), 5, (0, 0, 255), -1)
        return dp, self.image

    def Morphological_processing(self):


        gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)

        gray = cv.GaussianBlur(gray, (5, 5), 1)

        ref, threshold = cv.threshold(gray, self.threshold_num, 255, cv.THRESH_BINARY)

        kernel = np.ones((3, 3), np.uint8)

        blur = cv.morphologyEx(threshold, cv.MORPH_OPEN, kernel, iterations=4)

        mode = cv.RETR_EXTERNAL

        method = cv.CHAIN_APPROX_NONE

        contours, hierarchy = cv.findContours(blur, mode, method)
        return contours

    def Perspective_transform(self, dp, image):

        if len(dp) != 4: return
        upper_left = []
        lower_left = []
        lower_right = []
        upper_right = []
        for i in range(len(dp)):
            if dp[i][0] < 320 and dp[i][1] < 240:
                upper_left = dp[i]
            if dp[i][0] < 320 and dp[i][1] > 240:
                lower_left = dp[i]
            if dp[i][0] > 320 and dp[i][1] > 240:
                lower_right = dp[i]
            if dp[i][0] > 320 and dp[i][1] < 240:
                upper_right = dp[i]

        pts1 = np.float32([upper_left, lower_left, lower_right, upper_right])

        pts2 = np.float32([[0, 0], [0, 480], [640, 480], [640, 0]])

        M = cv.getPerspectiveTransform(pts1, pts2)

        Transform_img = cv.warpPerspective(image, M, (640, 480))
        return Transform_img

    def get_points(self, img):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        value = 30
        move_value = 100
        min_point = (-1,-1)
        max_point = (-1,-1)
        points = [[0, 0], [0, 0], [0, 0], [0, 0]]
        try:

            corners = cv.goodFeaturesToTrack(gray, 4, 0.3, 10)
            if len(corners) == 4:
                corners = np.int0(corners)
                for i in range(4):
                    points[i][0] = corners[i][0][0]
                    points[i][1] = corners[i][0][1]
            min_P = min(points[0][0] + points[0][1],
                           points[1][0] + points[1][1],
                           points[2][0] + points[2][1],
                           points[3][0] + points[3][1])
            max_P = max(points[0][0] + points[0][1],
                           points[1][0] + points[1][1],
                           points[2][0] + points[2][1],
                           points[3][0] + points[3][1])
            for i in range(4):
                if (points[i][0] + points[i][1]) == min_P: min_point = points[i]
                if (points[i][0] + points[i][1]) == max_P: max_point = points[i]
            point_initX = min_point[0] + value
            point_initY = min_point[1] + value - move_value
            point_endX = max_point[0] - value
            point_endY = max_point[1] - value - move_value
            Area = (point_endX - point_initX) * (point_endY - point_initY)
            if Area <= 7000:
                point_A = (point_initX, point_initY)
                point_B = (point_endX, point_endY)
                cv.rectangle(img, point_A, point_B, (0, 255, 0), 2)
                if point_initX>0: self.point_initX_list.append(point_initX)
                if point_initY>0: self.point_initY_list.append(point_initY)
                if point_endX>0: self.point_endX_list.append(point_endX)
                if point_endY>0: self.point_endY_list.append(point_endY)
        except Exception:
            pass
        return img

    def get_hsv(self, img):

        img = cv.resize(img, (640, 480), )

        point_Xmin = 50  # 200
        point_Xmax = 600  # 440
        point_Ymin = 80  # 200
        point_Ymax = 480  # 480

        # cv.rectangle(self.image, (point_Xmin, point_Ymin), (point_Xmax, point_Ymax),(105,105,105), 2)

        img = img[point_Ymin:point_Ymax, point_Xmin:point_Xmax]
        img = cv.resize(img, (640, 480))
        if self.index <= 50:
            hsv_range = ((0, 43, 46), (255, 255, 255))
            self.index += 1
            return img, hsv_range
        if self.GetPoints_status == "Runing" and 50 < self.index < 150:
            img = self.get_points(img)
            hsv_range = ((0, 43, 46), (255, 255, 255))
            self.index += 1
            return img, hsv_range
        if self.index >= 150:

            if len(self.point_initX_list)!=0 and \
                    len(self.point_initY_list)!=0 and  \
                    len(self.point_endX_list)!=0 and  \
                    len(self.point_endY_list)!=0: self.set_rect()
#             print("self.rect_A:",self.rect_A)
#             print("self.rect_B:",self.rect_B)
            self.GetPoints_status = "waiting"
            self.index += 1
        if self.GetPoints_status == "waiting":
            img, hsv_range = self.Read_HSV(img)
            return img, hsv_range

    def set_rect(self):
        initX_list = np.argmax(np.bincount(self.point_initX_list))
        initY_list = np.argmax(np.bincount(self.point_initY_list))
        endX_list = np.argmax(np.bincount(self.point_endX_list))
        endY_list = np.argmax(np.bincount(self.point_endY_list))
        self.rect_A = (initX_list, initY_list)
        self.rect_B = (endX_list, endY_list)

    def set_index(self):
        self.index = 0
        self.GetPoints_status = "Runing"
        self.point_initX_list=[]
        self.point_initY_list=[]
        self.point_endX_list=[]
        self.point_endY_list=[]
        
    def rect_init(self):

        self.rect_init_x = 290
        self.rect_init_y = 300
        self.rect_end_x = 350
        self.rect_end_y = 360

    def Read_HSV(self, img):
        H = [];S = [];V = []
        if sum(self.rect_A)<sum(self.rect_B):
            (a, b) = self.rect_A
            (c, d) = self.rect_B
        else:
            (a, b) = self.rect_B
            (c, d) = self.rect_A
        self.rect_init_x = min(a, c)
        self.rect_end_x = max(a, c)
        self.rect_init_y = min(b, d)
        self.rect_end_y = max(b, d)
        if self.rect_init_x <= 0: self.rect_init()
        if self.rect_init_y <= 0: self.rect_init()
        if self.rect_end_x >= 640: self.rect_init()
        if self.rect_end_y >= 480: self.rect_init()

        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        cv.rectangle(img, (self.rect_init_x, self.rect_init_y), (self.rect_end_x, self.rect_end_y), (0, 255, 0), 2)

        for i in range(self.rect_init_x, self.rect_end_x):
            for j in range(self.rect_init_y, self.rect_end_y):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])

        H_min = min(H);
        H_max = max(H)
        S_min = min(S);
        S_max = max(S)
        V_min = min(V);
        V_max = max(V)

        if H_min - 2 < 0:H_min = 0
        else:H_min -= 2
        if S_min - 15 < 0:S_min = 0
        else:S_min -= 15
        if V_min - 15 < 0:V_min = 0
        else:V_min -= 15
        if H_max + 2 > 255:H_max = 255
        else:H_max += 2
        S_max = 253;V_max = 255
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        if S_min < 5 or V_min < 5:
            cv.putText(img, txt1, (self.rect_init_x - 15, self.rect_init_y - 15), cv.FONT_HERSHEY_SIMPLEX, 1,
                       (0, 0, 255), 1)
        else:
            cv.putText(img, txt2, (self.rect_init_x - 15, self.rect_init_y - 15), cv.FONT_HERSHEY_SIMPLEX, 1,
                       (0, 255, 0), 1)
        cv.putText(img, lowerb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv.putText(img, upperb, (150, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range



class update_hsv:
    def __init__(self):

        self.image = None
        self.binary = None

    def Image_Processing(self, hsv_range):

        (lowerb, upperb) = hsv_range

        color_mask = self.image.copy()

        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        color = cv.inRange(hsv_img, lowerb, upperb)

        color_mask[color == 0] = [0, 0, 0]

        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)

        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)

        # _, contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) #python2
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # python3
        return contours, binary

    def draw_contours(self, hsv_name, contours):

        for i, cnt in enumerate(contours):

            mm = cv.moments(cnt)
            if mm['m00'] == 0:
                continue
            cx = mm['m10'] / mm['m00']
            cy = mm['m01'] / mm['m00']

            area = cv.contourArea(cnt)

            if area > 800:

                (x, y) = (np.int(cx), np.int(cy))
   
                cv.circle(self.image, (x, y), 5, (0, 0, 255), -1)

                rect = cv.minAreaRect(cnt)

                box = cv.boxPoints(rect)

                box = np.int0(box)

                cv.drawContours(self.image, [box], 0, (255, 0, 0), 2)
                cv.putText(self.image, hsv_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

    def get_contours(self, img, color_name, hsv_msg, color_hsv):
        binary = None

        self.image = cv.resize(img, (640, 480), )
        for key, value in color_hsv.items():

            if color_name == key:
                color_contours, binary = self.Image_Processing(hsv_msg)
            else:
                color_contours, _ = self.Image_Processing(color_hsv[key])

            self.draw_contours(key, color_contours)
        return self.image, binary


