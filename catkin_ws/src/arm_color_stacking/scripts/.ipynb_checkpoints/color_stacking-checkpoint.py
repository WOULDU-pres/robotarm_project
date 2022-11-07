#!/usr/bin/env python
# coding: utf-8
import rospy
import Arm_Lib
import cv2 as cv
from math import pi
from time import sleep
from stacking_move import stacking_move
from arm_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse


class color_stacking:
    def __init__(self):

        self.image = None
        self.color_name = None

        self.xy=[90,135]

        self.arm = Arm_Lib.Arm_Device()

        self.stacking_move = stacking_move()

        #self.n = rospy.init_node('ros_arm', anonymous=True)

        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)

    def stacking_grap(self, msg, xy=None):

        if xy!=None: self.xy=xy
        if len(msg)!=0:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
        for index, pos in enumerate(msg):
            try:

                joints = self.server_joint(pos)
                num = index + 1

                self.stacking_move.arm_run(str(num), joints)
            except Exception:
                print("sqaure_pos empty")

        self.arm.Arm_serial_servo_write(1, 90, 1000)
        sleep(1)

        joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]

        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
        sleep(1)

    def select_color(self, image, color_hsv, color_list):

        self.image = cv.resize(image, (640, 480))
        msg = []

        if len(color_list) >0:
            name_pos = self.color_(color_hsv, color_list[0])
            if name_pos != None: msg.append(name_pos)

        if len(color_list) > 1:
            name_pos = self.color_(color_hsv, color_list[1])
            if name_pos != None: msg.append(name_pos)

        if len(color_list) > 2:
            name_pos = self.color_(color_hsv, color_list[2])
            if name_pos != None: msg.append(name_pos)

        if len(color_list) > 3:
            name_pos = self.color_(color_hsv, color_list[3])
            if name_pos != None: msg.append(name_pos)
        return self.image, msg

    def get_Sqaure(self, hsv_lu):

        (lowerb, upperb) = hsv_lu

        mask = self.image.copy()

        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        img = cv.inRange(hsv_img, lowerb, upperb)

        mask[img == 0] = [0, 0, 0]

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)

        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)

        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  
        for i, cnt in enumerate(contours):

            x, y, w, h = cv.boundingRect(cnt)

            area = cv.contourArea(cnt)

            if area > 1000:

                cv.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                cv.circle(self.image, (int(x + w / 2), int(y + h / 2)), 5, (0, 0, 255), -1)

                cv.putText(self.image, self.color_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

                point_x = float(x + w / 2)
                point_y = float(y + h / 2)

                (a, b) = (round(((point_x - 320) / 4000), 5), round(((480 - point_y) / 3000) * 0.8+0.19, 5))
#                 (a, b) = (round(((point_x - 320) / 4000), 5), round(((240 - point_y) / 3000 + 0.265)*0.95, 5))
#                 print("------------------ stacking up -------------------")
#                 print(a, b)
#                 print("------------------ stacking down -------------------")
                return (a, b)

    def color_(self, color_hsv,name):

        sqaure_pos = None
        if name == "1":
            self.color_name = "red"
            # print "Choose red"
            sqaure_pos = self.get_Sqaure(color_hsv["red"])
        elif name == "2":
            self.color_name = "green"
            # print "Choose green"
            sqaure_pos = self.get_Sqaure(color_hsv["green"])
        elif name == "3":
            self.color_name = "blue"
            # print "Choose blue"
            sqaure_pos = self.get_Sqaure(color_hsv["blue"])
        elif name == "4":
            self.color_name = "yellow"
            # print "Choose yellow"
            sqaure_pos = self.get_Sqaure(color_hsv["yellow"])
        return sqaure_pos

    def server_joint(self, posxy):

        self.client.wait_for_service()

        request = kinemaricsRequest()
        request.tar_x = posxy[0]
        request.tar_y = posxy[1]
        request.kin_name = "ik"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):

                joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                joints[0] = response.joint1
                joints[1] = response.joint2
                joints[2] = response.joint3
                joints[3] = response.joint4
                joints[4] = response.joint5

                if joints[2] < 0:
                    joints[1] += joints[2] * 3 / 5
                    joints[3] += joints[2] * 3 / 5
                    joints[2] = 0
                # print joints
                return joints
        except Exception:
            rospy.loginfo("arg error")
