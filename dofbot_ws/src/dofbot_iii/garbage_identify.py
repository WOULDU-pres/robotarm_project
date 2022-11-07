#!/usr/bin/env python3
# coding: utf-8
import time
import torch
import rospy
import Arm_Lib
import cv2 as cv
import numpy as np
import PID
from time import sleep
from numpy import random
from utils.torch_utils import select_device
from models.experimental import attempt_load
from garbage_grap import garbage_grap_move
from dofbot_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse
from utils.general import non_max_suppression, scale_coords, xyxy2xywh, plot_one_box

model_path = '/home/jetson/yolov5-master/runs/train/fruit_yolov5s_more_result2/weights/best.pt'
# Initialize
dev ice = select_device()
# Load model
model = attempt_load(model_path, map_location=device)
# Get names and colors
names = model.module.names if hasattr(model, 'module') else model.names
# Get the color value randomly
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]


class garbage_identify:
    def __init__(self):
        # Init image
        self.frame = None
        # Create a DOFBOT instance
        self.arm = Arm_Lib.Arm_Device()
        # DOFBOT recognition position adjustment
        self.xy = [90, 130]
        self.garbage_index=0
        # Create a garbage recognition grab instance
        self.grap_move = garbage_grap_move()
        # Create the node 
        self.n = rospy.init_node('dofbot_garbage', anonymous=True)

        self.client = rospy.ServiceProxy("dofbot_kinemarics", kinemarics)

        self.img = None
        self.target_servox=90
        self.target_servoy=45
        self.Arm = Arm_Lib.Arm_Device()
        self.xservo_pid = PID.PositionalPID(1.9, 0.3, 0.35)
        self.yservo_pid = PID.PositionalPID(1.9, 0.3, 0.35)

#     def garbage_grap(self, msg, xy=None):
#         '''
#         Execute the grap function
#         :param msg: {name:pos,...}
#         '''
#         if xy != None: self.xy = xy
#         if len(msg)!=0:
#             self.arm.Arm_Buzzer_On(1)
#             sleep(0.5)
#         for index, name in enumerate(msg):
#             try:

#                 joints = self.server_joint(msg[name])
# #                 print(joints)

#                 self.grap_move.arm_run(str(name), joints)
#             except Exception:
#                 print("sqaure_pos empty")

#         joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]

#         self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
#         sleep(1)

    # def garbage_run(self, image):
    #     '''
    #     Execute the garbage recognition function
    #     :The original image
    #     :return: identified image, identified information (name, MSG)
    #     '''
    #     # Input size of image
    #     self.frame = cv.resize(image, (640, 480))
    #     txt0 = 'Model-Loading...'
    #     msg={}
    #     if self.garbage_index<3:
    #         cv.putText(self.frame, txt0, (190, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    #         self.garbage_index+=1
    #         return self.frame,msg 
    #     if self.garbage_index>=3:

    #         try: msg = self.get_pos() 
    #         except Exception: print("get_pos NoneType")
    #         return self.frame, msg

    def get_pos(self, image):
        '''
        Get identification information
        :return: name, location
        '''
        self.frame = cv.resize(image, (640, 480))
        img = self.frame.copy()

        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img).to(device)

        img = img.float()
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3: img = img.unsqueeze(0)
        # Inference
        pred = model(img)[0]
        # Get current time
        prev_time = time.time()
        # Apply NMS
        pred = non_max_suppression(pred, 0.4, 0.5)
        gn = torch.tensor(self.frame.shape)[[1, 0, 1, 0]]
        msg = {}
        if pred != [None]:
            # Process detections
            for i, det in enumerate(pred):  # detections per image
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], self.frame.shape).round()
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    prediction_status=True
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    label = '%s %.2f' % (names[int(cls)], conf)
                    # get name
                    name = names[int(cls)]
                    name_list = ["Strawberry"]
                    for i in name_list:
                        if name == i:prediction_status=False
                    if prediction_status==True: 
                        point_x = np.int(xywh[0] * 640)
                        point_y = np.int(xywh[1] * 480)
                        cv.circle(self.frame, (point_x, point_y), 5, (0, 0, 255), -1)

                        return point_x, point_y
                        
#                         plot_one_box(xyxy, self.frame, label=label, color=colors[int(cls)], line_thickness=2)
#                         # Get current time
#                         curr_time = time.time()
#                         # Calculation time difference
#                         exec_time = curr_time - prev_time
#                         info = "time: %.2f ms" % (1000 * exec_time)
#                         # show time info
# #                         cv.putText(self.frame, text=info, org=(50, 70), fontFace=cv.FONT_HERSHEY_SIMPLEX,
# #                                    fontScale=1, color=(255, 0, 0), thickness=2)

#                         (a, b) = (round(((point_x - 320) / 4000), 5), round(((480 - point_y) / 3000) * 0.8+0.19, 5))
#                         msg[name] = (a, b)
#         return msg

    def follow_function(self):
        color_x, color_y = self.get_pos(self)
        # (color_lower, color_upper) = HSV_config
        # self.img = cv.resize(img, (640, 480), )
        # self.img = cv.GaussianBlur(self.img, (5, 5), 0)
        # hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        # mask = cv.inRange(hsv, color_lower, color_upper)
        # mask = cv.erode(mask, None, iterations=2)
        # mask = cv.dilate(mask, None, iterations=2)
        # mask = cv.GaussianBlur(mask, (5, 5), 0)
        # cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
                
                # cv.circle(self.img, (int(color_x), int(color_y)), 5, (255, 0, 255), 2)
        self.xservo_pid.SystemOutput = color_x
        self.xservo_pid.SetStepSignal(320)
        self.xservo_pid.SetInertiaTime(0.01, 0.1)
        if not (self.target_servox>=180 and color_x<=320 or self.target_servox<=0 and color_x>=320):
                    self.xservo_pid.SystemOutput = color_x
                    self.xservo_pid.SetStepSignal(320)
                    self.xservo_pid.SetInertiaTime(0.01, 0.1)
                    target_valuex = int(1500 + self.xservo_pid.SystemOutput)
                    self.target_servox = int((target_valuex - 500) / 10)

                    if self.target_servox > 180:self.target_servox = 180
                    if self.target_servox < 0: self.target_servox = 0
        if not (self.target_servoy>=180 and color_y<=240 or self.target_servoy<=0 and color_y>=240):

                    self.yservo_pid.SystemOutput = color_y
                    self.yservo_pid.SetStepSignal(240)
                    self.yservo_pid.SetInertiaTime(0.01, 0.1)
                    target_valuey = int(1500 + self.yservo_pid.SystemOutput)
                    self.target_servoy = int((target_valuey - 500) / 10) - 45

                    if self.target_servoy > 360: self.target_servoy = 360
                    if self.target_servoy < 0: self.target_servoy = 0
        joints_0 = [self.target_servox, 135, self.target_servoy / 2, self.target_servoy / 2, 90, 30]
        self.Arm.Arm_serial_servo_write6_array(joints_0, 1000)
        # return self.img

    # def server_joint(self, posxy):
    #     '''
    #     Post a position request to get the joint rotation Angle
    #     :param posxy: position point x,y coordinate
    #     :return: rotation Angle of each joint
    #     '''

    #     self.client.wait_for_service()

    #     request = kinemaricsRequest()
    #     request.tar_x = posxy[0]
    #     request.tar_y = posxy[1]
    #     request.kin_name = "ik"
    #     try:
    #         response = self.client.call(request)
    #         if isinstance(response, kinemaricsResponse):

    #             joints = [0, 0, 0, 0, 0]
    #             joints[0] = response.joint1
    #             joints[1] = response.joint2
    #             joints[2] = response.joint3
    #             joints[3] = response.joint4
    #             joints[4] = response.joint5

    #             if joints[2] < 0:
    #                 joints[1] += joints[2] / 2
    #                 joints[3] += joints[2] * 3 / 4
    #                 joints[2] = 0
    #             # print joints
    #             return joints
    #     except Exception:
    #         rospy.loginfo("arg error")
