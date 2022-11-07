#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class grap_move:
    def __init__(self):

        self.move_status = True

        self.arm = Arm_Lib.Arm_Device()

        self.grap_joint = 135

    def move(self, joints, joints_down):

        joints_uu = [90, 80, 50, 50, 265, self.grap_joint]

        joints_up = [joints_down[0], 80, 50, 50, 265, 30]
 
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1500)
        sleep(1.5)

        self.arm.Arm_serial_servo_write(6, 20, 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(joints, 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)

        self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1)

        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)


    def arm_run(self, name, joints):

        if name == "red" and self.move_status == True:

            self.move_status = False
            print ("red")
            # print (joints[0], joints[1], joints[2], joints[3], joints[4])

            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]

#             joints_down = [45, 80, 35, 40, 265, self.grap_joint]

            joints_down = [45, 50, 20, 60, 265, self.grap_joint]

            self.move(joints, joints_down)

            self.move_status = True
        if name == "blue" and self.move_status == True:
            self.move_status = False
            # print ("blue")
            # print (joints[0], joints[1], joints[2], joints[3], joints[4])
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
#             joints_down = [27, 110, 0, 40, 265, self.grap_joint]
            joints_down = [27, 75, 0, 50, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        if name == "green" and self.move_status == True:
            self.move_status = False
            # print ("green")
            # print (joints[0], joints[1], joints[2], joints[3], joints[4])
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
#             joints_down = [152, 110, 0, 40, 265, self.grap_joint]
            joints_down = [152, 75, 0, 50, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        if name == "yellow" and self.move_status == True:
            self.move_status = False
            # print ("yellow")
            # print (joints[0], joints[1], joints[2], joints[3], joints[4])
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
#             joints_down = [137, 80, 35, 40, 265, self.grap_joint]
            joints_down = [137, 50, 20, 60, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
