#!/usr/bin/env python
# coding: utf-8

# # Catch game

# ### Import head file

# In[1]:


#!/usr/bin/env python
# coding: utf-8
import cv2 as cv
import threading
from time import sleep
from dofbot_config import *
import ipywidgets as widgets
from IPython.display import display
from snake_target import snake_target
from snake_ctrl import snake_ctrl


# ### Initialize DOFBOT position

# In[2]:


import Arm_Lib
Arm = Arm_Lib.Arm_Device()
joints_0 = [90, 135, 0,45, 0, 180]
Arm.Arm_serial_servo_write6_array(joints_0, 1000)


# ### Create the instance and initialize the parameters

# In[3]:


snake_target = snake_target()
snake_ctrl = snake_ctrl()

model = 'General'

color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
color_hsv  = {"red"   : ((0, 43, 46), (10, 255, 255)),
              "green" : ((35, 43, 46), (77, 255, 255)),
              "blue"  : ((100, 43, 46), (124, 255, 255)),
              "yellow": ((26, 43, 46), (34, 255, 255))}
HSV_path="/home/jetson/dofbot_ws/src/dofbot_snake_follow/scripts/HSV_config.txt"
try: read_HSV(HSV_path,color_hsv)
except Exception: print("Read HSV_config Error!!!")


# ### Create controls

# In[4]:


button_layout      = widgets.Layout(width='150px', height='27px', align_self='center')
output = widgets.Output()
choose_color=widgets.ToggleButtons( options=['red', 'green', 'blue','yellow'], button_style='success', 
    tooltips=['Description of slow', 'Description of regular', 'Description of fast'])
# Exit
exit_button = widgets.Button(description='Exit', button_style='danger', layout=button_layout)
imgbox = widgets.Image(format='jpg', height=480, width=640, layout=widgets.Layout(align_self='center'))
down_box = widgets.HBox([choose_color,exit_button], layout=widgets.Layout(align_self='center'));
controls_box = widgets.VBox([imgbox, down_box], layout=widgets.Layout(align_self='center'))
# ['auto', 'flex-start', 'flex-end', 'center', 'baseline', 'stretch', 'inherit', 'initial', 'unset']


# ### Control

# In[5]:


def exit_button_Callback(value):
    global model
    model = 'Exit'
#     with output: print(model)
exit_button.on_click(exit_button_Callback)


# ### Main process

# In[6]:


def camera():
    # Open camera
    capture = cv.VideoCapture(0)
    # The loop is executed when the camera is opened normally 
    while capture.isOpened():
        try:
            _, img = capture.read()

            img = cv.resize(img, (640, 480))

            img, snake_msg = snake_target.target_run(img, color_hsv)
            if len(snake_msg) == 1:
                threading.Thread(target=snake_ctrl.snake_main, args=(choose_color.value, snake_msg,)).start()
            if model == 'Exit':
                cv.destroyAllWindows()
                capture.release()
                break

            cv.putText(img, choose_color.value, (int(img.shape[0] / 2), 50), cv.FONT_HERSHEY_SIMPLEX, 2, color[random.randint(0, 254)], 2)
            imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
        except KeyboardInterrupt:capture.release()


# ### Start

# In[7]:


display(controls_box,output)
threading.Thread(target=camera, ).start()


# In[ ]:




