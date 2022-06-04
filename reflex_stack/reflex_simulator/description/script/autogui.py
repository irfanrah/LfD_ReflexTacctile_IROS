#!/usr/bin/env python3

from pkg_resources import iter_entry_points
import rospy
import rosnode
import tkinter 
from tkinter import * 
from tkinter import ttk
from std_msgs.msg import String, Float32MultiArray
from reflex_msgs.msg import Hand
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ROSImage
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError 
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ROSImage
from math import atan2, cos, sin
from constants import *
from utils import *
from preprocess import *
from demonstration_spawn import *
import os
import json
import pickle
import sklearn
from reflex_interface.msg import ArrayAction
import itertools
import inspect






def image_depth_callback(img_depth):
	global depth_image
	bridge = CvBridge()
	depth_image = bridge.imgmsg_to_cv2(img_depth)
	depth_image = np.array(depth_image, dtype=np.float32)
	depth_image = cv2.normalize(depth_image, depth_image, 0, 255, cv2.NORM_MINMAX)

def create_demonstration_folder():
	folder_path = "/home/jungeun/catkin_ws/src/datasets/demonstration"
	folder_name = str(len(os.listdir(folder_path)) + 1)
	path = os.path.join(folder_path, folder_name)
	os.mkdir(path)
	return path

def save_and_convert_img_to_state(image_depth , img_name):
	image_depth_path = path +"/depth-" + str(img_name) + ".jpg"
	cv2.imwrite(image_depth_path, image_depth)
	return image_depth_path

def save_demonstration():
	global path, state_vector
	path = create_demonstration_folder()
	remove_spawn()
	depth_image_reference_path = save_and_convert_img_to_state(depth_image, "references")
	spawn_random()
	time.sleep(0.5)
	depth_image_with_obj_path = save_and_convert_img_to_state(depth_image, "with-object")
	state_vector = generate_state_vector(depth_image_with_obj_path,depth_image_reference_path,50,invert=False,show_img=False,)
	print(f"created {path}")
	saveDemonstration['state'] = DISABLED
 
def callback_pressure(data):
	global pressureFinger 
	pressureFinger = [data.finger[0].pressure , data.finger[1].pressure, data.finger[2].pressure]
 
def hand_finger_callback(input):
	global hand_finger_state
	hand_finger_state = input.data
 

def save_action():
	state_action = {"state" :  state_vector.tolist(), 'p': hand_finger_state[:3], 'roll' :hand_finger_state[3], 'pitch' :hand_finger_state[4], 'yaw' :hand_finger_state[5], 'f' : hand_finger_state[6:9], 'pre' :hand_finger_state[9], 'spt1' :pressureFinger[0], 'spt2' :pressureFinger[1], 'spt3' :pressureFinger[2]}

	with open(path + '/state-action.txt', 'w') as f:
		f.write(json.dumps(state_action))
	print(f"saved to {path}")
	saveDemonstration['state'] = NORMAL
 
def reset_state():
	saveDemonstration['state'] = NORMAL

def predict_actions(regressors, state_vectors):
	output = np.array([regressors[i].predict(state_vectors) for i in range(len(regressors))])
	output = np.swapaxes(output, 0, 1)
	return output

def load_model():
	model_path = "/home/jungeun/catkin_ws/src/reflex_stack/reflex_simulator/description/script/model/new_demo_weights.model"
	loaded_model = pickle.load(open(model_path, 'rb'))
	return loaded_model


def set_mode():
	pub = rospy.Publisher("/gui/state", String,  queue_size=1)
	if setMode["text"] == "setToAuto":
		print("Change to Auto")
		pub.publish("auto_control")
		setMode["text"] = "setToManual"
	
	elif setMode["text"] == "setToManual":
		print("Change to Manual")
		pub.publish("manual_control")
		setMode["text"] = "setToAuto"

def action_to_ArrayAction_msg(action_to_robot, predicted_action):
	action_to_robot.px = predicted_action[0]
	action_to_robot.py = predicted_action[1]
	action_to_robot.pz = predicted_action[2]
	action_to_robot.roll = predicted_action[3]
	action_to_robot.pitch = predicted_action[4]
	action_to_robot.yaw = predicted_action[5]
	action_to_robot.f1 = predicted_action[6]
	action_to_robot.f2 = predicted_action[7]
	action_to_robot.f3 = predicted_action[8]
	action_to_robot.pre = predicted_action[9]
	action_to_robot.spt1 = predicted_action[10]
	action_to_robot.spt2 = predicted_action[11]
	action_to_robot.spt3 = predicted_action[12]

	
def send_action_vector():
	print("Sent Action Vector")
	model = load_model()
	predicted_action = predict_actions(model, [state_vector])
	pub = rospy.Publisher("/gui/predicted_action", ArrayAction,  queue_size=1)
	action_to_robot = ArrayAction()
	action_to_ArrayAction_msg(action_to_robot,predicted_action[0])
	for i in range(3):
		time.sleep(0.5)
		pub.publish(action_to_robot)
	print(action_to_robot)
	setMode["state"] = NORMAL


def auto_kill_gui(counter):
	if counter >= 1000:
		node_names_list = rosnode.get_node_names()
		isNotFound = True
		for name in node_names_list:
			if name == "/reflex_interface_node":
				isNotFound = False
				counter = itertools.count()
		if isNotFound:
			master.destroy()
			print("Relfex node not found, Quitting GUI")
	else:
		pass
	  
			
	
	

if __name__ == '__main__':
	rospy.init_node('auto_controller', anonymous=True)
	master = tkinter.Tk()
	master.title("Auto")
	master.geometry("800x600")

	START_COORDINATE = 50
	DELTA_X_COORDINATE = 100
	DELTA_Y_COORDINATE = 50
 
	image_depth_subscriber = rospy.Subscriber("/camera/depth/image_raw", ROSImage, image_depth_callback, queue_size=1)
	handFinger_listener = rospy.Subscriber("/gui/handFinger/pub_state", Float32MultiArray, hand_finger_callback, queue_size=1)
	pressure_listener = rospy.Subscriber("/reflex_takktile/hand_state", Hand, callback_pressure)
	
	
	saveDemonstration = ttk.Button(master,text='saveDemonstration', command=lambda: save_demonstration())
	saveDemonstration.place(x = START_COORDINATE ,y= START_COORDINATE)
 
	saveAction = ttk.Button(master,text='saveAction', command=lambda: save_action())
	saveAction.place(x = START_COORDINATE +  2*DELTA_X_COORDINATE,y= START_COORDINATE)
	
	resetState = ttk.Button(master,text='resetState', command=reset_state)
	resetState.place(x = START_COORDINATE ,y= START_COORDINATE + DELTA_Y_COORDINATE)
	
	sendActionVec = ttk.Button(master,text='sendActionVec', command=lambda: send_action_vector())
	sendActionVec.place(x = START_COORDINATE  +  2*DELTA_X_COORDINATE ,y= START_COORDINATE + DELTA_Y_COORDINATE)
 
	setMode = ttk.Button(master,text='setToAuto', command=lambda: set_mode(), state=DISABLED)
	setMode.place(x = START_COORDINATE  ,y= START_COORDINATE + 2*DELTA_Y_COORDINATE)
	
	counter = itertools.count()
	while not rospy.is_shutdown():
		master.update()
		auto_kill_gui(next(counter))