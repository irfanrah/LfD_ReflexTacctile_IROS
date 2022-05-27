#!/usr/bin/env python3
from email.mime import image
from enum import Flag
from turtle import color

from numpy import average
import rospy
import rosnode
import tkinter 
from tkinter import * 
from tkinter import ttk
from tkinter.ttk import Label
from std_msgs.msg import String, Float32MultiArray
from reflex_msgs.msg import Hand
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ROSImage
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError 
from demonstration_spawn import *
from reflex_interface.msg import ArrayAction



def callback_hand(inp_string):
	oneTimePublish = None
	pub = rospy.Publisher('/gui/hand/control', String, queue_size=1)
	rate_sleep = rospy.Rate(10) # 10hz
	rospy.loginfo(inp_string)
	pub.publish(inp_string)
	rate_sleep.sleep()
	pub.publish(oneTimePublish)

def callback_finger(inp_string):
	oneTimePublish = None
	updateFinger = "updateFinger"
	pub = rospy.Publisher('/gui/finger/control', String, queue_size=2)
	rate_sleep = rospy.Rate(10) # 10hz
	rospy.loginfo(inp_string)
	pub.publish(inp_string)
	rate_sleep.sleep()
	rospy.sleep(0.05)
	pub.publish(updateFinger)
	rospy.sleep(0.05)
	pub.publish(oneTimePublish)
 
def callback_pressure(data):
	global PRESSURE_FINGER 
	PRESSURE_FINGER = [data.finger[0].pressure , data.finger[1].pressure, data.finger[2].pressure]
 
def image_callback(img):
	global rgb_img
	bridge = CvBridge()
	rgb_img = bridge.imgmsg_to_cv2(img)
 
def image_depth_callback(img_depth):
	global depth_image
	bridge = CvBridge()
	depth_image = bridge.imgmsg_to_cv2(img_depth)
	depth_image = np.array(depth_image, dtype=np.float32)
	depth_image = cv2.normalize(depth_image, depth_image, 0, 255, cv2.NORM_MINMAX)

def hand_finger_callback(input):
	global hand_finger_state
	hand_finger_state = input.data
	

def finger_hand_updated_text(f1,f2,f3,hand_state, finger_state, approx= 4):
	if f1 != None and f2 != None and f3!= None and hand_state!= None and finger_state!= None: 
		f1.destroy()
		f2.destroy()
		f3.destroy()
		hand_state.destroy()
		finger_state.destroy()
	f1 = tkinter.Label(master=master, text=PRESSURE_FINGER[0], fg='black',bd = '3' ,font=("Helvetica", 10))
	f1.place(x = START_COORDINATE+DELTA_X_COORDINATE, y = START_COORDINATE+ 5/3*DELTA_Y_COORDINATE)
	f2 = tkinter.Label(master=master, text=PRESSURE_FINGER[1], fg='black',bd = '3' ,font=("Helvetica", 10))
	f2.place(x = START_COORDINATE+DELTA_X_COORDINATE, y = START_COORDINATE+ 6/3*DELTA_Y_COORDINATE)
	f3 = tkinter.Label(master=master, text=PRESSURE_FINGER[2], fg='black',bd = '3' ,font=("Helvetica", 10))
	f3.place(x = START_COORDINATE+DELTA_X_COORDINATE, y = START_COORDINATE+ 7/3*DELTA_Y_COORDINATE)
	hand_state = tkinter.Label(master=master, text=[round(i, approx) for i in hand_finger_state[:6]], fg='black',bd = '3' ,font=("Helvetica", 10))
	hand_state.place(x = START_COORDINATE+DELTA_X_COORDINATE, y = START_COORDINATE+ 8/3*DELTA_Y_COORDINATE)
	finger_state = tkinter.Label(master=master, text=[round(i, approx) for i in hand_finger_state[6:]], fg='black',bd = '3' ,font=("Helvetica", 10))
	finger_state.place(x = START_COORDINATE+DELTA_X_COORDINATE, y = START_COORDINATE+ 9/3*DELTA_Y_COORDINATE)	
	return f1,f2,f3, hand_state, finger_state

def update_rgb_img(my_rgb_img):
	b,g,r = cv2.split(my_rgb_img)
	img = cv2.merge((r,g,b)) 
	im = Image.fromarray(img)
	imgtk = ImageTk.PhotoImage(image=im)
	return imgtk
	

def update_depth_img(my_depth_img):
	im_d = Image.fromarray(my_depth_img)
	imgtk_d = ImageTk.PhotoImage(image=im_d)
	return imgtk_d

def auto_kill_gui(counter):
	if counter >= 1000:
		node_names_list = rosnode.get_node_names()
		isNotFound = True
		for name in node_names_list:
			if name == "/reflex_interface_node":
				isNotFound = False
		if isNotFound:
			master.destroy()
			print("Relfex node not found, Quitting GUI")
	else:
		pass

def action_to_goto_msg(goto_state_robot, lettuce_loc):
	goto_state_robot.px = lettuce_loc.position.x + 0.02
	goto_state_robot.py = lettuce_loc.position.y - 0.015
	goto_state_robot.pz = 0.078
	goto_state_robot.roll = 3.14
	goto_state_robot.pitch = 0
	goto_state_robot.yaw = lettuce_loc.orientation.z + 0.785
	goto_state_robot.f1 = 0
	goto_state_robot.f2 = 0
	goto_state_robot.f3 = 0
	goto_state_robot.pre = 0
	goto_state_robot.spt1 = lettuce_loc.orientation.w
	goto_state_robot.spt2 = 0
	goto_state_robot.spt3 = 0

def goto_lettuce():
	lettuce_location = check_lettuce_pose()
	pub_state = rospy.Publisher("/gui/state", String,  queue_size=1)
	pub_goto_loc = rospy.Publisher("/gui/goto_state", ArrayAction,  queue_size=1)
	pub_state.publish("goto_control")
	action_to_robot = ArrayAction()
	action_to_goto_msg(action_to_robot, lettuce_location)
	print(action_to_robot)
	for i in range(3):
		time.sleep(0.5)
		pub_goto_loc.publish(action_to_robot)
	pub_state.publish("manual_control")
	


if __name__ == '__main__':
	rospy.init_node('gui_controller', anonymous=True)
	master = tkinter.Tk()
	master.title("Config")
	master.geometry("1000x800")
 
	START_COORDINATE = 50
	DELTA_X_COORDINATE = 100
	DELTA_Y_COORDINATE = 50
	PRESSURE_FINGER = 0
 
	counter = 0
	f1 = None
	f2 = None
	f3 = None
	hand_state = None
	finger_state = None

	rgb_img = np.zeros([240,320,3], dtype=np.uint8)
	depth_image = np.ones([240,320], dtype=np.uint8)

	Zplus = ttk.Button(master,text='Zplus', command=lambda: callback_hand('Zplus'))
	Zplus.place(x = START_COORDINATE ,y= START_COORDINATE)
	Zmin = ttk.Button(master,text='Zmin',command=lambda: callback_hand('Zmin'))
	Zmin.place(x = START_COORDINATE ,y= START_COORDINATE + DELTA_Y_COORDINATE)

	Yplus = ttk.Button(master,text='Yplus',command=lambda: callback_hand('Yplus'))
	Yplus.place(x = START_COORDINATE + DELTA_X_COORDINATE  ,y= START_COORDINATE)
	Ymin = ttk.Button(master,text='Ymin',command=lambda: callback_hand('Ymin'))
	Ymin.place(x = START_COORDINATE + DELTA_X_COORDINATE ,y= START_COORDINATE + DELTA_Y_COORDINATE)

	Xplus = ttk.Button(master,text='Xplus',command=lambda: callback_hand('Xplus'))
	Xplus.place(x = START_COORDINATE + 2*DELTA_X_COORDINATE  ,y= START_COORDINATE)
	Xmin = ttk.Button(master,text='Xmin',command=lambda: callback_hand('Xmin'))
	Xmin.place(x = START_COORDINATE + 2*DELTA_X_COORDINATE ,y= START_COORDINATE + DELTA_Y_COORDINATE)
 
 
	rollPlus = ttk.Button(master,text='rollPlus', command=lambda: callback_hand('rollPlus'))
	rollPlus.place(x = START_COORDINATE + 4*DELTA_X_COORDINATE ,y= START_COORDINATE)
	rollMin = ttk.Button(master,text='rollMin',command=lambda: callback_hand('rollMin'))
	rollMin.place(x = START_COORDINATE + 4*DELTA_X_COORDINATE ,y= START_COORDINATE + DELTA_Y_COORDINATE)
 
	pitchPlus = ttk.Button(master,text='pitchPlus',command=lambda: callback_hand('pitchPlus'))
	pitchPlus.place(x = START_COORDINATE + 5*DELTA_X_COORDINATE  ,y= START_COORDINATE)
	pitchMin = ttk.Button(master,text='pitchMin',command=lambda: callback_hand('pitchMin'))
	pitchMin.place(x = START_COORDINATE + 5*DELTA_X_COORDINATE ,y= START_COORDINATE + DELTA_Y_COORDINATE)

	yawPlus = ttk.Button(master,text='yawPlus',command=lambda: callback_hand('yawPlus'))
	yawPlus.place(x = START_COORDINATE + 6*DELTA_X_COORDINATE  ,y= START_COORDINATE)
	yawMin = ttk.Button(master,text='yawMin',command=lambda: callback_hand('yawMin'))
	yawMin.place(x = START_COORDINATE + 6*DELTA_X_COORDINATE ,y= START_COORDINATE + DELTA_Y_COORDINATE)
	
	#Service Buttons
	resetPose = ttk.Button(master,text='resetPose', command=lambda: callback_hand('resetPose'))
	resetPose.place(x = START_COORDINATE + 7*DELTA_X_COORDINATE ,y= START_COORDINATE)
 
	openFinger = ttk.Button(master,text='openFinger', command=lambda: callback_hand('openFinger'))
	openFinger.place(x = START_COORDINATE + 8*DELTA_X_COORDINATE ,y= START_COORDINATE + 2*DELTA_Y_COORDINATE)
 
	closeFinger = ttk.Button(master,text='closeFinger', command=lambda: callback_hand('closeFinger'))
	closeFinger.place(x = START_COORDINATE + 8*DELTA_X_COORDINATE ,y= START_COORDINATE + 3*DELTA_Y_COORDINATE)
 
	thighenGrasp = ttk.Button(master,text='thighenGrasp', command=lambda: callback_hand('thighenGrasp'))
	thighenGrasp.place(x = START_COORDINATE + 8*DELTA_X_COORDINATE ,y= START_COORDINATE + 4*DELTA_Y_COORDINATE)
 
	closeUntilContact = ttk.Button(master,text='closeUntilContact', command=lambda: callback_hand('closeUntilContact'))
	closeUntilContact.place(x = START_COORDINATE + 8*DELTA_X_COORDINATE ,y= START_COORDINATE + 5*DELTA_Y_COORDINATE)
	
	
	#Finger Buttons
	f1Plus = ttk.Button(master,text='f1Plus', command=lambda: callback_finger('f1Plus'))
	f1Plus.place(x = START_COORDINATE + 4*DELTA_X_COORDINATE ,y= START_COORDINATE + 2* DELTA_Y_COORDINATE)
	f1Min = ttk.Button(master,text='f1Min',command=lambda: callback_finger('f1Min'))
	f1Min.place(x = START_COORDINATE + 4*DELTA_X_COORDINATE ,y= START_COORDINATE + 3 *DELTA_Y_COORDINATE)
 
	f2Plus = ttk.Button(master,text='f2Plus',command=lambda: callback_finger('f2Plus'))
	f2Plus.place(x = START_COORDINATE + 5*DELTA_X_COORDINATE  ,y= START_COORDINATE + 2* DELTA_Y_COORDINATE)
	f2Min = ttk.Button(master,text='f2Min',command=lambda: callback_finger('f2Min'))
	f2Min.place(x = START_COORDINATE + 5*DELTA_X_COORDINATE ,y= START_COORDINATE + 3 *DELTA_Y_COORDINATE)

	f3Plus = ttk.Button(master,text='f3Plus',command=lambda: callback_finger('f3Plus'))
	f3Plus.place(x = START_COORDINATE + 6*DELTA_X_COORDINATE  ,y= START_COORDINATE + 2* DELTA_Y_COORDINATE)
	f3Min = ttk.Button(master,text='f3Min',command=lambda: callback_finger('f3Min'))
	f3Min.place(x = START_COORDINATE + 6*DELTA_X_COORDINATE ,y= START_COORDINATE + 3*DELTA_Y_COORDINATE)
 
	prePlus = ttk.Button(master,text='prePlus',command=lambda: callback_finger('prePlus'))
	prePlus.place(x = START_COORDINATE + 7*DELTA_X_COORDINATE  ,y= START_COORDINATE + 2* DELTA_Y_COORDINATE)
	preMin = ttk.Button(master,text='preMin',command=lambda: callback_finger('preMin'))
	preMin.place(x = START_COORDINATE + 7*DELTA_X_COORDINATE ,y= START_COORDINATE + 3*DELTA_Y_COORDINATE)
 
	
	#Plugin Button
	respawnObject = ttk.Button(master,text='respawnObject', command= lambda: remove_and_spawnrand())
	respawnObject.place(x = START_COORDINATE + 4*DELTA_X_COORDINATE ,y= START_COORDINATE + 4* DELTA_Y_COORDINATE)
 
	gotoLettuce = ttk.Button(master,text='gotoLettuce', command= lambda: goto_lettuce())
	gotoLettuce.place(x = START_COORDINATE + 6*DELTA_X_COORDINATE ,y= START_COORDINATE + 4* DELTA_Y_COORDINATE)

	rospy.Subscriber("/reflex_takktile/hand_state", Hand, callback_pressure)
	
	tkinter.Label(master=master, text="f1 pressure : ", fg='black',bd = '3' ,font=("Helvetica", 10)).place(x = START_COORDINATE, y = START_COORDINATE+ 5/3*DELTA_Y_COORDINATE)
	tkinter.Label(master=master, text="f2 pressure : ", fg='black',bd = '3' ,font=("Helvetica", 10)).place(x = START_COORDINATE, y = START_COORDINATE+ 6/3*DELTA_Y_COORDINATE)
	tkinter.Label(master=master, text="f3 pressure : ", fg='black',bd = '3' ,font=("Helvetica", 10)).place(x = START_COORDINATE, y = START_COORDINATE+ 7/3*DELTA_Y_COORDINATE)
 
	## Image
	image_subscriber = rospy.Subscriber("/camera/rgb/image_raw", ROSImage, image_callback)
	image_rgb_label = tkinter.Label(master=master, image=None)
	image_rgb_label.place(x = START_COORDINATE  ,y= START_COORDINATE + 9*DELTA_Y_COORDINATE)
 
	## Image Depth
	image_depth_subscriber = rospy.Subscriber("/camera/depth/image_raw", ROSImage, image_depth_callback, queue_size=1)
	image_depth_label = tkinter.Label(master=master, image=None)
	image_depth_label.place(x = START_COORDINATE + 3*DELTA_X_COORDINATE  ,y= START_COORDINATE + 9*DELTA_Y_COORDINATE)
	
	#Finger and Hand State
	handFinger_listener = rospy.Subscriber("/gui/handFinger/pub_state", Float32MultiArray, hand_finger_callback, queue_size=1)
	tkinter.Label(master=master, text="Hand pose :", fg='black',bd = '3' ,font=("Helvetica", 10)).place(x = START_COORDINATE, y = START_COORDINATE+ 8/3*DELTA_Y_COORDINATE)
	tkinter.Label(master=master, text="Finger pose:", fg='black',bd = '3' ,font=("Helvetica", 10)).place(x = START_COORDINATE, y = START_COORDINATE+ 9/3*DELTA_Y_COORDINATE)
 
	while not rospy.is_shutdown():
		counter += 1
		if counter >= 1000:
			f1,f2,f3, hand_state, finger_state = finger_hand_updated_text(f1,f2,f3,hand_state,finger_state)
			auto_kill_gui(counter)
			counter = 0
		if rgb_img is not None:
			rgb_img_out = update_rgb_img(rgb_img)
			image_rgb_label.config(image=rgb_img_out)
		if depth_image is not None:
			depth_image_out = update_depth_img(depth_image)
			image_depth_label.config(image=depth_image_out)
   
		master.update()
