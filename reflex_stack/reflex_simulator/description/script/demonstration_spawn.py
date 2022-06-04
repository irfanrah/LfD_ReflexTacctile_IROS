#!/usr/bin/env python3

from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetWorldProperties, GetModelState, ApplyBodyWrench, SetLinkProperties, GetLinkProperties
from genpy import Duration
import rospy
import time
from geometry_msgs.msg import Pose, Wrench
import numpy as np


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def remove_spawn(name= "lettuce"):
    model_list = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    m_list = model_list()
    for i in m_list.model_names:
        if i == name:
            delete_model_client(model_name = name)
            
def spawn_box(obj_pose):
    
    obj_pose.position.z = 0.02
    obj_pose.orientation.x = 1.448 # fix #roll
    obj_pose.orientation.w = 0 # fix 
    
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model(model_name = "table", model_xml=open('/home/jungeun/catkin_ws/src/reflex_stack/reflex_simulator/description/urdf/environment/table_box.urdf' , 'r').read(),
                robot_namespace='/table',
                initial_pose=obj_pose,
                reference_frame='ground_plane' )
    
def spawn_random(useRandomLocation = True, useSpawnTable = False, useConverter = True):
    obj_pose = Pose()
    obj_pose.position.x = 2.5
    obj_pose.position.y = 2
    obj_pose.position.z = 0.1
    obj_pose.orientation.x = 1.448 # fix #roll
    obj_pose.orientation.y = 0
    obj_pose.orientation.z = 0.4
    obj_pose.orientation.w = 1 # fix 
    
    if useRandomLocation:
            obj_pose.position.x = np.random.uniform(0, 1) - 0.5
            obj_pose.position.y = np.random.uniform(0.1, 0.8)
            
    if useConverter:
        obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w = get_quaternion_from_euler(1.52 , 0 , np.random.uniform(0, 4) -2 )
        
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
    model_name='lettuce',
    model_xml=open('/home/jungeun/catkin_ws/src/reflex_stack/reflex_simulator/description/urdf/lettuce/lettuce.urdf', 'r').read(),
    robot_namespace='/lettuce',
    initial_pose=obj_pose,
    reference_frame='ground_plane')
    
    time.sleep(0.4)
    if useSpawnTable:
        for i in range(2):
            time.sleep(0.1)
            spawn_box(obj_pose)
            time.sleep(0.2)
            remove_spawn("table")
        
    ## Apply some force to stabilize
    # my_wrech = Wrench()
    # my_wrech.force.x = 0
    # my_wrech.force.y = 0
    # my_wrech.force.z = -1
    # my_wrech.torque.x = 0
    # my_wrech.torque.y = 0
    # my_wrech.torque.z = 0

    # model_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    # model_body_wrench(body_name = "lettuce::mylettuce" , wrench = my_wrech , start_time = rospy.Time(secs = 0, nsecs = 0) ,duration = rospy.Duration(secs = 1, nsecs = 0))

def check_lettuce_pose():
    check_lettuce_properties = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    check_lettuce = check_lettuce_properties(model_name="lettuce")
    return(check_lettuce.pose)

def remove_and_spawnrand():
    remove_spawn()
    remove_spawn("table")
    spawn_random()
    