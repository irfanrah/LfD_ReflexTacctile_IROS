#!/usr/bin/env python3

from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetWorldProperties, GetModelState, ApplyBodyWrench
from genpy import Duration
import rospy
import time
from geometry_msgs.msg import Pose, Wrench
import numpy as np

def remove_spawn(name= "lettuce"):
    model_list = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    m_list = model_list()
    for i in m_list.model_names:
        if i == name:
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_client(model_name = name)
            
def spawn_random(useRandomLocation = True):
    obj_pose = Pose()
    obj_pose.position.x = 2.5
    obj_pose.position.y = 2
    obj_pose.position.z = 0.1
    obj_pose.orientation.x = 1.448 # fix #roll
    obj_pose.orientation.y = 1
    obj_pose.orientation.z = 0.3
    obj_pose.orientation.w = 1.1 # fix 
    
    if useRandomLocation:
            obj_pose.position.x = np.random.uniform(0, 1) - 0.5
            obj_pose.position.y = np.random.uniform(0.1, 0.8)
            obj_pose.orientation.y = np.random.uniform(0.1, 1)
            obj_pose.orientation.z = np.random.uniform(0.1, 1)
            
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
    model_name='lettuce',
    model_xml=open('/home/jungeun/catkin_ws/src/reflex_stack/reflex_simulator/description/urdf/lettuce/lettuce.urdf', 'r').read(),
    robot_namespace='/lettuce',
    initial_pose=obj_pose,
    reference_frame='ground_plane')


    time.sleep(1)
    ## Apply some force to stabilize
    my_wrech = Wrench()
    my_wrech.force.x = 0
    my_wrech.force.y = 0
    my_wrech.force.z = 3
    my_wrech.torque.x = 0
    my_wrech.torque.y = 0
    my_wrech.torque.z = 0

    model_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    model_body_wrench(body_name = "lettuce::mylettuce" , wrench = my_wrech , start_time = rospy.Time(secs = 0, nsecs = 0) ,duration = rospy.Duration(secs = 1, nsecs = 0))

def check_lettuce_pose():
    check_lettuce_properties = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    check_lettuce = check_lettuce_properties(model_name="lettuce")
    return(check_lettuce.pose)

def remove_and_spawnrand():
    remove_spawn()
    spawn_random()
    