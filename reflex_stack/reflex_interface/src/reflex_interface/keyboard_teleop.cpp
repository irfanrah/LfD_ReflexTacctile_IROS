#include <iostream>
#include <map>
#include <termios.h>
#include <math.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>

#include "reflex_interface/GraspPrimitive.h"
#include "reflex_interface/PosIncrement.h"
#include "reflex_interface/ArrayAction.h"
#include <unistd.h>
#include <limits>

#include <array>

std::string node_name = "finger_teleop_node";
std::string source_frame = "world";
std::string target_frame = "reflex";

std::string open_srv_name = "reflex_interface/open";
std::string close_srv_name = "reflex_interface/close";
std::string pinch_srv_name = "reflex_interface/pinch";
std::string sph_open_srv_name = "reflex_interface/spherical_open";
std::string sph_close_srv_name = "reflex_interface/spherical_close";
std::string pos_incr_srv_name = "reflex_interface/position_increment";
std::string close_until_contact_srv_name = "reflex_interface/close_until_contact";
std::string tighten_grip_srv_name = "reflex_interface/tighten_grip";

float trans_scaling = 0.01;
float rot_scaling = 0.08;
float finger_scaling = 0.1;
float oldArrayAction = 0;
float AUTO_STEP_DELAY = 0.8;


int everyIteration = 0;
int publishEveryIteration = 1000;

const int lenHandPose = 6;

// format: {x, y, z, r, p ,y} in "reflex" frame
std::array<float, 6> init_pose = {0, 0, 0.2, M_PI, 0, 0};
std_msgs::Float32MultiArray currentHandFingerState; // this std_msgs have data class, so we do .data in while loop
std_msgs::Float32MultiArray cur_finger_pose_parse;

std_msgs::String f1_parse;
std_msgs::String f2_parse;
std_msgs::String f3_parse;
std_msgs::String preshape_parse;

std::string MODE = "manual_control";
std::string MANUAL_STATE = "manual_control";
std::string AUTO_STATE = "auto_control";
std::string GOTO_STATE = "goto_control";

std_msgs::Float32MultiArray array_action;
std_msgs::Float32MultiArray goto_control;

tf2::Transform getTcpToWristFrame()
{
	float z_offset = -0.09228;
	float x_offset = -0.02;

	tf2::Transform translate_to_wrist = tf2::Transform();
	translate_to_wrist.setIdentity();
	translate_to_wrist.setOrigin(tf2::Vector3{x_offset, 0, z_offset});

	return translate_to_wrist;
}

// keys for wrist teleoperation (note this is for my german keyboard)
std::map<char, std::vector<float>> wrist_bindings{

	// format: {x, y, z, r, p ,y}
	{'u', {1, 0, 0, 0, 0, 0}},
	{'i', {0, 1, 0, 0, 0, 0}},
	{'o', {0, 0, 1, 0, 0, 0}},
	{'j', {0, 0, 0, 1, 0, 0}},
	{'k', {0, 0, 0, 0, 1, 0}},
	{'l', {0, 0, 0, 0, 0, 1}},
	{'U', {-1, 0, 0, 0, 0, 0}},
	{'I', {0, -1, 0, 0, 0, 0}},
	{'O', {0, 0, -1, 0, 0, 0}},
	{'J', {0, 0, 0, -1, 0, 0}},
	{'K', {0, 0, 0, 0, -1, 0}},
	{'L', {0, 0, 0, 0, 0, -1}},
};



// keys for reflex finger teleoperation (note this is for my german keyboard)
std::map<char, std::vector<float>> finger_bindings{

	// format: {f1, f2, f3, preshape}
	{'q', {1, 0, 0, 0}},
	{'w', {0, 1, 0, 0}},
	{'e', {0, 0, 1, 0}},
	{'r', {0, 0, 0, 1}},
	{'Q', {-1, 0, 0, 0}},
	{'W', {0, -1, 0, 0}},
	{'E', {0, 0, -1, 0}},
	{'R', {0, 0, 0, -1}},
};

// BEGIN CODE FROM https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp

// for non-blocking keyboard inputs
int getch(void)
{
	int ch;
	struct termios oldt;
	struct termios newt;

	// store old settings, and copy to new settings
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;

	// make required changes and apply the settings
	newt.c_lflag &= ~(ICANON | ECHO);
	newt.c_iflag |= IGNBRK;
	newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
	newt.c_cc[VMIN] = 1;
	newt.c_cc[VTIME] = 0;
	tcsetattr(fileno(stdin), TCSANOW, &newt);

	// get the current character
	ch = getchar();

	// reapply old settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}

// END CODE FROM https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp

tf2::Transform calcTransformFromEuler(std::array<float, 6> pose)
{
	tf2::Vector3 t = {pose[0], pose[1], pose[2]};
	tf2::Quaternion q;
	q.setRPY(pose[3], pose[4], pose[5]);
	tf2::Transform transform(q, t);

	return transform;
}

class Listener {
	public:
		std::string handListener;
		std::string fingerListener;

		void guiHandCallback(const std_msgs::String::ConstPtr& msg);
		void guiFingerCallback(const std_msgs::String::ConstPtr& msg);
		void stateCallback(const std_msgs::String::ConstPtr& msg);
		void arrayAction(const reflex_interface::ArrayAction::ConstPtr& msg);
		void GotoControl(const reflex_interface::ArrayAction::ConstPtr& msg);
};

void Listener::guiHandCallback(const std_msgs::String::ConstPtr& msg){
	handListener = msg->data.c_str();}

void Listener::guiFingerCallback(const std_msgs::String::ConstPtr& msg){
	fingerListener = msg->data.c_str();}

void Listener::stateCallback(const std_msgs::String::ConstPtr& msg){
	MODE = msg->data.c_str();}

void Listener::arrayAction(const reflex_interface::ArrayAction::ConstPtr& msg){
	array_action.data.clear();
	array_action.data.push_back(msg->px);
	array_action.data.push_back(msg->py);
	array_action.data.push_back(msg->pz);
	array_action.data.push_back(msg->roll);
	array_action.data.push_back(msg->pitch);
	array_action.data.push_back(msg->yaw);
	array_action.data.push_back(msg->f1);
	array_action.data.push_back(msg->f2);
	array_action.data.push_back(msg->f3);
	array_action.data.push_back(msg->pre);
	array_action.data.push_back(msg->spt1);
	array_action.data.push_back(msg->spt2);
	array_action.data.push_back(msg->spt3);}

void Listener::GotoControl(const reflex_interface::ArrayAction::ConstPtr& msg){
	goto_control.data.clear();
	goto_control.data.push_back(msg->px);
	goto_control.data.push_back(msg->py);
	goto_control.data.push_back(msg->pz);
	goto_control.data.push_back(msg->roll);
	goto_control.data.push_back(msg->pitch);
	goto_control.data.push_back(msg->yaw);
	goto_control.data.push_back(msg->f1);
	goto_control.data.push_back(msg->f2);
	goto_control.data.push_back(msg->f3);
	goto_control.data.push_back(msg->pre);
	goto_control.data.push_back(msg->spt1);
	goto_control.data.push_back(msg->spt2);
	goto_control.data.push_back(msg->spt3);}


std_msgs::Float32MultiArray parsePosFinger(reflex_interface::PosIncrement& pos_incr_void , std_msgs::String& f1_parse,std_msgs::String& f2_parse,std_msgs::String& f3_parse,std_msgs::String& preshape_parse){
	const int lenHandPose = 6;
	const int startStringFinger = 6;
	const int lengthNumFinger = 8; 
	const int fingerTextSpace = 2;
	std_msgs::Float32MultiArray cur_finger_pose_parse;

	f1_parse.data.clear();
	f2_parse.data.clear();
	f3_parse.data.clear();
	preshape_parse.data.clear();
	cur_finger_pose_parse.data.clear();

	for (int i=0; i < lengthNumFinger; i++){
		f1_parse.data.push_back(pos_incr_void.response.message[i+startStringFinger]);}

	for (int i=0; i < lengthNumFinger; i++){
		f2_parse.data.push_back(pos_incr_void.response.message[i+startStringFinger+lengthNumFinger + fingerTextSpace]);}

	for (int i=0; i < lengthNumFinger; i++){
		f3_parse.data.push_back(pos_incr_void.response.message[i+startStringFinger+ 2* (lengthNumFinger + fingerTextSpace)]);}

	for (int i=0; i < lengthNumFinger; i++){
		preshape_parse.data.push_back(pos_incr_void.response.message[i+startStringFinger+ 3* (lengthNumFinger + fingerTextSpace)]);}

	
	float f1_val = std::stof(f1_parse.data);
	float f2_val = std::stof(f2_parse.data);
	float f3_val = std::stof(f3_parse.data);
	float preshape_val = std::stof(preshape_parse.data);

	cur_finger_pose_parse.data.push_back(f1_val);
	cur_finger_pose_parse.data.push_back(f2_val);
	cur_finger_pose_parse.data.push_back(f3_val);
	cur_finger_pose_parse.data.push_back(preshape_val);


	return cur_finger_pose_parse;
}

void exact_input_finger(reflex_interface::PosIncrement& pos_incr_void , ros::ServiceClient pos_incr_client_void ,std_msgs::Float32MultiArray current_pos_list, 
float desire_f1 , float desire_f2, float desire_f3 , float desire_preshape ){
	const int steepMovement = 3; // use =1 if not using steep movement

	float deltaf1 = desire_f1 - current_pos_list.data[0];
	float deltaf2 = desire_f2 - current_pos_list.data[1];
	float deltaf3 = desire_f3 - current_pos_list.data[2];
	float deltapreshape = desire_preshape - current_pos_list.data[3];

	ROS_INFO_STREAM(deltaf1);

	for(int i = 0; i < steepMovement; i++ ){
		pos_incr_void.request.f1 = deltaf1 / steepMovement;
		pos_incr_void.request.f2 = deltaf2 / steepMovement;
		pos_incr_void.request.f3 = deltaf3 / steepMovement;
		pos_incr_void.request.preshape = deltapreshape / steepMovement;
		pos_incr_client_void.call(pos_incr_void);
		usleep(0.5 *1000000);}
	ROS_INFO_STREAM(pos_incr_void.response.message);}


void update_pose(std::array<float, 6> cur_pose, tf2::Transform transform ,geometry_msgs::TransformStamped ts,tf2_ros::TransformBroadcaster br ){
	transform = calcTransformFromEuler(cur_pose) * getTcpToWristFrame();
	ts.header.stamp = ros::Time::now();
	ts.transform = tf2::toMsg(transform);
	br.sendTransform(ts);
}


template <class FloatType> int safeFloatToInt(const FloatType &num) {
   //check if float fits into integer
   if ( std::numeric_limits<int>::digits < std::numeric_limits<FloatType>::digits) {
      // check if float is smaller than max int
      if( (num < static_cast<FloatType>( std::numeric_limits<int>::max())) &&
          (num > static_cast<FloatType>( std::numeric_limits<int>::min())) ) {
         return static_cast<int>(num); //safe to cast
      } else {
        std::cerr << "Unsafe conversion of value:" << num << std::endl;
        //NaN is not defined for int return the largest int value
        return std::numeric_limits<int>::max();
      }
   } else {
      //It is safe to cast
      return static_cast<int>(num);
   }
}


int main(int argc, char **argv)
{   
	Listener listener;
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ROS_INFO("Launched %s node.", node_name.c_str());

	// service clients for hand_command_node
	ros::ServiceClient open_client = nh.serviceClient<reflex_interface::GraspPrimitive>(open_srv_name);
	ros::ServiceClient close_client = nh.serviceClient<reflex_interface::GraspPrimitive>(close_srv_name);
	ros::ServiceClient pinch_client = nh.serviceClient<reflex_interface::GraspPrimitive>(pinch_srv_name);
	ros::ServiceClient sph_open_client = nh.serviceClient<reflex_interface::GraspPrimitive>(sph_open_srv_name);
	ros::ServiceClient sph_close_client = nh.serviceClient<reflex_interface::GraspPrimitive>(sph_close_srv_name);
	ros::ServiceClient pos_incr_client = nh.serviceClient<reflex_interface::PosIncrement>(pos_incr_srv_name);
	ros::ServiceClient close_until_contact_client = nh.serviceClient<std_srvs::Trigger>(close_until_contact_srv_name);
	ros::ServiceClient tighten_grip_client = nh.serviceClient<std_srvs::Trigger>(tighten_grip_srv_name);

	// service messages
	std_srvs::Trigger trigger;
	reflex_interface::PosIncrement pos_incr;
	reflex_interface::GraspPrimitive gp;

	// init transform broadcaster
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped ts;
	tf2::Transform transform;

	// populate initial wrist transform (we are controlling TCP)
	std::array<float, 6> cur_pose = init_pose;
	transform = calcTransformFromEuler(init_pose) * getTcpToWristFrame();
	ts.header.frame_id = source_frame;
	ts.child_frame_id = target_frame;
	ts.transform = tf2::toMsg(transform);

	// TODO find another, more elegant solution for this
	// wait before publishing first transform to fix warning from wrist_controller_node
	// ""reflex" passed to lookupTransform argument target_frame does not exist."
	// this also waits for services of reflex interface to spawn
	ros::Duration(3).sleep();

	// send initial wrist transform
	ts.header.stamp = ros::Time::now();
	br.sendTransform(ts);
	open_client.call(gp);

	ROS_INFO("Listening to GUI input...");
	char key(' ');

	ros::Subscriber hand_gui_control_sub = nh.subscribe("/gui/hand/control", 1,  &Listener::guiHandCallback, &listener);
	ros::Subscriber finger_gui_control_sub = nh.subscribe("/gui/finger/control", 1,  &Listener::guiFingerCallback, &listener);
	ros::Subscriber state_sub = nh.subscribe("/gui/state", 1,  &Listener::stateCallback, &listener);
	ros::Subscriber action_state_sub = nh.subscribe("/gui/predicted_action", 1,  &Listener::arrayAction, &listener);
	ros::Subscriber goto_state_sub = nh.subscribe("/gui/goto_state", 1,  &Listener::GotoControl, &listener);

	ros::Publisher hand_gui_state_pub = nh.advertise<std_msgs::Float32MultiArray>("/gui/handFinger/pub_state" , 1000);

	std::string wrist_movement[6] = { "Zplus", "Zmin", "Yplus", "Ymin" , "Xplus", "Xmin"};
	std::string wrist_axis[6] = { "rollPlus", "rollMin", "pitchPlus", "pitchMin" , "yawPlus", "yawMin"};
	std::string option_service[5] = {"resetPose", "openFinger" , "closeFinger" , "thighenGrasp" , "closeUntilContact"};
	std::string finger_control[9] = {"f1Plus" , "f1Min" , "f2Plus" , "f2Min" , "f3Plus" , "f3Min" , "prePlus" , "preMin", "updateFinger"};
	
	
	//give value for finger update
	usleep(500000);
	open_client.call(gp);
	
	close_client.call(gp);
	usleep(500000);
	open_client.call(gp);
	pos_incr_client.call(pos_incr);


	while (ros::ok()) 
	{
		ros::spinOnce();

		std::array<float, 6> oldCurPose = cur_pose;
		
		
		if (MODE == MANUAL_STATE){
			// Z axis
			if (listener.handListener == wrist_movement[0]){
				cur_pose[2] += trans_scaling * wrist_bindings['o'][2];
			}else if (listener.handListener == wrist_movement[1]){
				cur_pose[2] += trans_scaling * wrist_bindings['O'][2];
			// Y axis
			}else if (listener.handListener == wrist_movement[2]){
				cur_pose[1] += trans_scaling * wrist_bindings['i'][1];
			}else if (listener.handListener == wrist_movement[3]){
				cur_pose[1] += trans_scaling * wrist_bindings['I'][1];
			// X axis
			}else if (listener.handListener == wrist_movement[4]){
				cur_pose[0] += trans_scaling * wrist_bindings['u'][0];
			}else if (listener.handListener == wrist_movement[5]){
				cur_pose[0] += trans_scaling * wrist_bindings['U'][0];
			}
			
			//Orientation Axis
			if (listener.handListener == wrist_axis[0]){
				cur_pose[3] += rot_scaling * wrist_bindings['j'][3];
			}else if (listener.handListener == wrist_axis[1]){
				cur_pose[3] += rot_scaling * wrist_bindings['J'][3];

			}else if (listener.handListener == wrist_axis[2]){
				cur_pose[4] += rot_scaling * wrist_bindings['k'][4];
			}else if (listener.handListener == wrist_axis[3]){
				cur_pose[4] += rot_scaling * wrist_bindings['K'][4];

			}else if (listener.handListener == wrist_axis[4]){
				cur_pose[5] += rot_scaling * wrist_bindings['l'][5];
			}else if (listener.handListener == wrist_axis[5]){
				cur_pose[5] += rot_scaling * wrist_bindings['L'][5];
			}

			// Position reset to initial
			if (listener.handListener == option_service[0]){
				cur_pose = init_pose;
				ROS_INFO("Wrist transform reset.");

				// cur_finger_pose_parse = parsePosFinger(pos_incr, f1_parse, f2_parse, f3_parse, preshape_parse);
				// exact_input_finger(pos_incr, pos_incr_client , cur_finger_pose_parse , 1, 1, 1, 1);

			}


			// Update the pose
			if (oldCurPose != cur_pose){
				update_pose(cur_pose, transform,ts ,br);
			}

			// Service
			if (listener.handListener == option_service[1]){
				open_client.call(gp);
				pos_incr_client.call(pos_incr);
				ROS_INFO_STREAM(pos_incr.response.message);}
			if (listener.handListener == option_service[2]){
				close_client.call(gp);
				pos_incr_client.call(pos_incr);
				ROS_INFO_STREAM(pos_incr.response.message);}
			if (listener.handListener == option_service[3]){
				tighten_grip_client.call(trigger);
				pos_incr_client.call(pos_incr);
				ROS_INFO_STREAM(pos_incr.response.message);}
			if (listener.handListener == option_service[4]){
				close_until_contact_client.call(trigger);
				usleep(1000);
				pos_incr_client.call(pos_incr);
				ROS_INFO_STREAM(pos_incr.response.message);}



			// Finger control
			if (listener.fingerListener == finger_control[0]){
				pos_incr.request.f1 = finger_scaling * finger_bindings['q'][0];
				pos_incr.request.f2 = 0;
				pos_incr.request.f3 = 0;
				pos_incr.request.preshape = 0;
			}else if (listener.fingerListener == finger_control[1]){
				pos_incr.request.f1 = finger_scaling * finger_bindings['Q'][0];
				pos_incr.request.f2 = 0;
				pos_incr.request.f3 = 0;
				pos_incr.request.preshape = 0;}
			if (listener.fingerListener == finger_control[2]){
				pos_incr.request.f1 = 0;
				pos_incr.request.f2 = finger_scaling * finger_bindings['w'][1];
				pos_incr.request.f3 = 0;
				pos_incr.request.preshape = 0;
			}else if (listener.fingerListener == finger_control[3]){
				pos_incr.request.f1 = 0;
				pos_incr.request.f2 = finger_scaling * finger_bindings['W'][1];
				pos_incr.request.f3 = 0;
				pos_incr.request.preshape = 0;}
			if (listener.fingerListener == finger_control[4]){
				pos_incr.request.f1 = 0;
				pos_incr.request.f2 = 0;
				pos_incr.request.f3 = finger_scaling * finger_bindings['e'][2];
				pos_incr.request.preshape = 0;
			}else if (listener.fingerListener == finger_control[5]){
				pos_incr.request.f1 = 0;
				pos_incr.request.f2 = 0;
				pos_incr.request.f3 = finger_scaling * finger_bindings['E'][2];
				pos_incr.request.preshape = 0;}
			if (listener.fingerListener == finger_control[6]){
				pos_incr.request.f1 = 0;
				pos_incr.request.f2 = 0;
				pos_incr.request.f3 = 0;
				pos_incr.request.preshape = finger_scaling * finger_bindings['r'][3];
			}else if (listener.fingerListener == finger_control[7]){
				pos_incr.request.f1 = 0;
				pos_incr.request.f2 = 0;
				pos_incr.request.f3 = 0;
				pos_incr.request.preshape = finger_scaling * finger_bindings['R'][3];}
			

			// Finger Service Update
			if (listener.fingerListener == finger_control[8]){
				pos_incr_client.call(pos_incr);
				ROS_INFO_STREAM(pos_incr.response.message);}
			
			//Pub state to GUI
			if (everyIteration <= publishEveryIteration){
					currentHandFingerState.data.clear();

					for (int i=0; i < lenHandPose; i++){ // Hand pose append
						currentHandFingerState.data.push_back(cur_pose[i]);}
					
					cur_finger_pose_parse = parsePosFinger(pos_incr, f1_parse, f2_parse, f3_parse, preshape_parse);

					for (int i=0; i < 4; i++){ // Finger pose append
						currentHandFingerState.data.push_back(cur_finger_pose_parse.data[i]);}

					hand_gui_state_pub.publish(currentHandFingerState);
					everyIteration = 0;
				}
			}
			else if(MODE == AUTO_STATE){
				if(oldArrayAction != array_action.data[0]){ // when grab data from array_action.data[0] for logic, error (teleop node died). Must use different approach -> solved at first the .data is null so it's make error
					ROS_INFO_STREAM("Starting Auto");
					usleep(AUTO_STEP_DELAY *1000000);
					cur_pose = {array_action.data[0], array_action.data[1], 0.2, M_PI, 0, 0};
					update_pose(cur_pose, transform,ts ,br);
					usleep( AUTO_STEP_DELAY *1000000);
					cur_pose = {array_action.data[0], array_action.data[1], 0.2, array_action.data[3], array_action.data[4], array_action.data[5]};
					update_pose(cur_pose, transform,ts ,br);
					usleep( AUTO_STEP_DELAY *1000000);
					cur_pose = {array_action.data[0], array_action.data[1], array_action.data[2], array_action.data[3], array_action.data[4], array_action.data[5]};
					update_pose(cur_pose, transform,ts ,br);
					usleep(AUTO_STEP_DELAY *1000000);
					cur_finger_pose_parse = parsePosFinger(pos_incr, f1_parse, f2_parse, f3_parse, preshape_parse);
					exact_input_finger(pos_incr, pos_incr_client , cur_finger_pose_parse , array_action.data[6], array_action.data[7], array_action.data[8], array_action.data[9]);
					usleep(AUTO_STEP_DELAY*2 *1000000);
					cur_pose = {array_action.data[0], array_action.data[1], 0.2, M_PI, array_action.data[4], array_action.data[5]};
					update_pose(cur_pose, transform,ts ,br);
					usleep(AUTO_STEP_DELAY*2 *1000000);
					cur_pose = {0, 0, 0.2, M_PI, 0, 0};
					update_pose(cur_pose, transform,ts ,br);
					usleep(AUTO_STEP_DELAY*5 *1000000);
					cur_finger_pose_parse = parsePosFinger(pos_incr, f1_parse, f2_parse, f3_parse, preshape_parse);
					exact_input_finger(pos_incr, pos_incr_client , cur_finger_pose_parse , 0.1, 0.1, 0.1, 0);
					usleep(AUTO_STEP_DELAY*1 *1000000);
					

					oldArrayAction = array_action.data[0];
				}
				
				ROS_INFO_STREAM("Do Nothing");
			}
			else if(MODE == GOTO_STATE){
				ROS_INFO_STREAM("Go To State Mode");
				usleep(1 *1000000);
				cur_pose = {goto_control.data[0], goto_control.data[1], goto_control.data[2], goto_control.data[3], goto_control.data[4], goto_control.data[5]};
				update_pose(cur_pose, transform,ts ,br);
			}
			
		
			
		everyIteration = everyIteration + 1;

		loop_rate.sleep();
	}


	
	return 0;
}