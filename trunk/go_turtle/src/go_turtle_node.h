#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/message_filter.h>
#include <geometry_msgs/Twist.h>
#include <go_turtle/actions.h>
#include "std_msgs/String.h"
#include <kobuki_msgs/Sound.h>
#include <sstream>

#define TIME_OUT	10000
#define ONLY_GOAL	0
#define ONLY_ACTION	3
#define FREQUENCY	100
#define ROTATION_TIME	5.2

 typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 double x_param,y_param,action_param;
 double start_time;
 double secs;
 int action_completed=0;

 bool goal_state =true;
 bool dogoal = true;
 bool doaction = false;

 typedef struct 
	{
	double x, y, action, timeout;
	} Setactions;
	Setactions setaction;

 void SetGoalCoords(move_base_msgs::MoveBaseGoal& goal);
 int SetAction(ros::NodeHandle n, ros::Publisher rotate_pub,ros::Publisher feedback_pub,double secs);
 void actionCallback(const go_turtle::actions& msg);
