/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c)  2013, Örebro University, Sweden
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.

*Authors: Ali Abdul Khaliq on 29/11/2013
*********************************************************************/

#include "go_turtle_node.h"

//======================================================================================//
//					Main						//
//======================================================================================//
int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_navigation_goals");

	ros::NodeHandle n;

	ros::Publisher rotate_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	ros::Publisher feedback_pub = n.advertise<std_msgs::String>("action_feedback", 1000);

	ros::Publisher sound_pub = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1000);

	ros::Subscriber sub = n.subscribe("/move_action", 1000, actionCallback);

	MoveBaseClient ac("move_base", true);

	while (!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal, action;

	goal.target_pose.header.frame_id = action.target_pose.header.frame_id = "map"; 
	goal.target_pose.header.stamp = action.target_pose.header.stamp = ros::Time::now();
	
	kobuki_msgs::Sound sound;
	sound.value=6;

	std::stringstream ss;
	std_msgs::String msg;
	ss<< "NULL";

	ros::Rate loop_rate(FREQUENCY);

   while (ros::ok())
   {
	
   ros::spinOnce();
   ros::Duration d = ros::Duration(1, 0);	

  if(dogoal)
	{
	if(goal_state)
	  {   
	  SetGoalCoords(goal);
	  ROS_INFO("Sending goal");
  	  start_time=ros::Time::now().toSec();
          ac.sendGoal(goal);
	  goal_state=false;
	  }

	actionlib::SimpleClientGoalState state = ac.getState();

 	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	  {
	  if(!action_completed)
	    {
	    ROS_INFO("Sending action");
	    d.sleep();
	    action_completed = SetAction(n,rotate_pub, feedback_pub, secs);
	    d.sleep();
	    sound_pub.publish(sound);
	    }
	  }
	}

   if(doaction)
	{
	start_time=ros::Time::now().toSec();
	if(!action_completed)
	   {
	   ROS_INFO("Sending action");
	   action_completed = SetAction(n,rotate_pub,feedback_pub, secs);
	   d.sleep();
	   sound_pub.publish(sound);
	   }
	}

  if(dogoal)
	{
        actionlib::SimpleClientGoalState state = ac.getState();
	if (state == actionlib::SimpleClientGoalState::ACTIVE && !action_completed)   { ss << "ACTIVE";    }
	if (state == actionlib::SimpleClientGoalState::SUCCEEDED && action_completed) { ss << "SUCCEEDED"; }
	if (state == actionlib::SimpleClientGoalState::RECALLED ) 		      { ROS_ERROR("RECALLED"); ss << "RECALLED";  }
	if (state == actionlib::SimpleClientGoalState::REJECTED ) 		      { ROS_ERROR("REJECTED"); ss << "REJECTED";  }
	if (state == actionlib::SimpleClientGoalState::ABORTED  ) 		      { ROS_ERROR("ABORTED");  ss << "ABORTED";   }
	if (state == actionlib::SimpleClientGoalState::LOST     ) 		      { ROS_ERROR("LOST");     ss << "LOST";      }
	}
  if(doaction)
	{
	if (!action_completed) { ss << "ACTIVE";   }
	if (action_completed)  { ss << "SUCCEEDED";}
	}
	
  if((ros::Time::now().toSec()-start_time)> setaction.timeout && !action_completed)
	{
	ROS_ERROR("Time out: Failed to reach goal");
	ss << "TIME_OUT";
	ac.cancelGoal();
	}

	msg.data = ss.str();
	feedback_pub.publish(msg);
	ss.str(std::string());
	ros::spinOnce();
	loop_rate.sleep();  
   }

 return 0;
}

//======================================================================================//
//					Set goal					//
//======================================================================================//
void SetGoalCoords(move_base_msgs::MoveBaseGoal& goal)
{
	geometry_msgs::Point goalPoint;
	goalPoint.x = setaction.x;
	goalPoint.y = setaction.y;

	goal.target_pose.pose.position = goalPoint;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);//orientationPoint;
}

//======================================================================================//
//					Set action					//
//======================================================================================//
int SetAction(ros::NodeHandle n, ros::Publisher rotate_pub,ros::Publisher feedback_pub,double secs)
{
	ros::Rate loop_rate(FREQUENCY);
	geometry_msgs::Twist pose;
	std_msgs::String msg;
	std::stringstream ss;
	ss << "ACTIVE";
	msg.data = ss.str();
	secs =ros::Time::now().toSec();

  do {
	pose.linear.x = 0.0;
	pose.linear.y = 0.0;
	pose.linear.z = 0.0;
	pose.angular.x = 0.0;
	pose.angular.y = 0.0;

	if(setaction.action ==0.0)
	   pose.angular.z = 0.0;
	else
	   pose.angular.z = 1.5 *(setaction.action/abs(setaction.action)) ;

	msg.data = ss.str();
	feedback_pub.publish(msg);
	rotate_pub.publish(pose);
	ros::spinOnce();
	loop_rate.sleep();
     }
	while(ros::ok() && (ros::Time::now().toSec()-secs)< (ROTATION_TIME * abs(setaction.action)));
	
	ss.str(std::string());

  return (1);
}

//======================================================================================//
//					Call back					//
//======================================================================================//
void actionCallback(const go_turtle::actions& msg)
{
	if(setaction.x!= msg.x || setaction.y!= msg.y || setaction.action!= msg.action || setaction.timeout!= msg.timeout)
	{
	action_completed=0;
	goal_state = true;
	dogoal = true;
	doaction = false;
	}

	if(setaction.action>3 || setaction.action<-2)
	{
	ROS_ERROR("wrong action");
	return;
	}

	setaction.x =  msg.x;
	setaction.y =  msg.y;
	setaction.action =  msg.action;
	setaction.timeout =  msg.timeout;

	if(setaction.action == ONLY_GOAL)
		doaction=false;
	if(setaction.action == ONLY_ACTION)
		{
		dogoal=false;
		doaction = true;
		}
}

//======================================================================================//
//					EOF						//
//======================================================================================//
