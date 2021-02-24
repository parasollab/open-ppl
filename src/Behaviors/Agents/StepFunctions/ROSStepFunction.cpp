#include "ROSStepFunction.h"

#include "Behaviors/Agents/PathFollowingAgent.h"

#include "MPProblem/Robot/Robot.h"

#include <trajectory_msgs/JointTrajectory.h>

std::vector<double> ROSStepFunction::s_jointStates = {};

/*-------------------------- Construction -----------------------*/
ROSStepFunction::
ROSStepFunction(Agent* _agent, XMLNode& _node) 
              : FollowPath(_agent,_node) {

  int argc = 0;
  char* argv[255];

  ros::init(argc,argv,this->m_agent->GetRobot()->GetLabel());

  ros::start();
  
  ROS_INFO_STREAM("Hello World!");

  ros::NodeHandle nh;
  m_armPub = nh.advertise<trajectory_msgs::JointTrajectory>(
                           "simplemanip1/arm_controller/command",
                            10);

  /*JointStateCallback callback = [this](const sensor_msgs::JointState _msg) {
    this->Callback(_msg);
  };*/

  m_stateSub = nh.subscribe("simplemanip1/joint_states",1000,Callback);
}

ROSStepFunction::
~ROSStepFunction() { }

/*---------------------------- Interface ---------------------------*/

/*------------------------- Helper Functions -----------------------*/

bool 
ROSStepFunction::
ReachedWaypoint(const Cfg& _waypoint) {
  ros::spinOnce();

  auto js = s_jointStates;
  std::vector<double> jointStates;
  for(auto d : js) {
    jointStates.push_back(d/PI);
  }
  
  //TODO::Check if m_jointState has reached the current waypoint

  return true;
}

void 
ROSStepFunction::
MoveToWaypoint(const Cfg& _waypoint) {
  MoveArm(_waypoint.GetData());
}
    
void 
ROSStepFunction::
MoveArm(std::vector<double> _goal) {
  if(ros::ok()) {

    ros::Rate rate(3);

    trajectory_msgs::JointTrajectory msg;

    msg.joint_names.clear();
    msg.joint_names.push_back("sphere_0_to_link_1");
    msg.joint_names.push_back("sphere_1_to_link_2");
    msg.joint_names.push_back("sphere_2_to_link_3");

    msg.points.resize(1);

    //msg.points[0].positions = _goal;
    for(auto d : _goal) {
      msg.points[0].positions.push_back(d*PI);
    }

    msg.points[0].time_from_start = ros::Duration(3.0);

    ROS_INFO_STREAM("Sending command:\n" << msg);
    m_armPub.publish(msg);
    rate.sleep();
    m_armPub.publish(msg);
    rate.sleep();
    m_armPub.publish(msg);
    rate.sleep();
  }
}
   
void 
ROSStepFunction::
Callback(const sensor_msgs::JointState _msg) {
  ROS_INFO_STREAM("Received: " << _msg.position);
  s_jointStates = _msg.position;
}
