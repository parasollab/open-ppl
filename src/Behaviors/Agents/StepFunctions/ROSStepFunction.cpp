#include "ROSStepFunction.h"

#include "Behaviors/Agents/PathFollowingAgent.h"

#include "MPProblem/Robot/Robot.h"

#include <trajectory_msgs/JointTrajectory.h>

/*----------------------- Construction --------------------*/
ROSStepFunction::
ROSStepFunction(Agent* _agent, XMLNode& _node) 
              : StepFunction(_agent,_node) {

  int argc = 0;
  char* argv[255];

  ros::init(argc,argv,this->m_agent->GetRobot()->GetLabel());

  ros::start();
  
  ROS_INFO_STREAM("Hello World!");

  ros::NodeHandle nh;
  m_armPub = nh.advertise<trajectory_msgs::JointTrajectory>(
                           "simplemanip1/arm_controller/command",
                            10);
  /*
  ros::spin();

  ros::shutdown();
  */
}

ROSStepFunction::
~ROSStepFunction() { }

/*------------------------- Interface -----------------------*/

void 
ROSStepFunction::
StepAgent(double _dt) {
  std::cout << "Calling step function." << std::endl;


  auto p = dynamic_cast<PathFollowingAgent*>(this->m_agent);
  if(!p)
    return;

  //TODO::Roughly follow path following code that moves from waypoint to waypoint


  //TODO::Create ROS Controller that can be called here

/*
  std::vector<double> point = {0,0,0};
  MoveArm(arm_pub,point);

  point = {.785,.785,.785};
  MoveArm(arm_pub,point);

  point = {1.57,1.57,1.57};
  MoveArm(arm_pub,point);

  point = {.785,.785,.785};
  MoveArm(arm_pub,point);

  point = {0,0,0};
  MoveArm(arm_pub,point);

  point = {-.785,-.785,-.785};
  MoveArm(arm_pub,point);

  point = {-1.57,-1.57,-1.57};
  MoveArm(arm_pub,point);

  ros::spinOnce();

  ros::Duration(3).sleep();
*/
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

    msg.points[0].positions = _goal;

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
