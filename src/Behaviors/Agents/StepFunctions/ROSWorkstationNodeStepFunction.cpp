#include "ROSWorkstationNodeStepFunction.h"

#include "Behaviors/Agents/Agent.h"
#include "MPProblem/Robot/Robot.h"

double ROSWorkstationNodeStepFunction::s_timeUntilEmpty = 0.0;

ROSWorkstationNodeStepFunction::
ROSWorkstationNodeStepFunction(Agent* _agent, XMLNode& _node)
      : StepFunction(_agent, _node) {
  
  ros::start();

  ros::NodeHandle nh;

  m_timeUntilEmptySub = nh.subscribe("/estimated_time_to_empty", 1000, TimeToEmptyCallback);
 
  m_receivepartsPublisher = nh.advertise<std_msgs::Int32>("/receive_parts", 1000);

  ROSWorkstationNodeStepFunction::s_timeUntilEmpty = 0.0;
}

ROSWorkstationNodeStepFunction::
~ROSWorkstationNodeStepFunction() {}

void
ROSWorkstationNodeStepFunction::
StepAgent(double _dt) {
  ros::Rate rate(3);
  if(ros::ok())
    ros::spinOnce();
  
  if(ROSWorkstationNodeStepFunction::s_timeUntilEmpty < 20.0) {
    std_msgs::Int32 msg;
    msg.data = 5;
    m_receivepartsPublisher.publish(msg);
    rate.sleep();
  }
}

void
ROSWorkstationNodeStepFunction::
TimeToEmptyCallback(const std_msgs::Float32 _msg) {
  ROSWorkstationNodeStepFunction::s_timeUntilEmpty = double(_msg.data);
}
