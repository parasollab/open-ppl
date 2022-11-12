#ifndef PPL_ROS_WORKSTATION_NODE_STEP_FUNCTION_H_
#define PPL_ROS_WORKSTATION_NODE_STEP_FUNCTION_H_

#include "StepFunction.h"

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

class ROSWorkstationNodeStepFunction : public StepFunction {
  
  public:
    static double s_timeUntilEmpty;

    ROSWorkstationNodeStepFunction(Agent* _agent, XMLNode& _node);

    ~ROSWorkstationNodeStepFunction();

    virtual void StepAgent(double _dt) override;

    static void TimeToEmptyCallback(const std_msgs::Float32 _msg);

  protected:

    ros::Subscriber m_timeUntilEmptySub;

    ros::Publisher m_receivepartsPublisher;

};

#endif