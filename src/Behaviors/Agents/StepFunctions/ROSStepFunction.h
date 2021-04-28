#ifndef _PPL_ROS_STEP_FUNCTION_H_
#define _PPL_ROS_STEP_FUNCTION_H_

#include "FollowPath.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class ROSStepFunction : public FollowPath {

  public:
    ///@name Static variables
    ///@{

    static std::vector<double> s_jointStates;

    ///@}
    ///@name Construction
    ///@{

    ROSStepFunction(Agent* _agent, XMLNode& _node);

    ~ROSStepFunction();

    ///@}
    ///@name Interface
    ///@{

    ///@}

  protected:

    //Temp function::Needs to be moved to controller class.
    void MoveArm(std::vector<double> _goal, double _dt);

    ///@name Helper Functions
    ///@{

    virtual bool ReachedWaypoint(const Cfg& _waypoint) override;

    virtual void MoveToWaypoint(const Cfg& _waypoint, double _dt) override;

    static void Callback(const sensor_msgs::JointState _msg);

    ///@}
    ///@name Internal State
    ///@{

    ros::Publisher m_armPub;

    ros::Subscriber m_stateSub;

    double m_time{1};

    ///@}
};


using JointStateCallback = std::function<void(const sensor_msgs::JointState _msg)>;

#endif
