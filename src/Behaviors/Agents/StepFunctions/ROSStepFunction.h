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

    /// Call back function to pass into ros behaviors.
    static void Callback(const sensor_msgs::JointState _msg);

    ///@}
    ///@name Internal State
    ///@{

    ros::Publisher m_armPub; ///< Publisher for arm controls.

    ros::Subscriber m_stateSub; ///< Subscriber for robot state.

    double m_time{1}; ///< Time duration to execute controls.

    bool m_sim{true};

    ///@}
};


using JointStateCallback = std::function<void(const sensor_msgs::JointState _msg)>;

#endif
