#ifndef _PPL_ROS_STEP_FUNCTION_H_
#define _PPL_ROS_STEP_FUNCTION_H_

#include "FollowPath.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

// #include <move_base_msgs/MoveBaseAction.h>
// #include <move_base_msgs/MoveBaseResult.h>
// #include <actionlib/client/simple_action_client.h>
// #include "actionlib/client/simple_client_goal_state.h"
// #include "actionlib/client/terminal_state.h"

class ROSStepFunction : public FollowPath {

  public:

    // typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

    void MoveBase(const Cfg& _goal, double _dt);

    ///@name Helper Functions
    ///@{

    virtual bool ReachedWaypoint(const Cfg& _waypoint) override;
    virtual bool ReachedWaypointArm(const Cfg& _waypoint);
    virtual bool ReachedWaypointBase(const Cfg& _waypoint);

    virtual void MoveToWaypoint(const Cfg& _waypoint, double _dt) override;

    /// Call back function to pass into ros behaviors.
    static void Callback(const sensor_msgs::JointState _msg);

    static void OdomCallback(const nav_msgs::Odometry _msg);

    Cfg GetCurrentState();

    // void SimpleDoneCallback(const actionlib::SimpleClientGoalState& _state, const move_base_msgs::MoveBaseResult::ConstPtr& _result);

    // void SimpleFeedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& _feedback, const Cfg& _waypoint);

    ///@}
    ///@name Internal State
    ///@{

    ros::Publisher m_armPub; ///< Publisher for arm controls.

    ros::Subscriber m_stateSub; ///< Subscriber for robot state.

    double m_time{1}; ///< Time duration to execute controls.

    bool m_sim{true};

    std::string m_robotLabel;

    bool m_reachedWaypoint{false};

    // MoveBaseClient m_ac;

    ros::Publisher m_boxerPub;

    ros::Subscriber m_boxerOdomSub;

    ///@}
};


using JointStateCallback = std::function<void(const sensor_msgs::JointState _msg)>;

using BoxerOdomCallback = std::function<void(const nav_msgs::Odometry _msg)>;

#endif
