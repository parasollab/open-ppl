#ifndef _PPL_ROS_STEP_FUNCTION_H_
#define _PPL_ROS_STEP_FUNCTION_H_

#include "FollowPath.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

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

    /// Callback function to get boxer odometry.
    static void OdomCallback(const nav_msgs::Odometry _msg);

    /// Function to get current state of robot from ROS
    Cfg GetCurrentState();

    ///@}
    ///@name Internal State
    ///@{

    ros::Publisher m_armPub; ///< Publisher for arm controls.

    ros::Subscriber m_stateSub; ///< Subscriber for robot state.

    double m_time{1}; ///< Time duration to execute controls.

    bool m_sim{true};

    std::vector<std::string> m_jointNames;

    std::string m_robotLabel;

    ros::Publisher m_boxerPub; ///< Publisher for boxer controls.

    ros::Subscriber m_boxerOdomSub; ///< Subscriber for boxer odometry.

    ///@}
};


using JointStateCallback = std::function<void(const sensor_msgs::JointState _msg)>;

using BoxerOdomCallback = std::function<void(const nav_msgs::Odometry _msg)>;

#endif
