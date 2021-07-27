#ifndef PPL_FOLLOW_PATH_H_
#define PPL_FOLLOW_PATH_H_

#include "StepFunction.h"

class Cfg;

class FollowPath : public StepFunction {

  public: 
    ///@name Construction
    ///@{

    FollowPath(Agent* _agent, XMLNode& _node);

    virtual ~FollowPath();

    ///@}
    ///@name Interface
    ///@{

    virtual void StepAgent(double _dt) override;

    ///@}

  protected: 

    ///@name Helper Functions
    ///@{

    /// Follow the path of waypoints.
    /// @param _dt the discrete timestep.
    void ExecutePath(double _dt);

    /// Check if the robot has reached the waypoint.
    /// @param _waypoint The waypoint to check the status of.
    virtual bool ReachedWaypoint(const Cfg& _waypoint);

    /// Propogate controls for moving the robot to the waypoint.
    /// @param _waypoint The wayopint to move the robot towards.
    /// @param _dt The discrete timesteps
    virtual void MoveToWaypoint(const Cfg& _waypoint, double _dt);

    ///@}
    ///@name Internal State
    ///@{

    ///< Distance metric to eval distance to waypoint.
    std::string m_waypointDm; 

    ///< Threshold to declare you've reached a waypoint.
    double m_waypointThreshold{.05};

    size_t m_pathIndex{0}; ///< Index of current waypoint in the path.

    ///@}

};

#endif
