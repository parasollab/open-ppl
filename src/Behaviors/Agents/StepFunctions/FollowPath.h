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

    void ExecutePath();

    virtual bool ReachedWaypoint(const Cfg& _waypoint);

    virtual void MoveToWaypoint(const Cfg& _waypoint);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_waypointDm;

    double m_waypointThreshold{.05};

    size_t m_pathIndex{0};
    ///@}

};

#endif
