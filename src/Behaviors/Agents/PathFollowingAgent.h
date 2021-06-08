#ifndef PATH_FOLLOWING_AGENT_H_
#define PATH_FOLLOWING_AGENT_H_

#include "PlanningAgent.h"

#include "ConfigurationSpace/Cfg.h"

////////////////////////////////////////////////////////////////////////////////
/// This agent calls pmpl to generate a path, then attempts to follow the path
/// waypoints as closely as possible by relying on the robot's controller.
////////////////////////////////////////////////////////////////////////////////
class PathFollowingAgent : public PlanningAgent {

  public:

    ///@name Construction
    ///@{

    PathFollowingAgent(Robot* const _r);

    PathFollowingAgent(Robot* const _r, const PathFollowingAgent& _a);

    PathFollowingAgent(Robot* const _r, XMLNode& _node);

    virtual std::unique_ptr<Agent> Clone(Robot* const _r) const override;

    virtual ~PathFollowingAgent();

    ///@}
    ///@name Agent Interface
    ///@{

    virtual void Uninitialize() override;

    void ClearVisualGraph();

    ///@}
    ///@name Planning
    ///@{

    virtual bool HasPlan() const override;

    virtual void ClearPlan() override;

    void SetPlan(std::vector<Cfg> _path);
    ///@}

  protected:

    ///@name Planning Helpers
    ///@{

    virtual void WorkFunction(std::shared_ptr<MPProblem> _problem) override;

    ///@}
    ///@name Task Helpers
    ///@{

    virtual bool EvaluateTask() override;

    virtual void ExecuteTask(const double _dt) override;

    ///@}
    ///@name Internal State
    ///@{

    std::vector<Cfg> m_path; ///< The path to follow.
    size_t m_pathIndex{0};   ///< The path node that is the current subgoal.

    /// The distance metric for checking whether the agent has reached a
    /// path waypoint.
    std::string m_waypointDm;

    /// The distance threshold for waypoint proximity.
    double m_waypointThreshold{.05};

    size_t m_pathVisualID{size_t(-1)}; ///< The ID of the path drawing.

    size_t m_graphVisualID{size_t(-1)}; ///< The ID of the graph drawing.
    ///@}

};

#endif
