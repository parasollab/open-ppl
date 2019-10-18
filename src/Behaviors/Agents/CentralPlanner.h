#ifndef PMPL_CENTRAL_PLANNER_H_
#define PMPL_CENTRAL_PLANNER_H_

#include "Agent.h"
#include "DummyAgent.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"


////////////////////////////////////////////////////////////////////////////////
/// This is a template for loading paths into member path following agents to
/// enable physical experiments. Constructs a plan for multiple agents using 
/// a multi-robot planning algorithm.
////////////////////////////////////////////////////////////////////////////////
class CentralPlanner : public Agent {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef RoadmapGraph<CfgType, WeightType>         RoadmapType;
    typedef typename RoadmapType::vertex_descriptor   VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@}
    ///@name Construction
    ///@{

    /// Create an agent group with some robot as the coordinator.
    /// @param _r The coordinator robot.
    CentralPlanner(Robot* const _r);

    CentralPlanner(Robot* const _r, XMLNode& _node);

    virtual ~CentralPlanner();

    virtual std::unique_ptr<Agent> Clone(Robot* const _r) const;

    ///@}
    ///@name Agent Interface
    ///@{

    /// Call PMPL to create a path for the agent to follow.
    virtual void Initialize() override;

    /// Follow the roadmap.
    virtual void Step(const double _dt) override;

    /// Clean up.
    virtual void Uninitialize() override;
    ///@}

  private:

    ///@{

	void LoadMemberPlans();

	///@}

  protected:

    ///@name Internal State
    ///@{

    MPLibrary* m_library{nullptr};   ///< The shared-roadmap planning library.
    MPSolution* m_solution{nullptr}; ///< The shared-roadmap solution.
    std::vector<std::string> m_memberLabels;  ///< Labels for the group members.
    std::vector<DummyAgent*> m_memberAgents;       ///< All robots in the group.
	bool m_initialized{false};

    ///@}
};

#endif
