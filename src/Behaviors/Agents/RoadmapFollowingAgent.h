#ifndef ROADMAP_FOLLOWING_AGENT_H_
#define ROADMAP_FOLLOWING_AGENT_H_

#include "PlanningAgent.h"

#include "MPLibrary/PMPL.h"


////////////////////////////////////////////////////////////////////////////////
/// This agent calls pmpl once and then follows the resulting path.
////////////////////////////////////////////////////////////////////////////////
class RoadmapFollowingAgent : public PlanningAgent {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef RoadmapGraph<CfgType, WeightType>         GraphType;
    typedef typename GraphType::vertex_descriptor     VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@}
    ///@name Construction
    ///@{

    RoadmapFollowingAgent(Robot* const _r);

    RoadmapFollowingAgent(Robot* const _r, const RoadmapFollowingAgent& _a);

    RoadmapFollowingAgent(Robot* const _r, XMLNode& _node);

    virtual std::unique_ptr<Agent> Clone(Robot* const _r) const override;

    virtual ~RoadmapFollowingAgent();

    ///@}
    ///@name Agent Interface
    ///@{

    virtual void Uninitialize() override;

    ///@}
    ///@name Planning
    ///@{
    
    virtual bool HasPlan() const override;

    virtual void ClearPlan() override;

    ///@}

  protected:

    ///@name Planning Helpers
    ///@{

    virtual void GeneratePlan() override;

    ///@}
    ///@name Task Helpers
    ///@{

    virtual bool EvaluateTask() override;

    virtual void ExecuteTask(const double _dt) override;

    ///@}
    ///@name Internal State
    ///@{

    VIDIterator m_currentSubgoal;    ///< The current subgoal in the path.
    const WeightType* m_edge{nullptr}; ///< The current edge being traversed.

    ///@}

};

#endif
