#ifndef ROADMAP_FOLLOWING_AGENT_H_
#define ROADMAP_FOLLOWING_AGENT_H_

#include "Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"


////////////////////////////////////////////////////////////////////////////////
/// This agent calls pmpl once and then follows the resulting path.
////////////////////////////////////////////////////////////////////////////////
class RoadmapFollowingAgent : public Agent {
  public:

    ///@name Motion Planning Types
    ///@{

    //Stolen from Roadmap.h:
    typedef RoadmapGraph<CfgType, WeightType>     GraphType;
    typedef typename GraphType::vertex_descriptor VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@}

    ///@name Construction
    ///@{

    RoadmapFollowingAgent(Robot* const _r);

    virtual ~RoadmapFollowingAgent();

    ///@}
    ///@name Agent Interface
    ///@{

    /// Call PMPL to create a path for the agent to follow.
    /// @TODO can also have an istream version that reads in roadmap from file.
    virtual void Initialize() override;

    /// Follow the roadmap.
    virtual void Step(const double _dt) override;

    /// Clean up.
    virtual void Uninitialize() override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    //I believe the MPSolution needs to be kept as well, since when deleting
    // it, the Path we save will be thrown away too. This could probably be
    // solved using a shared_ptr or something, but for now I'll do this.
    MPSolution* m_solution;

    //Path type seems necessary, as I need that list of VIDs to get edges
    Path* m_roadmap{nullptr}; ///< The roadmap to follow

    /// These are both similar measures, one is the iterator that is set to the
    /// current vertex that we are on. The path index is just a counter so to
    /// easily know the index of the vertex we are on.
    VIDIterator m_currentVID{0};   ///< The path iterator that is set to current Cfg.
    std::size_t m_currentVertexPathIndex{0};

    /// This is the number of Step() calls that the same control
    /// (in m_delayControl) should be repeated.
    std::size_t m_delayStepCount{0};

    Control* m_delayControl{nullptr};

    MPLibrary* m_library{nullptr}; ///< This agent's planning library.

    ///@}

};

#endif
