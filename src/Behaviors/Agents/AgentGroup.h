#ifndef AGENT_GROUP_H_
#define AGENT_GROUP_H_

#include "Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "Behaviors/Agents/ICreateChildAgent.h"
#include "Behaviors/Agents/PathFollowingChildAgent.h"
#include "MPLibrary/PMPL.h"


////////////////////////////////////////////////////////////////////////////////
/// This agent calls pmpl once and then follows the resulting path.
///
/// @WARNING This object currently only supports homogeneous robot teams, and
///          assumes a shared roadmap model.
////////////////////////////////////////////////////////////////////////////////
class AgentGroup : public Agent {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef RoadmapGraph<CfgType, WeightType>         GraphType;
    typedef typename GraphType::vertex_descriptor     VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@}
    ///@name Construction
    ///@{

    AgentGroup(Robot* const _r);

    virtual ~AgentGroup();

    ///@}
    ///@name Agent Interface
    ///@{

    /// Call PMPL to create a path for the agent to follow.
    virtual void Initialize() override;

    /// Follow the roadmap.
    virtual void Step(const double _dt) override;

    /// Get a helpers robot for the worker robot
    std::vector<Robot*>& GetHelpers();

    /// Get the charging locations for problem
    std::vector<pair<Cfg, bool>>& GetChargingLocations();

    /// Clean up.
    virtual void Uninitialize() override;

    ///@}

  private:

    ///@name
    ///@{

    /// Set the next task for each child agent.
    void SetNextChildTask();

    const Cfg& GetRandomRoadmapPoint() const;

    std::vector<Robot*> GetChildRobots();

    ///@}

  protected:

    ///@name Internal State
    ///@{

    MPLibrary* m_library{nullptr};   ///< The shared-roadmap planning library.

    MPSolution* m_solution{nullptr}; ///< The shared-roadmap solution.

    std::vector<PathFollowingChildAgent*> m_RobotGroup;

    std::vector<pair<Cfg, bool>> m_chargingLocations;  ///< pair of <chargingLocation, isFree>

    //std::unordered_map<Robot*, Cfg> m_availableHelpers; ///< Helper robots avaialable

    std::vector<Robot*> m_availableHelpers;
    //std::vector<ICreateChildAgent*> m_RobotGroup;

    ///@}

};

#endif

