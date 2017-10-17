#ifndef PATH_FOLLOWING_AGENT_H_
#define PATH_FOLLOWING_AGENT_H_

#include "Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"
#include "Behaviors/Controllers/ICreateController.h"

class ControllerMethod;
////////////////////////////////////////////////////////////////////////////////
/// This agent calls pmpl once and then follows the resulting path.
////////////////////////////////////////////////////////////////////////////////
class PathFollowingAgent : public Agent {

  public:

    ///@name Construction
    ///@{

    PathFollowingAgent(Robot* const _r);

    virtual ~PathFollowingAgent();

    ///@}
    ///@name Agent Interface
    ///@{

    /// Call PMPL to create a path for the agent to follow.
    virtual void Initialize() override;

    /// Follow the path.
    virtual void Step(const double _dt) override;

    /// Clean up.
    virtual void Uninitialize() override;

    ControllerMethod* m_hardwareController{nullptr}; ///< Low-level controller.
    
    void UpdateOdometry(const double&, const double&, const double&);
    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::vector<double> m_odometry{0.0,0.0,0.0};

    std::vector<Cfg> m_path; ///< The path to follow.
    size_t m_pathIndex{0};   ///< The path node that is the current subgoal.

    MPLibrary* m_library{nullptr}; ///< This agent's planning library.

    ///@}

};

#endif
