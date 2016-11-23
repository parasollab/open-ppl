#ifndef PATH_FOLLOWING_AGENT_H_
#define PATH_FOLLOWING_AGENT_H_

#include "Agent.h"


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
    virtual void Step() override;

    /// Clean up.
    virtual void Uninitialize() override;

    ///@}

};

#endif
