#ifndef I_CREATE_AGENT_H_
#define I_CREATE_AGENT_H_

#include <string>

#include "RoadmapFollowingAgent.h"


class ICreateAgent : public RoadmapFollowingAgent {

  public:

    /// @param _r The PMPL robot.
    /// @param _ip The IP of the create's netbook.
    ICreateAgent(Robot* const _r, const std::string& _ip);

  protected:

    virtual void ApplyCurrentControls() override;

    const std::string m_ip;

};
#endif
