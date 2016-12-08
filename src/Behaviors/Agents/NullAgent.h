#ifndef NULL_AGENT_H_
#define NULL_AGENT_H_

#include "Agent.h"


////////////////////////////////////////////////////////////////////////////////
/// The world's most boring agent. It does nothing on each time step.
////////////////////////////////////////////////////////////////////////////////
class NullAgent : public Agent {

  public:

    ///@name Construction
    ///@{

    NullAgent(Robot* const _r);
    virtual ~NullAgent();

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Do nothing.
    virtual void Initialize() override;

    /// Do nothing on each step.
    virtual void Step(const double _dt) override;

    /// Do nothing.
    virtual void Uninitialize() override;

    ///@}

};

#endif
