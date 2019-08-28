#ifndef DUMMY_AGENT_H_
#define DUMMY_AGENT_H_

#include "PathFollowingAgent.h"

#include "ConfigurationSpace/Cfg.h"


////////////////////////////////////////////////////////////////////////////////
/// Same as path following agent except it doesn't try to plan ever.
/// Localizing is currently commented out. 
//////////////////////////////////////////////////////////////////////////////////
class DummyAgent : public PathFollowingAgent {

  public:

    ///@name Construction
    ///@{

    DummyAgent(Robot* const _r);

    DummyAgent(Robot* const _r, const DummyAgent& _a);

    DummyAgent(Robot* const _r, XMLNode& _node);

    virtual ~DummyAgent();

    ///@}
    ///@name Agent Interface
    ///@{

    virtual void Step(const double _dt) override;

		virtual bool EvaluateTask() override;
    ///@}

  protected:

		size_t m_pathID;

};

#endif
