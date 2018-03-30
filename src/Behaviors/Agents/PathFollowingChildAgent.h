#ifndef PATH_FOLLOWING_CHILD_AGENT_H_
#define PATH_FOLLOWING_CHILD_AGENT_H_

#include "PathFollowingAgent.h"

//#include "ConfigurationSpace/Cfg.h"
//#include "MPLibrary/PMPL.h"

class BatteryConstrainedGroup;
class NetbookInterface;


////////////////////////////////////////////////////////////////////////////////
/// This agent follows a set of tasks and executes the helper worker behavior.
////////////////////////////////////////////////////////////////////////////////
class PathFollowingChildAgent : public PathFollowingAgent {

  public:

    ///@name Construction
    ///@{

    PathFollowingChildAgent(Robot* const _r);

    PathFollowingChildAgent(Robot* const _r, XMLNode& _node);

    virtual ~PathFollowingChildAgent();

    ///@}
    ///@name Child Interface
    ///@{

    /// Check if the agent's battery is considered to be below the 'low'
    /// threshold.
    bool IsBatteryLow();

    /// Check if the agent's battery is considered to be above the 'high'
    /// threshold.
    bool IsBatteryHigh();

    ///@}
    
    ///@name Internal State Functions
    ///@{
   
    ///Set m_parentAgent to the passed in group controller
    ///@param _parent The parent agent.
    void SetParentAgent(BatteryConstrainedGroup* const _parent);

    virtual bool IsChild() const override;

    /// Finds the battery break along its path and communicates it with the
    /// parent agent.
    void SetBatteryBreak();
    ///@}

  protected:

    ///@name Helper Functions
    ///@{

    void Rotate(double&, double);

    ///@}
    ///@name Planning Helpers
    ///@{
    
    virtual void WorkFunction(std::shared_ptr<MPProblem> _problem) override;

    ///@}
    ///@name Task Helpers
    ///@{
    
    virtual bool SelectTask() override;

    // TODO: Remove this.
    // virtual bool EvaluateTask() override;

    ///@}
    ///@name Controller Helpers
    ///@{

    virtual void ExecuteControls(const ControlSet& _c, const size_t _steps) override;

    ///@}

    ///@name Internal State
    ///@{

    BatteryConstrainedGroup* m_parentAgent{nullptr};

    double m_distance{0.0}; ///< The distance traveled since localizing.

    bool reactive{true};

    ///@}
    //

};

#endif
