#ifndef ICREATE_CHILD_AGENT_H_
#define ICREATE_CHILD_AGENT_H_

#include "Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"


////////////////////////////////////////////////////////////////////////////////
/// This agent calls pmpl once and then follows the resulting path.
////////////////////////////////////////////////////////////////////////////////
class ICreateChildAgent : public Agent {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef RoadmapGraph<CfgType, WeightType>         GraphType;
    typedef typename GraphType::vertex_descriptor     VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@}
    ///@name Construction
    ///@{

    ICreateChildAgent(Robot* const _r);

    virtual ~ICreateChildAgent();

    ///@}
    ///@name Agent Interface
    ///@{

    /// Call PMPL to create a path for the agent to follow.
    virtual void Initialize() override;
    
    void InitializeMpSolution(MPSolution*);

    /// Follow the roadmap.
    virtual void Step(const double _dt) override;

    /// Clean up.
    virtual void Uninitialize() override;
    
    /// Set solution
    void SetTask(MPTask* _task);

    /// Set solution
    void SetMPRoadmap(RoadmapType* _solution);
    
    void SetMPLibrary(MPLibrary* _library);
    ///@}
    MPSolution* m_solution{nullptr}; ///< The solution with the roadmap to follow.

    MPTask* GetNewTask();
  private:

    ///@name Helpers
    ///@{

    /// Check that the simulated robot state matches the expected state from the
    /// roadmap.
    void CheckRobot() const;


    /// Set the next subgoal in the path.
    void SetNextSubgoal();

    /// Set the next set of controls to follow.
    void SetNextControls();

    /// Check if the path traversal is complete.
    bool PathCompleted() const noexcept;

    
    ///@}

  protected:

    /// Continue following the current controls.
    virtual void ApplyCurrentControls();

    ///@name Internal State
    ///@{

    MPLibrary* m_library{nullptr};   ///< This agent's planning library.


    VIDIterator m_currentSubgoal;    ///< The current subgoal in the path.

    /// The number of steps left to repeat the current control(s).
    size_t m_stepsRemaining{0};

    const WeightType* m_edge{nullptr}; ///< The current edge being traversed.

    MPTask* m_task{nullptr};

    ///@}

};

#endif

