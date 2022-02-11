#ifndef PPL_HANDOFF_STRATEGY_PPL_
#define PPL_HANDOFF_STRATEGY_PPL_

#include "GraspStrategy.h"

class HandoffStrategy : public GraspStrategy {

  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Construction
    ///@{

    HandoffStrategy();

    HandoffStrategy(XMLNode& _node);

    ~HandoffStrategy();

    ///@}
    ///@name Interface
    ///@{

    virtual bool operator()(Interaction* _interaction, State& _start) override;

    ///@}

  protected:
    ///@name Helper Functions
    ///@{

    State GenerateTransitionState(Interaction* _interaction, const State& _previous, const size_t _next, MPSolution* _solution);

    State GenerateInitialState(Interaction* _interaction, const State& _previous, const size_t _next);

    virtual std::vector<std::shared_ptr<GroupTask>> GenerateTasks(
              std::vector<std::string> _conditions, 
              std::unordered_map<Robot*,Constraint*> _startConstraints,
              std::unordered_map<Robot*,Constraint*> _goalConstraints) override;

    void GeneratePathConstraints(std::vector<std::string> _startConditions, 
                                 std::vector<std::string> _endConditions);
    ///@}
    ///@name Internal State
    ///@{

    std::unordered_map<Robot*,Constraint*> m_pathConstraintMap;

    ///@}

};

#endif
