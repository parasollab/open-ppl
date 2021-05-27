#ifndef PPL_INTERACTION_STRATEGY_EXAMPLE_H_
#define PPL_INTERACTION_STRATEGY_EXAMPLE_H_

#include "InteractionStrategyMethod.h"

class InteractionStrategyExample : public InteractionStrategyMethod {
  public:
    ///@name Local Types
    ///@{

    typedef InteractionStrategyMethod::State State;
		typedef MPSolutionType<MPTraits<Cfg,
            DefaultWeight<Cfg>>> MPSolution;

    ///@}
    ///@name Construction
    ///@{

    InteractionStrategyExample();

    InteractionStrategyExample(XMLNode& _node);

    virtual ~InteractionStrategyExample();

    ///@}
    ///@name Interface
    ///@{

    virtual bool operator()(Interaction* _interaction, const State& _state) override;

    ///@}

  private:
    ///@name Helper Functions
    ///@{

    void AssignRoles(const State& _state, std::vector<std::string> _conditions);

    std::unordered_map<Robot*,Constraint*> GenerateConstraints(std::vector<std::string> _conditions);

    std::vector<GroupTask*> GenerateTasks(std::vector<std::string> _conditions, 
                            std::unordered_map<Robot*,Constraint*> _startConstraints,
                            std::unordered_map<Robot*,Constraint*> _goalConstraints);

    bool PlanMotions(std::vector<GroupTask*> _tasks, MPSolution* _solution, std::string _label);

    State InterimState(Interaction* _interaction);
    ///@}
    ///@name Internal State
    ///@{

    std::string m_mpStrategyLabel;

    std::unordered_map<std::string,Robot*> m_roleMap;

    std::unordered_map<Robot*,Cfg> m_interimCfgMap;
    ///@}
};

#endif
