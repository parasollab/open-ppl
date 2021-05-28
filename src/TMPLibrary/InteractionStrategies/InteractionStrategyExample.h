#ifndef PPL_INTERACTION_STRATEGY_EXAMPLE_H_
#define PPL_INTERACTION_STRATEGY_EXAMPLE_H_

#include "InteractionStrategyMethod.h"

#include "ConfigurationSpace/GroupPath.h"

class InteractionStrategyExample : public InteractionStrategyMethod {
  public:
    ///@name Local Types
    ///@{

    typedef InteractionStrategyMethod::State State;
		typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>> GroupPathType;
    typedef GroupLocalPlan<Cfg> GroupWeightType;

    ///@}
    ///@name Construction
    ///@{

    InteractionStrategyExample();

    InteractionStrategyExample(XMLNode& _node);

    virtual ~InteractionStrategyExample();

    ///@}
    ///@name Interface
    ///@{

    virtual bool operator()(Interaction* _interaction, State& _state) override;

    ///@}

  private:
    ///@name Helper Functions
    ///@{

    void AssignRoles(const State& _state, std::vector<std::string> _conditions);

    std::unordered_map<Robot*,Constraint*> GenerateConstraints(std::vector<std::string> _conditions);

    std::vector<GroupTask*> GenerateTasks(std::vector<std::string> _conditions, 
                            std::unordered_map<Robot*,Constraint*> _startConstraints,
                            std::unordered_map<Robot*,Constraint*> _goalConstraints);

    GroupPathType* PlanMotions(std::vector<GroupTask*> _tasks, MPSolution* _solution, std::string _label);

    GroupPathType* ConstructCompositePath(MPSolution* _solution);

    State InterimState(Interaction* _interaction);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_mpStrategyLabel;

    std::string m_dmLabel;

    std::string m_lpLabel;

    std::unordered_map<std::string,Robot*> m_roleMap;

    std::unordered_map<Robot*,Cfg> m_interimCfgMap;

    std::unordered_map<Robot*,std::vector<Cfg>> m_individualPaths;
 
    State m_finalState;

    ///@}
};

#endif
