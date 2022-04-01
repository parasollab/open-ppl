#ifndef _PPL_DEPENDENT_PATHS_H_
#define _PPL_DEPENDENT_PATHS_H_

#include "InteractionStrategyMethod.h"
#include "TMPLibrary/StateGraphs/StateGraph.h"
class DependentPaths : public InteractionStrategyMethod {
  protected:
    ///@name Helper Functions
    ///@{

    std::vector<std::shared_ptr<GroupTask>> GenerateTasks(std::vector<std::string> _conditions, 
                            std::unordered_map<Robot*,Constraint*> _startConstraints,
                            std::unordered_map<Robot*,Constraint*> _goalConstraints);

    std::vector<Path*> PlanMotions(std::vector<std::shared_ptr<GroupTask>> _tasks, MPSolution* _solution, 
                                   std::string _label, const std::set<Robot*>& _staticRobots,
                                   const State& _state);

    State InterimState(Interaction* _interaction, const std::string& _current, 
                       const std::string& _next, const std::vector<Path*> _paths);

    ///@}
    ///@name Internal State
    std::string m_mpStrategyLabel;

    std::unordered_map<Robot*,Cfg> m_interimCfgMap;

    std::unordered_map<Robot*,std::vector<Cfg>> m_individualPaths;
 
    State m_finalState;

    ///@}
  public:
    ///@name Local Types
    ///@{

    typedef InteractionStrategyMethod::State                 State;
		typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;
    typedef PathType<MPTraits<Cfg,DefaultWeight<Cfg>>>       Path;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>>      GroupPathType;
    typedef GroupLocalPlan<Cfg>                              GroupWeightType;

    ///@}
    ///@name Construction
    ///@{

    DependentPaths();

    DependentPaths(XMLNode& _node);

    ~DependentPaths();

    ///@}
    ///@name Interface
    ///@{

    virtual bool operator()(Interaction* _interaction, State& _state) override;

    ///@}

  private:
    //For now we are using one sampler, will do two later
    string m_firstSamplerLabel;
    string m_restSamplerLabel;

};
#endif
