#ifndef PPL_SCHEDULED_CBS_H_
#define PPL_SCHEDULED_CBS_H_

#include "TaskEvaluatorMethod.h"

#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupPath.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "Traits/CfgTraits.h"

#include "Utilities/CBS.h"

class ScheduledCBS : public TaskEvaluatorMethod {

  public:

    ///@name Local Types
    ///@{

    typedef MPTraits<Cfg>::GroupRoadmapType                GroupRoadmapType;
    typedef MPTraits<Cfg>::GroupPathType                   GroupPathType;
    typedef std::pair<std::pair<size_t,size_t>,size_t>     Constraint;
    typedef CBSNode<SemanticTask,Constraint,GroupPathType> Node;

    ///@}
    ///@name Construction
    ///@{

    ScheduledCBS();

    ScheduledCBS(XMLNode& _node);

    ~ScheduledCBS();

    ///@}
    ///@name Task Evaluator Overrides
    ///@{

    virtual void Initialize() override;

    ///@}

    //TODO::Add function to set upper bound for quitting early 

  private:

    ///@name Overrides
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    ///@}
    ///@name CBS Functions
    ///@{

    bool LowLevelPlanner(Node& _node, SemanticTask* _task);

    std::vector<std::pair<SemanticTask*,Constraint>> ValidationFunction(Node& _node);

    double CostFunction(Node& _node);

    void InitialSolutionFunction(std::vector<Node>& _root, std::vector<SemanticTask*> _tasks,
                        CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
                        CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost);

    ///@}
    ///@name Helper Functions
    ///@{

    GroupPathType* QueryPath(SemanticTask* _task, const double _startTime,
                             const Node& _node);

    void ConvertToPlan(const Node& _node);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_vcLabel;

    std::string m_queryLabel;

    std::string m_queryStrategy;
    ///@}

};

#endif