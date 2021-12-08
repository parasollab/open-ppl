#ifndef PPL_NEXT_BEST_SEARCH_H_
#define PPL_NEXT_BEST_SEARCH_H_

#include "TMPStrategyMethod.h"

#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupPath.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "Traits/CfgTraits.h"

#include "Utilities/CBS.h"

class Decomposition;

class NextBestSearch : public TMPStrategyMethod {

  public:

    ///@name Local Types
    ///@{

    typedef size_t                                         VID;
    typedef MPTraits<Cfg>::GroupRoadmapType                GroupRoadmapType;
    typedef MPTraits<Cfg>::GroupPathType                   GroupPathType;
    typedef std::pair<size_t,GroupCfg>                     Constraint;
    typedef CBSNode<SemanticTask,Constraint,GroupPathType> Node;

    typedef std::set<Constraint> ConstraintSet;
    typedef std::map<SemanticTask*,ConstraintSet> ConstraintMap;

    ///@}
    ///@name Construction
    ///@{

    NextBestSearch();

    NextBestSearch(XMLNode& _node);

    virtual ~NextBestSearch();

    ///@}
    ///@name Interface
    ///@{

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    virtual void PlanTasks() override;

    double FindTaskPlan(Decomposition* _decomp);

    void ComputeMotions(Node& _bestNode);

    ///@}
    ///@name CBS Functors
    ///@{

    bool LowLevelPlanner(Node& _node, SemanticTask* _task);

    GroupPathType* QueryPath(SemanticTask* _task, const double& _startTime);

    std::vector<std::pair<SemanticTask*,Constraint>> ValidationFunction(Node& _node);

    double CostFunction(Node& _node);

    std::vector<Node> SplitNodeFunction(Node& _node, 
                        std::vector<std::pair<SemanticTask*,Constraint>> _constraints,
                        CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
                        CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost);

    void InitialSolutionFunction(std::vector<Node>& _root, std::vector<SemanticTask*> _tasks,
                        CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
                        CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost);

    double RobotGroupPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
              const double _sourceTimestep, const double _bestTimestep) const;

    bool IsEdgeSafe(const VID _source, const VID _target, const GroupCfg& _conflictCfg) const;
  
    ///@name Internal State
    ///@{

    std::string m_queryStrategy;

    double m_currentStartTime{0};

    std::string m_vcLabel;

    const ConstraintSet* m_currentConstraints{nullptr};
    std::set<ConstraintMap> m_constraintCache;

    ///@}

};

#endif
