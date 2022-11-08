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
    typedef TMPBaseObject::GroupCfgType                    GroupCfgType;
    typedef TMPBaseObject::GroupLocalPlanType              GroupLocalPlanType;
    typedef TMPBaseObject::GroupRoadmapType                GroupRoadmapType;
    typedef GroupPath<MPTraits<Cfg>>                       GroupPathType;
    typedef std::pair<std::pair<size_t,size_t>,size_t>     Constraint;
    typedef CBSNode<SemanticTask,Constraint,GroupPathType> Node;

    typedef std::set<Constraint> ConstraintSet;
    typedef std::map<SemanticTask*,ConstraintSet> ConstraintMap;

    typedef std::unordered_map<size_t,std::vector<Range<double>>> VertexIntervals;

    typedef std::unordered_map<size_t,std::unordered_map<
                  size_t,std::vector<Range<double>>>> EdgeIntervals;

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

    void SaveSolution(const Node& _node);

    void ComputeIntervals(SemanticTask* _task, const Node& _node);

    std::vector<Range<double>> ConstructSafeIntervals(std::vector<Range<double>> _unsafeIntervals);

    ///@}
    ///@name CBS Functors
    ///@{

    bool LowLevelPlanner(Node& _node, SemanticTask* _task);

    GroupPathType* QueryPath(SemanticTask* _task, const double _startTime,
                             const Node& _node);

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

    bool IsEdgeSafe(const VID _source, const VID _target, const Constraint _constraint,
                    const size_t _startTime) const;
 
    ConstraintSet::iterator LowerBound(size_t _bound) const;
    ConstraintSet::iterator UpperBound(size_t _bound) const;

    
 
    ///@name Internal State
    ///@{

    std::string m_queryLabel;

    std::string m_queryStrategy;

    double m_currentStartTime{0};

    std::string m_vcLabel;

    const ConstraintSet* m_currentConstraints{nullptr};
    std::set<ConstraintMap> m_constraintCache;

    std::string m_safeIntervalLabel;
    
    VertexIntervals m_vertexIntervals;
    EdgeIntervals m_edgeIntervals;

    bool m_savePaths{false};

    typedef std::map<size_t,std::vector<Range<double>>> UnsafeVertexIntervals;
    typedef std::map<std::pair<size_t,size_t>,std::vector<Range<double>>> UnsafeEdgeIntervals;

    std::vector<GroupCfgType> m_conflicts;
    std::vector<std::map<SemanticTask*,UnsafeVertexIntervals>> m_unsafeVertexIntervalMap;
    std::vector<std::map<SemanticTask*,UnsafeEdgeIntervals>> m_unsafeEdgeIntervalMap;

    double m_upperBound;

    std::string m_motionEvaluator;

    ///@}

};

#endif
