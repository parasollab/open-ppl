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

    typedef MPTraits<Cfg>::GroupRoadmapType                   GroupRoadmapType;
    typedef MPTraits<Cfg>::GroupPathType                      GroupPathType;

    // Edge <Source, Target>, Time Interval <Start, End> 
    typedef std::pair<std::pair<size_t,size_t>,Range<size_t>> Constraint;
    typedef CBSNode<SemanticTask,Constraint,GroupPathType>    Node;

    typedef std::map<size_t,std::vector<Range<size_t>>> VertexIntervals;
    typedef std::map<std::pair<size_t,size_t>,std::vector<Range<size_t>>> EdgeIntervals;

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
    void SetUpperBound(double _upperBound); 

  private:

    ///@name Overrides
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    ///@}
    ///@name CBS Functions
    ///@{

    bool LowLevelPlanner(Node& _node, SemanticTask* _task);

    std::vector<std::pair<SemanticTask*,Constraint>> ValidationFunction(Node& _node);

    std::vector<Node> SplitNodeFunction(Node& _node, 
          std::vector<std::pair<SemanticTask*,Constraint>> _constraints, 
          CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
          CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost);

    double CostFunction(Node& _node);

    void InitialSolutionFunction(std::vector<Node>& _root, std::vector<SemanticTask*> _tasks,
                        CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
                        CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost);

    ///@}
    ///@name Helper Functions
    ///@{

    GroupPathType* QueryPath(SemanticTask* _task, const size_t _startTime,
                             const Node& _node);

    void ConvertToPlan(const Node& _node, Plan* _plan);

    size_t FindStartTime(SemanticTask* _task, std::set<SemanticTask*> _solved, 
                         std::map<SemanticTask*,size_t> _endTimes);

    void ComputeIntervals(SemanticTask* _task, const Node& _node);

    std::vector<Range<size_t>> ConstructSafeIntervals(std::vector<Range<size_t>>& _unsafeIntervals);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_vcLabel;

    std::string m_queryLabel;

    std::string m_queryStrategy;

    std::map<GroupPathType*,size_t> m_startTimes;
    std::map<GroupPathType*,size_t> m_endTimes;

    double m_upperBound;

    typedef std::map<size_t,std::vector<Range<size_t>>> UnsafeVertexIntervals;
    typedef std::map<std::pair<size_t,size_t>,std::vector<Range<size_t>>> UnsafeEdgeIntervals;

    std::vector<std::map<SemanticTask*,UnsafeVertexIntervals>> m_unsafeVertexIntervalMap;
    std::vector<std::map<SemanticTask*,UnsafeEdgeIntervals>> m_unsafeEdgeIntervalMap;

    VertexIntervals m_vertexIntervals;
    EdgeIntervals m_edgeIntervals;

    ///@}

};

#endif
