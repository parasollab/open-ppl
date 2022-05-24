#ifndef PPL_SMART_H_
#define PPL_SMART_H_

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"

class SMART : public TaskEvaluatorMethod {

  public: 
    ///@name Local Types
    ///@{

    typedef GroupLocalPlan<Cfg>                                  GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>            GroupRoadmapType;
    
    typedef size_t                                                       VID;
    typedef std::vector<std::pair<GroupRoadmapType*,VID>>                Vertex;
    typedef std::vector<std::pair<GroupRoadmapType*,std::pair<VID,VID>>> Edge;

    typedef GenericStateGraph<Vertex,Edge> TensorProductRoadmap;

    // Robot* is the object, size_t is the vid in the single object mode graph
    typedef std::map<Robot*,size_t> Mode;

    ///@name MAPF Heuristic Type
    ///@{

    typedef GenericStateGraph<std::pair<size_t,size_t>,double> HeuristicSearch;
    typedef std::vector<size_t>                                CBSSolution;
    typedef std::pair<std::pair<size_t,size_t>,size_t>         CBSConstraint;
    typedef CBSNode<Robot,CBSConstraint,CBSSolution>           CBSNodeType;

    ///@}
    ///@}
    ///@name Construction
    ///@{

    SMART();

    SMART(XMLNode& _node);

    virtual ~SMART() = default;

    ///@}
    ///@name Task Evaluator Interface
    ///@{

    virtual void Initialize() override;

    ///@}
  protected:

    ///@name Helper Functions
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    ///@}
    ///@name MAPF Heuristic Functions
    ///@{

    Mode ComputeMAPFHeuristic(Mode mode);

    bool LowLevelPlanner(CBSNodeType& _node, Robot* _robot);

    std::vector<std::pair<Robot*,CBSConstraint>> ValidationFunction(CBSNodeType& _node);

    double CostFunction(CBSNodeType& _node);

    std::vector<CBSNodeType> SplitNodeFunction(CBSNodeType& _node,
        std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
        CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
        CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost);

    ///@}
    ///@name Internal State
    ///@{

    std::map<Mode,Mode> m_cachedHeuristics;

    ///@}

};

#endif
