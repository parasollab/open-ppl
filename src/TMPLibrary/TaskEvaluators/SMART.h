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

    struct HeuristicValues {
      Mode nextMode;
      double costToGo;
    };

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

    void ComputeGoalBias(size_t _modeID);

    void CreateSMARTreeRoot();

    size_t Select();

    size_t Extend();

    size_t Rewire();

    ///@}
    ///@name MAPF Heuristic Functions
    ///@{

    HeuristicValues ComputeMAPFHeuristic(size_t _modeID);

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

    std::map<size_t,HeuristicValues> m_cachedHeuristics;

    bool m_cachedMAPFGoals{false};

    std::map<Robot*,size_t> m_MAPFGoals;

    std::map<Robot*,size_t> m_MAPFStarts;

    std::vector<Mode> m_modes;

    ///@}

};

#endif
