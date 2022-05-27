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

    typedef GroupLocalPlan<Cfg>                       GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType> GroupRoadmapType;
    typedef size_t                                    VID;

    // Robot* is the object, size_t is the vid in the single object mode graph
    typedef std::map<Robot*,size_t> Mode;

    typedef std::map<GroupRoadmapType*,GroupCfg> Direction;

    struct Vertex {
      std::vector<std::pair<GroupRoadmapType*,VID>> cfgs;
      size_t modeID;

      bool operator==(const Vertex& _other) const {
        return cfgs == _other.cfgs and modeID == _other.modeID;
      }
    };

    struct Edge {
      std::vector<std::pair<GroupRoadmapType*,std::pair<VID,VID>>> transitions;
      double cost;

      bool operator==(const Edge& _other) const {
        return transitions == _other.transitions and cost == _other.cost;
      }
    };

    typedef GenericStateGraph<Vertex,Edge> TensorProductRoadmap;

    struct ActionExtendedState {
      VID vid; ///< VID in tensor product roadmap
      size_t ahid; ///< Action history id

      bool operator==(const ActionExtendedState& _other) const {
        return vid == _other.vid and ahid == _other.ahid;
      }
    };

    struct ActionExtendedEdge {
      double cost;
      bool operator==(const ActionExtendedEdge& _other) const {
        return cost == _other.cost;
      }
    };

    typedef GenericStateGraph<ActionExtendedState,ActionExtendedEdge> ActionExtendedGraph;

    typedef std::vector<size_t> ActionHistory;

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

    void CreateSMARTreeRoot();

    std::pair<size_t,size_t> SelectMode();

    std::pair<size_t,Direction> SelectVertex(size_t _modeID, 
                size_t _historyID, Mode _heuristic);

    size_t Extend(size_t _qNear, Direction _direction, size_t _modeID, 
                  size_t _historyID, Mode _heuristic);

    size_t Rewire(size_t _qNew, size_t _qNear, size_t _modeID, size_t _historyID);

    bool ValidConnection(const Vertex& _source, const Vertex& _target);

    bool CheckForModeSwitch(size_t _qNew);

    bool CheckForGoal(size_t _qNew);

    Direction GetRandomDirection(size_t _historyID);

    Direction GetHeuristicDirection(size_t _vid, size_t _modeID, Mode _heuristic);

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
    ///@name XML Parameters
    ///@{

    size_t m_maxIterations;

    double m_heuristicProb{0.5};

    double m_goalBias{0.5};

    std::string m_dmLabel;

    std::string m_cdLabel;

    ///@}
    ///@name Internal State
    ///@{

    std::map<size_t,HeuristicValues> m_cachedHeuristics;

    bool m_cachedMAPFGoals{false};

    std::map<Robot*,size_t> m_MAPFGoals;

    std::map<Robot*,size_t> m_MAPFStarts;

    std::vector<Mode> m_modes;

    std::unique_ptr<TensorProductRoadmap> m_tensorProductRoadmap;

    std::unique_ptr<ActionExtendedGraph> m_actionExtendedGraph;

    std::vector<ActionHistory> m_actionHistories;

    // Map from mode id to action history ids
    std::map<size_t,std::vector<size_t>> m_modeHistories;

    std::vector<size_t> m_biasedModes;

    // Map of history ID to relevant tensor product roadmap vids
    std::map<size_t,std::set<size_t>> m_historyVIDs;

    // Map of history ID to vid that made progress towards mode goal
    std::map<size_t,size_t> m_historyVIDBias;

    std::map<size_t,double> m_distanceMap;

    ///@}

};

std::ostream& operator<<(std::ostream& _os, const SMART::Vertex);
std::istream& operator>>(std::istream& _is, const SMART::Vertex);

std::ostream& operator<<(std::ostream& _os, const SMART::Edge);
std::istream& operator>>(std::istream& _is, const SMART::Edge);

std::ostream& operator<<(std::ostream& _os, const SMART::ActionExtendedState);
std::istream& operator>>(std::istream& _is, const SMART::ActionExtendedState);

std::ostream& operator<<(std::ostream& _os, const SMART::ActionExtendedEdge);
std::istream& operator>>(std::istream& _is, const SMART::ActionExtendedEdge);
#endif
