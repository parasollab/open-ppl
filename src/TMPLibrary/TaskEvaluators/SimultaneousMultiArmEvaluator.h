#ifndef PPL_SIMULTANEOUS_MULTI_ARM_EVALUATOR_H_
#define PPL_SIMULTANEOUS_MULTI_ARM_EVALUATOR_H_

#include "TaskEvaluatorMethod.h"

#include "TMPLibrary/ActionSpace/Condition.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/StateGraphs/ObjectCentricModeGraph.h"

class SimultaneousMultiArmEvaluator : public TaskEvaluatorMethod {
  public:

    ///@name LocalTypes
    ///@{

    typedef Condition::State                                State;

    typedef ObjectCentricModeGraph::ModeInfo                ModeInfo;
    typedef ObjectCentricModeGraph::ObjectMode              ObjectMode;
    typedef ObjectCentricModeGraph::ObjectModeSwitch        ObjectModeSwitch;
    typedef ObjectCentricModeGraph::GraphType               GraphType;
    typedef ObjectCentricModeGraph::SingleObjectModeGraph   SingleObjectModeGraph;
    typedef GraphType::VID                                  VID;

    typedef GroupLocalPlan<Cfg>                             GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>       GroupRoadmapType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>       TensorProductRoadmap;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>>     GroupPathType;
    typedef GroupLPOutput<MPTraits<Cfg,DefaultWeight<Cfg>>> GroupLPOutputType;
    typedef std::unordered_map<std::string,Robot*>          RoleMap;

    typedef std::vector<std::pair<GroupRoadmapType*,VID>>   TransitionVertex;
    typedef std::unordered_map<Robot*,std::vector<Cfg>>     InteractionPath;
    typedef std::map<TransitionVertex,
                 std::map<TransitionVertex,
                      InteractionPath*>>                    TransitionMap;

    struct TaskState {
      size_t vid; ///< TensorProductRoadmap VID
      size_t mode; ///< VID of current mode in ModeGraph

      bool operator==(const TaskState& _other) const {
        return vid == _other.vid and mode == _other.mode;
      }
    };

    struct TaskEdge {
      std::vector<std::pair<TransitionVertex,TransitionVertex>> transitions;
      double cost;

      bool operator==(const TaskEdge& _other) const {
        return cost == _other.cost and transitions == _other.transitions;
      }
    };

    typedef GenericStateGraph<TaskState,TaskEdge>        TaskGraph;
    typedef TaskGraph::VID                               TID;

    // Vector of mode graph vertices
    typedef std::vector<size_t>                          ActionHistory;
    typedef size_t                                       AHID;
    
    struct ActionExtendedState {
      TID vid;   ///< TaskGraph VID
      AHID ahid; ///< ActionHistory ID

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

    typedef GenericStateGraph<ActionExtendedState,
                              ActionExtendedEdge>        ActionExtendedGraph;

    typedef GenericStateGraph<std::pair<size_t,size_t>,double> HeuristicSearch;
    typedef std::pair<std::pair<size_t,size_t>,size_t>           CBSConstraint;
    typedef std::vector<size_t>                                  CBSSolution;
    typedef CBSNode<Robot,CBSConstraint,CBSSolution>             CBSNodeType;

    ///@}
    ///@name Construction
    ///@{

    SimultaneousMultiArmEvaluator();

    SimultaneousMultiArmEvaluator(XMLNode& _node);

    virtual ~SimultaneousMultiArmEvaluator();

    ///@}
    ///@name Task Evaluator Interface

    virtual void Initialize() override;

    ///@}

  protected:

    ///@name Helper Functions
    ///@{

    ///Exectute
    ///@param _plan pointer
    ///@return True if exectuion is successful
    virtual bool Run(Plan* _plan = nullptr) override;

    size_t CreateRootNodes();

    std::pair<size_t,size_t> SelectMode();

    std::set<VID> GetModeNeighbors(VID _vid);

    bool SampleTransition(VID _source, VID _target);

    void ConnectToExistingRoadmap(Interaction* _interaction, State& _state, State& _end, 
                                  bool _reverse, size_t _sourceMode, size_t _targetMode);
    
    VID AddToRoadmap(GroupCfg _cfg);

    VID CreateTensorProductVertex(const std::vector<GroupCfg>& _cfgs);

    TID Select(size_t _modeID, size_t _history, std::unordered_map<Robot*,size_t> _heuristic);

    GroupCfg SampleVertex(size_t _modeID);

    TID Extend(TID _qNear, size_t _history, std::unordered_map<Robot*,size_t> _heuristic);

    GroupCfg GetHeuristicDirection(size_t _mode, std::unordered_map<Robot*,size_t> _heuristic);

    TID Rewire(TID _qNew, size_t _history);

    size_t AddToActionExtendedGraph(TID _qBest, TID _qNew, size_t _history);

    void CheckForModeTransition(size_t _aid, size_t _history);

    void CheckForGoal(size_t _aid);

    std::vector<GroupCfg> SplitTensorProductVertex(GroupCfg _cfg, size_t _modeID);

    size_t AddHistory(const ActionHistory& _history);

    ///@}
    ///@name Heuristic Functions
    ///@{

    void ComputeGoalBiasHeuristic();

    std::unordered_map<Robot*,size_t> ComputeMAPFSolution(ObjectMode _objectMode);

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

    size_t m_maxIters; ///< Number of iterations to look for a path.

    size_t m_maxAttempts; ///< Number of attempts to sample a transition.

    /// Label for connector method to use in connecting transitions.
    std::string m_connectorLabel; 

    /// Label for compute distance between two group cfgs
    std::string m_dmLabel;

    /// Label for verifying local plans in TPR
    std::string m_lpLabel;

    /// TensorProductRoadmap for whole system.
    std::unique_ptr<TensorProductRoadmap> m_tensorProductRoadmap; 

    /// Cached interaction paths.
    std::vector<std::unique_ptr<InteractionPath>> m_interactionPaths;

    /// Map from transition starts to transition ends to interaction path.
    TransitionMap m_transitionMap;

    /// Graph of task vertices (mode extended tensor product roadmap).
    std::unique_ptr<TaskGraph> m_taskGraph;

    /// Task graph extended to track action history.
    std::unique_ptr<ActionExtendedGraph> m_actionExtendedGraph;

    /// Set of all action histories. Will be accessed by index.
    std::vector<ActionHistory> m_actionHistories;

    /// Map of mode graph vid to all action history indices that reach it.
    std::unordered_map<size_t,std::vector<size_t>> m_modeHistories;

    /// Map of task vertices explored within a particulr history
    std::unordered_map<size_t,std::set<TID>> m_historyVertices;

    std::vector<std::pair<bool,RoleMap>> m_plannedInteractions;

    std::unordered_map<Robot*,size_t> m_heuristicStarts;
    std::unordered_map<Robot*,size_t> m_heuristicGoals;

    std::map<std::pair<size_t,size_t>,size_t> m_modeVertexBias;

    double m_heuristicProb{.5};

    double m_goalBias{.5};

    std::unordered_map<size_t,double> m_goalBiasCosts;
    std::vector<size_t> m_orderedModesToGoal;

    ///@}
};

std::ostream& operator<<(std::ostream& _os, const SimultaneousMultiArmEvaluator::TaskState);
std::istream& operator>>(std::istream& _is, const SimultaneousMultiArmEvaluator::TaskState);

std::ostream& operator<<(std::ostream& _os, const SimultaneousMultiArmEvaluator::TaskEdge);
std::istream& operator>>(std::istream& _is, const SimultaneousMultiArmEvaluator::TaskEdge);

std::ostream& operator<<(std::ostream& _os, const SimultaneousMultiArmEvaluator::ActionExtendedState);
std::istream& operator>>(std::istream& _is, const SimultaneousMultiArmEvaluator::ActionExtendedState);

std::ostream& operator<<(std::ostream& _os, const SimultaneousMultiArmEvaluator::ActionExtendedEdge);
std::istream& operator>>(std::istream& _is, const SimultaneousMultiArmEvaluator::ActionExtendedEdge);

#endif
