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

    typedef Condition::State                             State;

    typedef ObjectCentricModeGraph::ObjectMode           ObjectMode;
    typedef ObjectCentricModeGraph::ObjectModeSwitch     ObjectModeSwitch;
    typedef ObjectCentricModeGraph::GraphType            GraphType;
    typedef GraphType::VID                               VID;

    typedef GroupLocalPlan<Cfg>                          GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>    GroupRoadmapType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>    TensorProductRoadmap;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>>  GroupPathType;

    typedef std::vector<std::pair<GroupRoadmapType*,VID>>           TransitionVertex;
    typedef std::unordered_map<Robot*,std::vector<Cfg>>  InteractionPath;
    typedef std::map<TransitionVertex,
                 std::map<TransitionVertex,
                      InteractionPath*>>                 TransitionMap;

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

    typedef std::vector<TID>                             ActionHistory;
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

    void ConnectToExistingRoadmap(Interaction* _interaction, State& _state, State& _end, bool _reverse);
    
    VID AddToRoadmap(GroupCfg _cfg);

    VID CreateTensorProductVertex(const std::vector<GroupCfg>& _cfgs);

    ///@}
    ///@name Internal State
    ///@{

    size_t m_maxIters; ///< Number of iterations to look for a path.

    size_t m_maxAttempts; ///< Number of attempts to sample a transition.

    /// Label for connector method to use in connecting transitions.
    std::string m_connectorLabel; 

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
