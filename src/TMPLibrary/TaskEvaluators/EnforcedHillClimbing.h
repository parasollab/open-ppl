#ifndef ENFORCED_HILL_CLIMBING_H_
#define ENFORCED_HILL_CLIMBING_H_

//#include "MPProblem/Robot/Robot.h"

#include "TMPLibrary/Actions/Action.h"
#include "TMPLibrary/State.h"
#include "TMPLibrary/TaskEvaluators/Heuristics/RelaxedGraphPlan.h"
#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"
#include "TMPLibrary/TMPTools/InteractionTemplate.h"

#include "Vector.h"
#include <containers/sequential/graph/graph.h>

////////////////////////////////////////////////////////////////////////////////
/// Implementation of the Relaxed GRAPHPLAN as described in the
/// paper: The FF Planning System: Fast Plan Generation Through
/// Heuristic Search
///
////////////////////////////////////////////////////////////////////////////////

class EnforcedHillClimbing : public TaskEvaluatorMethod {

  public:

    ///@name Local Types
    ///@{

    typedef std::unordered_map<std::string,RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> CapabilityMap;

    ///@name Construction
    ///@{

    EnforcedHillClimbing();
    EnforcedHillClimbing(XMLNode& _node);

    /// @param _capabilityMap Set of roadmaps for each cabaility to be used in
    /// @param _robots Set of robots available to solve task
    /// @param _task Task to solve
    /// @param _handoffs ITs in the system
    /// @param _library MPLibrary utilized by the coordinator calling this
    /// @param _manipulator Indicates the type of robot team being used
    EnforcedHillClimbing(const CapabilityMap& _capabilityMap,
                          std::vector<Robot*> _robots,
                          MPTask* _task,
                          std::vector<std::unique_ptr<InteractionTemplate>>& _handoffs,
                          MPLibrary* _library,
                          bool _manipulator);

    virtual ~EnforcedHillClimbing();

    ///Returns a sequence of actions to solve the problem
    std::vector<std::shared_ptr<Action>> Solve();

    ///@}

  protected:

    ///@name Helper Functions
    ///@{

    virtual bool Run(Plan* _plan= nullptr) override;

    /// Converts a cfg to a bundary that can be used as a location for the TP
    /// logic
    /// @param _cfg Cfg of the initial point being converted to a boundary
    Boundary* CfgToBoundary(Cfg _cfg);

    ///Checks if state has already been considered by the algorithm at the
    ///current step
    /// @param _state State that is ebing checked for duplication
    bool DuplicateState(State _state);

    ///@}
    ///@name Internal State
    ///@{

    bool m_debug{true}; ///< Toggle debug messages.

    ///@}

    CapabilityMap m_capabilityMap; ///< Set of capability maps for the robot team

    State m_start; ///< Start state of the problem
    State m_goal;  ///< Goal state of the problem

    std::vector<Robot*> m_robots; ///< Set of robots available to solve the task

    /// Set of ITs in the system
    std::set<std::pair<const Boundary*, const Boundary*>> m_handoffLocations;

    const Boundary* m_initialLocation{nullptr}; ///< Initial location of the task
    const Boundary* m_goalLocation{nullptr};    ///< Goal location of the task

    std::vector<State> m_visitedStates; ///< States that have already been considered in the algorithm

    MPLibrary* m_library{nullptr};

    bool m_manipulator; ///< Indicates robot team available
};

#endif
