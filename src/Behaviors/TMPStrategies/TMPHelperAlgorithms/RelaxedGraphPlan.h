#ifndef RELAXED_GRAPH_PLAN_H_
#define RELAXED_GRAPH_PLAN_H_

#include "Vector.h"

#include <unordered_map>

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "ConfigurationSpace/Weight.h"
#include "MPLibrary/PMPL.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Robot.h"
#include "../Actions/Action.h"
#include "../State.h"


class Cfg;
class MPTask;
class Robot;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Implementation of the Relaxed GRAPHPLAN as described in the
/// paper: The FF Planning System: Fast Plan Generation Through
/// Heuristic Search
///
////////////////////////////////////////////////////////////////////////////////


struct FactLayer {
  std::unordered_map<Robot*, std::set<const Boundary*>> m_possibleRobotLocations;

  //TODO:: Adjust for multiple objects
  std::set<const Boundary*>                             m_possibleObjectLocations;

  //TODO:: Adjust for multiple objects
  std::set<Robot*>                                      m_possibleRobotPayloads;
};

struct ActionLayer {
  std::vector<std::shared_ptr<Action>> m_actions;
};

class RelaxedGraphPlan {

  public:

    typedef std::unordered_map<std::string,RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> CapabilityMap;

    ///@name Construction
    ///@{

    /// @param _start The start state of the problem
    /// @param _goal The goal state of the problem
    /// @param _robots The robots availbale to solve the problem
    /// @param _handoffLocations The IT locations in the system
    /// @param _initialLocation The initial locatino of the task
    /// @param _goalLocation The goal location of the task
    /// @param _capabilityMap The map of robot capabilities to roadmaps
    /// @param _library The library used for MP checks
    /// @param _manipulator Indicates the type of robot team available
    RelaxedGraphPlan(State _start,
                     State _goal,
                     std::vector<Robot*> _robots,
                     std::set<std::pair<const Boundary*, const Boundary*>> _handoffLocations,
                     const Boundary* _initialLocation,
                     const Boundary* _goalLocation,
                     const CapabilityMap& _capabilityMap,
                     MPLibrary* _library,
                     bool _manipulator);

    virtual ~RelaxedGraphPlan();

    ///@}
    ///@name Interface
    ///@{

    /// Returns the action plan found using the relaxed graph plan heuristic
    std::list<std::shared_ptr<Action>> Heuristic();

    //TODO:: Right Now just returns possible actions in the first action layer
    /// Returns the set of recommended actions to consider next in formulating
    /// an action plan
    std::vector<std::shared_ptr<Action>> RecommendedActions();

    ///@}

  protected:

    ///@name Helper Functions
    ///@{

    /// Creates the intial fact and action layers in the forward portion of the
    /// planning
    void CreateGraphLayers();

    /// Adds a robot location to the input fact layer
    /// @param  _robot Robot being added at the location to the fact layer
    /// @param _location Location the robot is being added at to the fact layer
    /// @param _newFacts Fact layer that the robot location is being added to
    void AddRobotLocation(Robot* _robot, const Boundary* _location, FactLayer* _newFacts);

    /// Adds an object/task location to the input fact layer
    /// @param _location Location the object/task can now be at in this fact
    /// layer
    /// @param _newFacts Fact layer being updated with this task/object location
    void AddObjectLocation(const Boundary* _location, FactLayer* _newFacts);

    /// Adds a task/object owner to the input fact layer
    /// @param _robot Robot that cna now own the task/object at this fact layer
    /// @param _newFacts Fact layer that is being updated with this owner
    void AddObjectOwner(Robot* _robot, FactLayer* _newFacts);

    /// Works backwards through the graph layers to find a recommended set of
    /// actions
    void BackTrace();

    /// Checks if the goal state has been satisified
    bool IsAtGoal();

    /// Resolves parallel actions from the backtrace and returns linearized
    /// action plan
    std::list<std::shared_ptr<Action>> LinearizePlan();

    ///@}

    ///@name Internal State
    ///@{

    bool m_debug{true};               ///< Toggle debug messages.

    ///@}

    /// Fact and action layers developed in the forward plan. Layers indicate
    /// all facts and actions that can be true at that layer
    std::vector<std::pair<FactLayer*, ActionLayer*>> m_layerGraph;

    /// Keeps track of when goal facts can be achieved
    std::vector<FactLayer*>                         m_goalLayers;

    /// Recommended set of next actions to take to achieve goal state
    std::vector<std::shared_ptr<Action>>            m_helpfulActions;

    State m_start; ///< Starting state of the problem
    State m_goal; ///< Goal state of the problem

    std::vector<std::shared_ptr<Action>> m_allActions; ///< All possible actions in the system

    std::vector<const Boundary*> m_allLocations; ///< All possible robot locations in the system

    FactLayer* m_allFacts{nullptr}; ///< All the facts that have been found to be true/achievable

    std::vector<ActionLayer*> m_actionPlan; ///< Parallel action plan found by backtrace

    std::vector<FactLayer*>   m_trueFacts; ///< Facts that are found to be true during the plan

    const CapabilityMap& m_capabilityMap; ///< Roadmaps for each capability in the problem used for mp

    MPLibrary* m_library{nullptr}; ///< Library used for mp checks

    FactLayer* m_finalGoalLayer{nullptr}; ///< Goal Layer representing end state

    std::list<std::shared_ptr<Action>> m_linearActions; ///< Linear action plan

    bool m_manipulator; ///< Indicates the type of robot available
};

#endif
