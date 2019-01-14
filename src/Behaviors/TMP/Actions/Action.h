#ifndef ACTION_H_
#define ACTION_H_

#include "../State.h"
#include <vector>

#include "MPProblem/Robot/Robot.h"
#include "MPLibrary/PMPL.h"

struct FactLayer;

////////////////////////////////////////////////////////////////////////////////
/// Facts to be used in TMP methods
////////////////////////////////////////////////////////////////////////////////


class Action {

  public:

    ///@name Construction
    ///@{

    Action();

    virtual ~Action();

    ///@}
    ///@name Interface Functions
    ///@{
    //
    /// Checks if the preconditions for this action are met in the given fact
    /// layer
    /// @param _factLayer The possible state that is the action could be
    /// performed from
    virtual bool CheckPreConditions(const FactLayer* _factLayer);

    /// Sets the layer membership of this action
    /// @param _layerMembership The action layer that this action is assigned to
    void SetLayerMembership(size_t _layerMembership);

    /// Returns this action's action layer assignment
    size_t GetLayerMembership();

    /// Returns if this action has been used already
    bool Used();

    /// Marks this action as already used
    void SetUsed(bool _used);

    /// Returns the start state conditions of this action
    State& GetStartState();

    /// Returns the result of performing this action
    State& GetResultState();

    /// Returns the robots involved in this action
    virtual std::vector<Robot*> GetRobots();

    /// Returns a string describing this action
    virtual std::string PrintAction();

    /// Returns the cost of performing this action
    double GetCost();

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const Action& _action) const;

    bool operator!=(const Action& _action) const;

    ///@}

  protected:

    ///@name Helper Functions
    ///@{

    /// Performs a collision and terrain validity check for the robot at the
    /// location provided
    /// @param _robot Robot to perform the check with
    /// @param _location Location to perform the check at
    bool CanBeInLocation(Robot* _robot, const Boundary* _location);

    ///@}
    ///@name Internal State
    ///@{

    bool m_debug{true};               ///< Toggle debug messages.

    size_t m_layerMembership; ///< action layer that this belongs to

    bool m_used{false}; ///< Used to distinguish if the action is included

    State m_startState;  ///< Represents conditions of action
    State m_resultState; ///< Represents result of action

    MPLibrary* m_library{nullptr}; ///< library being used to perform mp checks

    bool m_manipulator; ///< indicates the type of robot team being used

    /// Base cost of 100 for actions such as starting and handing off task to be
    /// considered in the other plan cost
    double m_cost{100}; ///< Cost of performing this action
    ///@}
};

#endif
