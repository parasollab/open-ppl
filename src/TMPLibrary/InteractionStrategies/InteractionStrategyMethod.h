#ifndef _PPL_INTERACTION_STRATEGY_METHOD_H_
#define _PPL_INTERACTION_STRATEGY_METHOD_H_

#include "TMPLibrary/ActionSpace/Condition.h"
#include "TMPLibrary/TMPBaseObject.h"
//for dependent sampling
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"

class FormationCondition;
class Interaction;
class MotionCondition;

class InteractionStrategyMethod : public TMPBaseObject {

  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;
    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;
    typedef PathType<MPTraits<Cfg,DefaultWeight<Cfg>>>       Path;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>>      GroupPathType;
    typedef GroupLocalPlan<Cfg>                              GroupWeightType;
    typedef std::map<Robot*, const Boundary*> BoundaryMap;


    ///@}
    ///@name Construction
    ///@{

    InteractionStrategyMethod() = default;

    InteractionStrategyMethod(XMLNode& _node);

    virtual ~InteractionStrategyMethod();

    ///@}
    ///@name Interface
    ///@{

    ///@param _interaction The interaction to plan.
    ///@param _start Input as the start state. Modified to reflect the
    ///              output state.
    ///@return bool representing if successful or not.
    virtual bool operator()(Interaction* _interaction, State& _start);

    ///@}

  protected:

    ///@name Helper Functions
    ///@{

    /// Assign interaction roles to the robots in the input state from the input set 
    /// of conditions.
    /// @param _state The state to extract the robots and connect to the conditions.
    /// @param _conditions The set of conditions to use to assign roles.
    virtual void AssignRoles(const State& _state, const std::vector<std::string>& _conditions);

    /// For each input condition, assign a group and individual robots to the roles.
    /// @param _state The state to use to extract the roles.
    /// @param _conditions The set of conditions to assign roles from.
    /// @param _usedGroups The set of robot groups that have already been claimed.
    void AssignRolesFromConditions(const State& _state,
                          const std::vector<Condition*>& _conditions,
                          std::unordered_set<RobotGroup*>& _usedGroups);

    /// Use the pre-conditions in the interaction to set the local 
    /// boundary for planning the interaction.
    /// @param _interaction The interaction to extract the boundary for.
    /// @param _state State to center interaction boundary on.
    void SetInteractionBoundary(Interaction* _interaction, const State& _state);

    /// Generate a set of CSpace constraints corresponding to the input state.
    /// @param _state The state to convert into CSpace constraints.
    std::unordered_map<Robot*,Constraint*> GenerateConstraints(const State& _state);

    /// Generate a set of motion constraints from the listed conditions.
    /// @param _conditions The conditions to generate the constraints from.
    /// @param _groups The robot groups the constraints will form.
    std::unordered_map<Robot*,Constraint*> GenerateConstraints(
                                           const std::vector<std::string>& _conditions,
                                           const std::vector<RobotGroup*>& _groups,
                                           const State& _state,
                                           const std::set<Robot*>& _staticRobots);

    //This is used for the dependentpaths sampling method. Highly stylized after above func
    std::unordered_map<Robot*,Constraint*> GenerateConstraintsDependent(
                                           const std::vector<std::string>& _conditions,
                                           const std::vector<RobotGroup*>& _groups,
                                           const State& _state,
                                           const std::set<Robot*>& _staticRobots);


    /// Sample a set of motion constraints for each group in the interaction boundary.
    /// @param _groups The robot groups to find constraints for.
    std::unordered_map<Robot*,Constraint*> SampleMotionConstraints(
                                           const std::vector<std::string>& _conditions,
                                           const std::vector<Robot*> _robots,
                                           const State& _state,
                                           const std::set<Robot*>& _staticRobots);

    //Sampling for the dependent paths constraint
    std::unordered_map<Robot*,Constraint*> SampleMotionConstraintsDependent(
                                           const std::vector<std::string>& _conditions,
                                           const std::vector<Robot*> _robots,
                                           const State& _state,
                                           const std::set<Robot*>& _staticRobots);


    void SetActiveFormations(std::vector<std::string> _conditions, MPSolution* _solution);


    // TODO::Temporary - delete after IRving's implementation is checked in.
    std::vector<Path*> DecouplePath(MPSolution* _solution, GroupPathType* _groupPath);

    /// Compare a formation condition to a motion condition for role equivalence.
    /// @param _f The formation condition to consider.
    /// @param _m The motion condition to consider.
    /// @return If the two are conditioned on the same roles.
    bool CompareConditionRoleSets(FormationCondition* _f,
                                  MotionCondition* _m);

    std::set<Robot*> GetStaticRobots(const std::vector<std::string>& _conditions);

    std::set<Robot*> GetRobots(const std::vector<std::string>& _conditions);

    void ConfigureStaticRobots(const std::set<Robot*>& _staticRobots, const State& _state); 

    void ResetStaticRobots(const std::set<Robot*>& _staticRobots);

    void MoveStateToLocalSolution(Interaction* _interaction, State& _state);

    void SampleStartState(Interaction* _interaction, State& _state);

    ///@}
    ///@name Internal State
    ///@{

    std::unordered_map<std::string,Robot*> m_roleMap;

    std::unique_ptr<Boundary> m_boundary; ///< Boundary to plan interaction within.

    std::string m_sgLabel; ///< State graph label

    std::string m_smLabel; ///< Sampler method label

    std::string m_vcLabel; ///< Validity checker label

    size_t m_numNodes; ///< Number of samples for sampling constraint cfgs.
    size_t m_maxAttempts; ///< Number of attempts for sampling constraint cfgs.

    ///@}

};
#endif
