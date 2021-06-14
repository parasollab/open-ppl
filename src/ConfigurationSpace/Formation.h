#ifndef PPL_FORMATION_H_
#define PPL_FORMATION_H_

#include "Cfg.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "MPProblem/Robot/Robot.h"

class Formation {
  public:

    ///@name Local Types
    ///@{

    struct FormationConstraint {
      Robot* dependentRobot;
      Body* dependentBody;
      Robot* referenceRobot;
      Body* referenceBody;
      Transformation transformation;
    };

    ///@}
    ///@name Construction
    ///@{

    Formation(std::vector<Robot*> _robots, Robot* _leader, 
              std::unordered_map<Robot*,FormationConstraint> _constraintMap);

    ~Formation();

    ///@}
    ///@name Interface 
    ///@{

    /// Get a random configuration of the formation.
    std::vector<Cfg> RandomFormationCfg(const Boundary* _b);

    /// Find the c-space increment that moves from a start to a goal in a fixed
    /// number of steps.
    /// @param _start The start configurations of the robots in the formation.
    /// @param _goal The goal configurations of the robots in the formation.
    /// @param _nTicks The number of steps to take.
    std::vector<Cfg> FindIncrement(std::vector<Cfg> _start, std::vector<Cfg> _goal, 
                                   const size_t _nTicks);

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    /// Use robots and constraints to build formation multibody.
    void BuildMultiBody();

    void AddBody(Body* _body, bool _base = false);

    void AddConnection(Body* _first, Body* _second, Connection* _connection);

    /// Construct the formation cspace boundary from multibody.
    void InitializePlanningSpace();

    /// Convert a set of robot cfgs into the formation degrees-of-freedom.
    ///@param _cfgs The configurations of the robots in the formation.
    std::vector<double> ConvertToFormationDOF(std::vector<Cfg> _cfgs);

    /// Convert formation degrees-of-freedom into a set of individual robot
    /// cfgs.
    ///@param _dofs The formation dofs to convert.
    std::vector<Cfg> ConvertToIndividualCfgs(std::vector<double> _dofs);

    ///@}
    ///@name Internal State
    ///@{

    /// Set of robots in the formation.
    std::vector<Robot*> m_robots;

    /// The lead robot in the formation - used to construct multibody.
    Robot* m_leader;

    /// The multibody representing the formation.
    std::unique_ptr<MultiBody> m_multibody;

    /// The degrees of freedom of the formation.
    std::vector<double> m_dofs;

    // The configuration space of the formation.
    std::unique_ptr<CSpaceBoundingBox> m_cspace;

    /// The map of constraints defining the formation
    std::unordered_map<Robot*,FormationConstraint> m_constraintMap;

    /// The map of individual robot connections to formation connnections.
    /// The boolean value represents if the direction is the same or if it
    /// has been inverted (same = true, inverted = false);
    std::unordered_map<Connection*,std::pair<bool,size_t> m_jointMap;

    /// Map of individual robot bodies to formation bodies.
    std::unordered_map<Body*,size_t> m_bodyMap;

    ///@}
};

#endif
