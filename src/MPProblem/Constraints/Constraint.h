#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

class Boundary;
class Cfg;
class Robot;


////////////////////////////////////////////////////////////////////////////////
/// An abstract base class representing the required interface for a constraint
/// on the state of a robot. The Constraint interface requires plain jane Cfg's
/// to ensure that they are usable by reactive agents.
////////////////////////////////////////////////////////////////////////////////
class Constraint {

  public:

    ///@name Construction
    ///@{

    /// Create a constraint for a robot.
    /// @param _r The robot to constrain.
    Constraint(Robot* const _r);

    virtual ~Constraint() = default;

    ///@}
    ///@name Constraint Interface
    ///@{

    /// Get a sampling boundary that describes the subset of CSpace allowed by
    /// this constraint.
    virtual const Boundary* GetBoundary() const = 0;

    /// Determine whether a given configuration of the robot satisfies this
    /// constraint.
    /// @param _c The configuration to check.
    /// @return   True if _c satisfies this constraint.
    virtual bool Satisfied(const Cfg& _c) const = 0;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    Robot* const m_robot; ///< The subject of this constraint.

    ///@}

};

#endif
