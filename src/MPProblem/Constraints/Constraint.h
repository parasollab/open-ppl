#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

class ActiveMultiBody;
class Boundary;
class Cfg;


////////////////////////////////////////////////////////////////////////////////
/// An abstract base class representing the required interface for a constraint
/// on the state of a movable object. The Constraint interface requires plain
/// jane Cfg's to ensure that they are usable by reactive agents.
////////////////////////////////////////////////////////////////////////////////
class Constraint {

  public:

    ///@name Construction
    ///@{

    /// Create a constraint for a movable object.
    /// @param _m The movable object to constrain.
    Constraint(ActiveMultiBody* const _m);

    virtual ~Constraint() = default;

    ///@}
    ///@name Constraint Interface
    ///@{

    /// Get a sampling boundary that describes the subset of CSpace allowed by
    /// this constraint.
    virtual Boundary* GetBoundary() const = 0;

    /// Determine whether a given configuration of the object satisfies this
    /// constraint.
    /// @param _c The configuration to check.
    /// @return   True if _c satisfies this constraint.
    virtual bool Satisfied(const Cfg& _c) const = 0;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    ActiveMultiBody* const m_multibody; ///< The subject of this constraint.

    ///@}

};

#endif
