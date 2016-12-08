#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

class ActiveMultiBody;
class Boundary;


////////////////////////////////////////////////////////////////////////////////
/// An abstract base class representing the required interface for a constraint
/// on the state of a movable object.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ConstraintType {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    /// Create a constraint for a movable object.
    /// @param _m The movable object to constrain.
    ConstraintType(ActiveMultiBody* const _m);

    virtual ~ConstraintType() = default;

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
    virtual bool operator()(const CfgType& _c) const = 0;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    ActiveMultiBody* const m_multibody; ///< The subject of this constraint.

    ///@}

};

/*--------------------------------- Construction -----------------------------*/

template <typename MPTraits>
inline
ConstraintType<MPTraits>::
ConstraintType(ActiveMultiBody* const _m) : m_multibody(_m) { }

/*----------------------------------------------------------------------------*/

#endif
