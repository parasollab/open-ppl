#ifndef DOF_CONSTRAINT_H_
#define DOF_CONSTRAINT_H_

#include <map>

#include "Constraint.h"
#include "Geometry/Boundaries/Range.h"

////////////////////////////////////////////////////////////////////////////////
/// Bound the permitted DOF values for an object.
////////////////////////////////////////////////////////////////////////////////
class DOFConstraint : public Constraint {

  ///@name Internal State
  ///@{

  std::map<size_t, Range<double>> m_limits; ///< The restricted DOF ranges.

  ///@}

  public:

    ///@name Construction
    ///@{

    DOFConstraint(const ActiveMultiBody*);
    virtual ~DOFConstraint() = default;

    ///@}
    ///@name Constraint Interface
    ///@{

    virtual const bool operator()(const Cfg& _c) const;

    ///@}
    ///@name Creation Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Add a limit for a specific DOF value, overwriting any previously set
    /// limits.
    /// @param _dof The DOF index to limit.
    /// @param _min The minimum allowed value for the limited DOF.
    /// @param _max The maximum allowed value for the limited DOF.
    void SetLimit(const size_t _dof, const double _min, const double _max);

    ///@}

};

#endif
