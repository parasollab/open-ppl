#ifndef C_SPACE_CONSTRAINT_H_
#define C_SPACE_CONSTRAINT_H_

#include <limits>
#include <map>

#include "Constraint.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Range.h"
#include "Utilities/RuntimeUtils.h"

#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"


////////////////////////////////////////////////////////////////////////////////
/// Bound the permitted DOF values for an object.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CSpaceConstraint : public ConstraintType<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    CSpaceConstraint(ActiveMultiBody* const, XMLNode&);

    virtual ~CSpaceConstraint() = default;

    ///@}
    ///@name Constraint Interface
    ///@{

    virtual Boundary* GetBoundary() const override;

    virtual bool operator()(const CfgType& _c) const override;

    ///@}
    ///@name Creation Interface
    ///@{

    /// Add a limit for a specific DOF value, overwriting any previously set
    /// limits.
    /// @param _dof The DOF index to limit.
    /// @param _min The minimum allowed value for the limited DOF.
    /// @param _max The maximum allowed value for the limited DOF.
    void SetLimit(const size_t _dof, const double _min, const double _max);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    using ConstraintType<MPTraits>::m_multibody;

    /// @TODO use n-dim. aabb here.
    std::map<size_t, Range<double>> m_limits; ///< The restricted DOF ranges.

    ///@}

};

/*--------------------------------- Construction -----------------------------*/

template <typename MPTraits>
CSpaceConstraint<MPTraits>::
CSpaceConstraint(ActiveMultiBody* const _m, XMLNode& _node)
  : ConstraintType<MPTraits>(_m) {
  for(auto& child : _node) {
    if(_node.Name() == "Limit") {
      const size_t dof = _node.Read("dof", true, size_t(0), size_t(0),
          size_t(_m->DOF()), "The DOF index to restrict");
      const double min = _node.Read("min", true, 0.,
          std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
          "Min value");
      const double max = _node.Read("max", true, 0.,
          std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
          "Max value");
      SetLimit(dof, min, max);
    }
  }
}

/*------------------------------ Constraint Interface ------------------------*/

template <typename MPTraits>
Boundary*
CSpaceConstraint<MPTraits>::
GetBoundary() const {
  /// @TODO
  throw RunTimeException(WHERE, "Not yet implemented.");
  return nullptr;
}


template <typename MPTraits>
bool
CSpaceConstraint<MPTraits>::
operator()(const CfgType& _c) const {
  const auto& cfg = _c.GetData();

  // If any of the limits are violated, the constraint is not satisfied.
  for(const auto& constraint : m_limits) {
    const auto& index = constraint.first;
    const auto& range = constraint.second;

    if(!range.Inside(cfg[index]))
      return false;
  }
  return true;
}

/*------------------------------- Creation Interface -------------------------*/

template <typename MPTraits>
void
CSpaceConstraint<MPTraits>::
SetLimit(const size_t _dof, const double _min, const double _max) {
  AssertMsg(_dof < m_multibody->DOF(), WHERE + "\nRequested limit on DOF "
      "index " + std::to_string(_dof) + ", but multibody has only " +
      std::to_string(m_multibody->DOF()) + " DOFs.");

  m_limits[_dof] = Range<double>(_min, _max);
}

/*----------------------------------------------------------------------------*/

#endif
