#ifndef WORK_SPACE_CONSTRAINT_H_
#define WORK_SPACE_CONSTRAINT_H_

#include <functional>
#include <vector>

#include "Constraint.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Utilities/RuntimeUtils.h"

#include "Transformation.h"

#include "Geometry/Boundaries/BoundingSphere.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"


////////////////////////////////////////////////////////////////////////////////
/// Restrict the allowed world transformations for a specific component of a
/// movable object.
///
/// This constraint checks the world transformation of a specific FreeBody
/// against one or more constraint functions. All constraint functions must be
/// satisfied for the constraint to be satisfied. Several primitive constraint
/// functions can be combined to produce more complex constraints.
///
/// @TODO Add XML parsing.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class WorkspaceConstraint : public ConstraintType<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name LocalTypes
    ///@{

    typedef std::function<const bool(const Transformation& _t)>
        ConstraintFunction;

    ///@}
    ///@name Construction
    ///@{

    WorkspaceConstraint(ActiveMultiBody* const, XMLNode&);

    virtual ~WorkspaceConstraint() = default;

    ///@}
    ///@name Constraint Interface
    ///@{

    virtual Boundary* GetBoundary() const override;

    virtual bool operator()(const CfgType& _c) const override;

    ///@}
    ///@name Creation Interface
    ///@{

    /// Add a generic constraint function.
    /// @param[in] _f The constraint function to add.
    void AddFunction(ConstraintFunction&& _f);

    /// Set a positional bound on the _i'th coordinate.
    /// @param _i The index of the coordinate to bound, in the range [0, 2].
    /// @param _min The minimum value of the transform for coordinate _i.
    /// @param _max The maximum value of the transform for coordinate _i.
    void SetPositionalBound(const size_t _i, double _min, double _max);

    /// @TODO Add interface for setting rotational constraints.

    ///@}

  protected:

    ///@name Internal State
    ///@{

    using ConstraintType<MPTraits>::m_multibody;

    FreeBody* m_freeBody{nullptr}; ///< The part of the object to constrain.

    std::vector<ConstraintFunction> m_constraints; ///< The set of constraints.

    ///@}

};

/*--------------------------------- Construction -----------------------------*/

template <typename MPTraits>
WorkspaceConstraint<MPTraits>::
WorkspaceConstraint(ActiveMultiBody* const _m, XMLNode& _node)
  : ConstraintType<MPTraits>(_m) {
  // Parse the label for the part we need to constrain.
  std::string partLabel = _node.Read("part", true, "", "The label for the robot "
      "part to constrain.");

  // Find the part.
  for(size_t i = 0; i < m_multibody->NumFreeBody(); ++i) {
    auto body = m_multibody->GetFreeBody(i).get();
    if(body->Label() == partLabel) {
      m_freeBody = body;
      break;
    }
  }

  // Make sure we found it.
  if(!m_freeBody)
    throw ParseException(WHERE, "Could not find robot part with label '" +
        partLabel + "'.");

  // Parse the constraint data.
}

/*------------------------------ Constraint Interface ------------------------*/

template <typename MPTraits>
Boundary*
WorkspaceConstraint<MPTraits>::
GetBoundary() const {
  /// @TODO
  throw RunTimeException(WHERE, "Not yet implemented");
  return nullptr;
}


template <typename MPTraits>
bool
WorkspaceConstraint<MPTraits>::
operator()(const CfgType& _c) const {
  // Configure the object at _c and get the transformation for the constrained
  // free body.
  m_multibody->Configure(_c);
  const auto& currentTransform = m_freeBody->GetWorldTransformation();

  // Check the current transform against each constraint function. If any fail,
  // then the constraint isn't satisfied.
  for(auto& constraintFunction : m_constraints)
    if(!constraintFunction(currentTransform))
      return false;
  return true;
}

/*------------------------------- Creation Interface -------------------------*/

template <typename MPTraits>
void
WorkspaceConstraint<MPTraits>::
AddFunction(ConstraintFunction&& _f) {
  m_constraints.push_back(_f);
}


template <typename MPTraits>
void
WorkspaceConstraint<MPTraits>::
SetPositionalBound(const size_t _i, const double _min, const double _max) {
  AssertMsg(_i < m_multibody->PosDOF(), WHERE + "\nCan't set a positional bound "
      "on index " + std::to_string(_i) + " for a multibody that has only " +
      std::to_string(m_multibody->PosDOF()) + " positional DOFs.");

  m_constraints.push_back(
      [_i, _min, _max](const Transformation& _t) {
        const auto& position = _t.translation();
        return Range<double>(_min, _max).Inside(position[_i]);
      });
}

/*----------------------------------------------------------------------------*/
#endif
