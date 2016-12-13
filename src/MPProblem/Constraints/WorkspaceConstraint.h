#ifndef WORK_SPACE_CONSTRAINT_H_
#define WORK_SPACE_CONSTRAINT_H_

#include <cstddef>
#include <functional>
#include <vector>

#include "Constraint.h"

class FreeBody;
class XMLNode;
namespace mathtool {
  class Transformation;
}


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
class WorkspaceConstraint : public Constraint {

  public:

    ///@name LocalTypes
    ///@{

    typedef std::function<const bool(const mathtool::Transformation& _t)>
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

    virtual bool Satisfied(const Cfg& _c) const override;

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
    void SetPositionalBound(const size_t _i, const double _min,
        const double _max);

    /// @TODO Add interface for setting rotational constraints.

    ///@}

  protected:

    ///@name Internal State
    ///@{

    FreeBody* m_freeBody{nullptr}; ///< The part of the object to constrain.

    std::vector<ConstraintFunction> m_constraints; ///< The set of constraints.

    ///@}

};

#endif
