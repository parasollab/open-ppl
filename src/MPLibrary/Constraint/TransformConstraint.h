#ifndef TRANSFORM_CONSTRAINT_H_
#define TRANSFORM_CONSTRAINT_H_

#include <functional>
#include <vector>

#include "Constraint.h"

class FreeBody;
class Transformation;

////////////////////////////////////////////////////////////////////////////////
/// Restrict the allowed world transformations for a specific component of a
/// movable object.
///
/// This constraint checks the world transformation of a specific FreeBody
/// against one or more constraint functions. Several primitive constraint
/// functions can be combined to produce more complex constraints.
////////////////////////////////////////////////////////////////////////////////
class TransformConstraint : public Constraint {

  public:

    ///@name LocalTypes
    ///@{

    std::function<const bool(const Transformation& _t)> ConstraintFunction;

    ///@}
    ///@name Construction
    ///@{

    TransformConstraint(const ActiveMultiBody* _m, const FreeBody* _f);
    virtual ~TransformConstraint() = default;

    ///@}
    ///@name Constraint Interface
    ///@{

    virtual bool operator()(const Cfg& _c) const;

    ///@}
    ///@name Creation Interface
    ///@{

    /// Add a generic constraint function.
    /// @param[in] _f The constraint function to add.
    void AddFunction(ConstraintFunction&& _f);

    ////////////////////////////////////////////////////////////////////////////
    /// Set a positional bound on the _i'th coordinate.
    /// @param _i The index of the coordinate to bound, in the range [0, 2].
    /// @param _min The minimum value of the transform for coordinate _i.
    /// @param _max The maximum value of the transform for coordinate _i.
    void SetPositionalBound(const size_t _i, double _min, double _max);

    /// @TODO: Flush out interface for setting constraints.

    ///@}

  protected:

    ///@name Internal State
    ///@{

    FreeBody* const m_freeBody; ///< The sub-piece of the object to constrain.

    std::vector<ConstraintFunction> m_constraints; ///< The set of constraints.

    ///@}

};

#endif
