#ifndef WORK_SPACE_CONSTRAINT_H_
#define WORK_SPACE_CONSTRAINT_H_

#include <cstddef>
#include <functional>
#include <vector>

#include "Constraint.h"

class Body;
class XMLNode;
namespace mathtool {
  class Transformation;
}


////////////////////////////////////////////////////////////////////////////////
/// Restrict the allowed world transformations for a specific component of a
/// movable object.
///
/// This constraint checks the world transformation of a specific Body
/// against one or more constraint functions. All constraint functions must be
/// satisfied for the constraint to be satisfied. Several primitive constraint
/// functions can be combined to produce more complex constraints.
///
/// @TODO Add XML parsing.
/// @TODO Rename this object to 'TransformConstraint' since that's what it does.
///       Create a separate WorkspaceConstraint that constrains either some or
///       all of the robot to avoid specific areas/planes in workspace.
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

    /// Construct a constraint from an XML node.
    /// @param _r The robot to constrain.
    /// @param _node The node to parse.
    explicit WorkspaceConstraint(Robot* const _r, XMLNode& _node);

    virtual ~WorkspaceConstraint();

    virtual std::unique_ptr<Constraint> Clone() const override;

    ///@}
    ///@name Constraint Interface
    ///@{

    virtual void SetRobot(Robot* const _r) override;

    virtual const Boundary* GetBoundary() const override;

    virtual bool Satisfied(const Cfg& _c) const override;

    ///@}
    ///@name Creation Interface
    ///@{

    /// Add a generic constraint function.
    /// @param _f The constraint function to add.
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

    ///@name Helpers
    ///@{

    /// Set the constrained body pointer by part label.
    /// @param _label The label of the part to constrain.
    void SetPart(const std::string& _label);

    ///@}
    ///@name Internal State
    ///@{

    Body* m_body{nullptr}; ///< The part of the object to constrain.

    std::vector<ConstraintFunction> m_constraints; ///< The set of constraints.

    ///@}

};

#endif
