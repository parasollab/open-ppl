#ifndef C_SPACE_CONSTRAINT_H_
#define C_SPACE_CONSTRAINT_H_

#include <cstddef>
#include <iostream>
#include <map>

#include "Constraint.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"

class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Bound the permitted DOF values for an object.
////////////////////////////////////////////////////////////////////////////////
class CSpaceConstraint : public Constraint {

  public:

    ///@name Construction
    ///@{

    /// Construct a constraint from an XML node.
    /// @param _r The robot to constrain.
    /// @param _node The node to parse.
    explicit CSpaceConstraint(Robot* const _r, XMLNode& _node);

    virtual ~CSpaceConstraint();

    virtual std::unique_ptr<Constraint> Clone() const override;

    ///@}
    ///@name Constraint Interface
    ///@{

    virtual const Boundary* GetBoundary() const override;

    virtual bool Satisfied(const Cfg& _c) const override;

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

    CSpaceBoundingBox m_bbx;   ///< The C-Space bounding box for this constraint.

    ///@}
    ///@name Debugging
    ///@{

    friend std::ostream& operator<<(std::ostream&, const CSpaceConstraint&);

    ///@}

};

#endif
