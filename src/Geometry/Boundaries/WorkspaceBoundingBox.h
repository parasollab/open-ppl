#ifndef WORKSPACE_BOUNDING_BOX_H_
#define WORKSPACE_BOUNDING_BOX_H_

#include "AbstractBoundingBox.h"


////////////////////////////////////////////////////////////////////////////////
/// A two or three dimensional bounding box in workspace.
////////////////////////////////////////////////////////////////////////////////
class WorkspaceBoundingBox : public AbstractBoundingBox {

  public:

    ///@name Construction
    ///@{

    WorkspaceBoundingBox(const size_t _n);

    WorkspaceBoundingBox(const std::vector<double>& _center);

    virtual ~WorkspaceBoundingBox() = default;

    virtual Boundary* Clone() const override;

    ///@}
    ///@name Property Accessors
    ///@{

    virtual std::string Type() const noexcept override;

    ///@}
    ///@name Containment Testing
    ///@{

    virtual bool InBoundary(const Cfg& _c) const override;

    ///@}
    ///@name CGAL Representations
    ///@{

    using Boundary::CGALPolyhedron;

    virtual CGALPolyhedron CGAL() const override;

    ///@}

};

#endif
