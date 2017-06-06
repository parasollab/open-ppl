#ifndef WORKSPACE_BOUNDING_SPHERE_H_
#define WORKSPACE_BOUNDING_SPHERE_H_

#include "AbstractBoundingSphere.h"


////////////////////////////////////////////////////////////////////////////////
/// A 2 or 3 dimensional bounding sphere in workspace.
////////////////////////////////////////////////////////////////////////////////
class WorkspaceBoundingSphere : public AbstractBoundingSphere {

  public:

    ///@name Construction
    ///@{

    WorkspaceBoundingSphere(const size_t _n,
        const double _radius = std::numeric_limits<double>::max());

    WorkspaceBoundingSphere(const std::vector<double>& _center,
        const double _radius = std::numeric_limits<double>::max());

    WorkspaceBoundingSphere(const Vector3d& _center,
        const double _radius = std::numeric_limits<double>::max());

    virtual ~WorkspaceBoundingSphere() = default;

    virtual Boundary* Clone() const override;

    ///@}
    ///@name Property Accessors
    ///@{

    virtual Boundary::Space Type() const noexcept override;

    virtual std::string Name() const noexcept override;

    ///@}
    ///@name Containment Testing
    ///@{

    using Boundary::InBoundary;

    virtual bool InBoundary(const Cfg& _cfg) const override;

    ///@}
};

#endif
