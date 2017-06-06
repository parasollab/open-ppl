#ifndef C_SPACE_BOUNDING_SPHERE_H_
#define C_SPACE_BOUNDING_SPHERE_H_

#include "AbstractBoundingSphere.h"


////////////////////////////////////////////////////////////////////////////////
/// An n-dimensional bounding sphere in c-space.
////////////////////////////////////////////////////////////////////////////////
class CSpaceBoundingSphere : public AbstractBoundingSphere {

  public:

    ///@name Construction
    ///@{

    CSpaceBoundingSphere(const size_t _n,
        const double _radius = std::numeric_limits<double>::max());

    CSpaceBoundingSphere(const std::vector<double>& _center,
        const double _radius = std::numeric_limits<double>::max());

    virtual ~CSpaceBoundingSphere() = default;

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
