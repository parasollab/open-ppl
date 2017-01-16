#ifndef CSPACE_BOUNDING_BOX_H_
#define CSPACE_BOUNDING_BOX_H_

#include "AbstractBoundingBox.h"


////////////////////////////////////////////////////////////////////////////////
/// An n-dimensional bounding box in c-space.
////////////////////////////////////////////////////////////////////////////////
class CSpaceBoundingBox : public AbstractBoundingBox {

  public:

    ///@name Construction
    ///@{

    CSpaceBoundingBox(const size_t _n);

    CSpaceBoundingBox(const std::vector<double>& _center);

    virtual ~CSpaceBoundingBox() = default;

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

};

#endif
