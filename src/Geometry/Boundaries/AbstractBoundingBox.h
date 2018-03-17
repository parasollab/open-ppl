#ifndef ABSTRACT_BOUNDING_BOX_H_
#define ABSTRACT_BOUNDING_BOX_H_

#include "Boundary.h"
#include "Geometry/Shapes/NBox.h"


////////////////////////////////////////////////////////////////////////////////
/// An abstract axis-aligned N-dimensional bounding box.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class AbstractBoundingBox :  public Boundary, public NBox {

  public:

    ///@name Construction
    ///@{

    /// Construct an infinite bounding box in n dimensions.
    /// @param[in] _n The number of dimensions.
    explicit AbstractBoundingBox(const size_t _n);

    /// Construct an infinite bounding box in n dimensions.
    /// @param[in] _n The number of dimensions.
    explicit AbstractBoundingBox(const std::vector<double>& _center);

    /// Construct a bounding box from an XML node.
    /// @param _node The XML node to parse.
    AbstractBoundingBox(XMLNode& _node);

    virtual ~AbstractBoundingBox() noexcept;

    ///@}
    ///@name Property Accesors
    ///@}

    virtual size_t GetDimension() const noexcept override;

    virtual double GetMaxDist(const double _r1 = 2., const double _r2 = .5)
        const override;

    virtual Range<double> GetRange(const size_t _i) const override;

    virtual const std::vector<double>& GetCenter() const noexcept override;

    ///@}
    ///@name Sampling
    ///@{

    virtual std::vector<double> GetRandomPoint() const override;

    virtual void PushInside(std::vector<double>& _sample) const noexcept override;

    ///@}
    ///@name Scaling
    ///@{

    virtual void ScalePoint(std::vector<double>& _point) const noexcept override;

    virtual void UnscalePoint(std::vector<double>& _point) const noexcept
        override;

    ///@}
    ///@name Containment Testing
    ///@{

    using Boundary::InBoundary;

    virtual bool InBoundary(const std::vector<double>& _p) const override;

    ///@}
    ///@name Clearance Testing
    ///@{

    virtual double GetClearance(const Vector3d& _p) const override;

    virtual Vector3d GetClearancePoint(const Vector3d& _p) const override;

    /// Get an integer describing which three-dimensional side a given point _p
    /// is nearest.
    /// @param _p The point of interest.
    /// @return An integer indicating the nearest side.
    int GetSideID(const vector<double>& _p) const;

    ///@}
    ///@name Modifiers
    ///@{

    virtual void SetCenter(const std::vector<double>& _c) noexcept override;

    virtual void ApplyOffset(const Vector3d& _v) override;

    virtual void ResetBoundary(const vector<pair<double, double>>& _bbx,
        const double _margin) override;

    ///@}
    ///@name I/O
    ///@{

    virtual void Read(istream& _is, CountingStreamBuffer& _cbs) override;

    virtual void Write(ostream& _os) const override;

    ///@}

};

/*----------------------------------- I/O ------------------------------------*/

std::ostream& operator<<(std::ostream& _os, const AbstractBoundingBox& _b);

/// @TODO Move impl from environment to here.
//istream& operator>>(istream& _is, const Boundary& _b);

/*----------------------------------------------------------------------------*/

#endif
