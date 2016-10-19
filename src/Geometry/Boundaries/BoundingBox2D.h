#ifndef BOUNDING_BOX_2D_H_
#define BOUNDING_BOX_2D_H_

#include "Geometry/Boundaries/Boundary.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Geometry
/// A two-dimensional, axis-aligned bounding box.
////////////////////////////////////////////////////////////////////////////////
class BoundingBox2D :  public Boundary {

  public:

    ///@name Construction
    ///@{

    BoundingBox2D();

    BoundingBox2D(pair<double, double> _x, pair<double, double> _y);

    virtual ~BoundingBox2D() = default;

    ///@}
    ///@name Property Accessors
    ///@{

    virtual string Type() const noexcept override {return "Box2D";}

    virtual double GetMaxDist(double _r1 = 2.0, double _r2 = 0.5) const override;

    virtual pair<double, double> GetRange(size_t _i) const override;

    ///@}
    ///@name Sampling
    ///@{

    virtual Point3d GetRandomPoint() const override;

    ///@}
    ///@name Containment Testing
    ///@{

    virtual const bool InBoundary(const Vector3d& _p) const override;

    ///@}
    ///@name Clearance Testing
    ///@{

    virtual double GetClearance(const Vector3d& _p) const override;

    virtual int GetSideID(const vector<double>& _p) const override;

    virtual Vector3d GetClearancePoint(const Vector3d& _p) const override;

    ///@}
    ///@name Modifiers
    ///@{

    virtual void ApplyOffset(const Vector3d& _v) override;

    virtual void ResetBoundary(const vector<pair<double, double>>& _bbx,
        double _margin) override;

    ///@}
    ///@name I/O
    ///@{

    virtual void Read(istream& _is, CountingStreamBuffer& _cbs) override;

    virtual void Write(ostream& _os) const override;

    ///@}
    ///@name CGAL Representations
    ///@{

    virtual CGALPolyhedron CGAL() const override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    pair<double, double> m_bbx[2];

    ///@}

};

#endif
