#ifndef BOUNDINGSPHERE_H_
#define BOUNDINGSPHERE_H_

#include "Geometry/Boundaries/Boundary.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Geometry
/// A bounding sphere in 3d.
////////////////////////////////////////////////////////////////////////////////
class BoundingSphere : public Boundary {

  public:

    ///@name Construction
    ///@{

    BoundingSphere();

    BoundingSphere(const Vector3d& _center, double _radius);

    virtual ~BoundingSphere() = default;

    ///@}
    ///@name
    ///@{

    const double GetRadius() const {return m_radius;}

    ///@}
    ///@name Property Accessors
    ///@{

    virtual string Type() const noexcept override {return "Sphere";}

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

  private:

    ///@name Internal State
    ///@{

    double m_radius; ///< The bounding sphere radius.

    ///@}

#ifdef _PARALLEL
  public:
    void define_type(stapl::typer& _t) {
      _t.member(m_radius);
    }
#endif
};


#ifdef _PARALLEL
namespace stapl {
  template <typename Accessor>
    class proxy<BoundingSphere, Accessor>
    : public Accessor {
      private:
        friend class proxy_core_access;
        typedef BoundingSphere target_t;

      public:
        explicit proxy(Accessor const& acc) : Accessor(acc) { }
        operator target_t() const { return Accessor::read(); }
        proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
        proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
        Point3d GetRandomPoint() const { return Accessor::const_invoke(&target_t::GetRandomPoint);}
    };
}
#endif

#endif
