#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include "Boundary.h"
#include "Range.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Geometry
/// An axis-aligned N-dimensional bounding box.
////////////////////////////////////////////////////////////////////////////////
class BoundingBox :  public Boundary {

  public:

    ///@name Construction
    ///@{

    /// Construct an infinite bounding box.
    /// @param[in] _dimension The number of dimensions.
    BoundingBox(const size_t _dimension = 3);

    /// Construct a three-dimensional bounding box.
    /// @param[in] _x The range in the X dimension.
    /// @param[in] _y The range in the Y dimension.
    /// @param[in] _z The range in the Z dimension.
    BoundingBox(pair<double, double> _x,
        pair<double, double> _y,
        pair<double, double> _z);

    virtual ~BoundingBox() = default;

    ///@}
    ///@name Property Accesors
    ///@}

    virtual string Type() const noexcept override;

    virtual double GetMaxDist(double _r1 = 2.0, double _r2 = 0.5) const override;

    virtual pair<double, double> GetRange(size_t _i) const override;

    ///@}
    ///@name Sampling
    ///@{

    virtual Point3d GetRandomPoint() const override;

    ///@}
    ///@name Containment Testing
    ///@{

    virtual bool InBoundary(const Vector3d& _p) const override;
    virtual bool InCSpace(const Cfg& _c) const override;

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

    virtual void SetRange(const size_t _i, const double _min, const double _max);

    virtual void SetRange(const size_t _i, Range<double>&& _r);

    ///@}
    ///@name I/O
    ///@{

    virtual void Read(istream& _is, CountingStreamBuffer& _cbs) override;

    virtual void Write(ostream& _os) const override;

    ///@}
    ///@name CGAL Representations
    ///@{

    /// A CGAL representation of the workspace portion of this boundary.
    virtual CGALPolyhedron CGAL() const override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    void UpdateCenter();

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<Range<double>> m_bbx;

    ///@}

#ifdef _PARALLEL
  public:
    void define_type(stapl::typer &_t)
    {
      _t.member(m_bbx);
    }
#endif

};

#ifdef _PARALLEL
namespace stapl {

  //////////////////////////////////////////////////////////////////////////////
  /// @TODO A parallel MP researcher needs to document this class.
  //////////////////////////////////////////////////////////////////////////////
  template <typename Accessor>
  class proxy<BoundingBox, Accessor> : public Accessor {

    private:

      friend class proxy_core_access;
      typedef BoundingBox target_t;

    public:

      explicit proxy(Accessor const& acc) : Accessor(acc) { }

      operator target_t() const {return Accessor::read();}

      proxy const& operator=(proxy const& rhs) {
        Accessor::write(rhs);
        return *this;
      }

      proxy const& operator=(target_t const& rhs) {
        Accessor::write(rhs);
        return *this;
      }

      Point3d GetRandomPoint() const {
        return Accessor::const_invoke(&target_t::GetRandomPoint);
      }
  };

}
#endif

#endif
