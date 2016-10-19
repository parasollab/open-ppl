#ifndef TETRAHEDRAL_BOUNDARY_H_
#define TETRAHEDRAL_BOUNDARY_H_

#include "Boundary.h"

////////////////////////////////////////////////////////////////////////////////
/// A tetrahedral bounding region.
/// @ingroup Environment
////////////////////////////////////////////////////////////////////////////////
class TetrahedralBoundary : public Boundary {

  public:

    ///@name Construction
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Construct a TetrahedralBoundary from four points.
    /// @param _pts The points to use.
    /// @param _check Check that the points are correct?
    ///
    /// If no check is used, the first three points must form an outward-facing
    /// facet as shown.
    ///
    ///     1
    ///    /|\
    ///   / | \
    ///  2--|--3
    ///   \ | /
    ///    \|/
    ///     0
    TetrahedralBoundary(const array<Point3d, 4>& _pts, const bool _check = true);
    TetrahedralBoundary(const vector<Point3d>& _pts, const bool _check = true);

    virtual ~TetrahedralBoundary() = default;

    ///@}
    ///@name Property Accessors
    ///@{

    virtual string Type() const override {return "Tetrahedral";}

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

    ///@}
    ///@name Clearance Testing
    ///@{

    virtual double GetClearance(const Vector3d& _p) const override;

    virtual int GetSideID(const vector<double>& _p) const override;

    virtual Vector3d GetClearancePoint(const Vector3d& _p) const override;

    ///@}
    ///@name Modifiers
    ///@}

    virtual void ApplyOffset(const Vector3d& _v) override;

    virtual void ResetBoundary(vector<pair<double, double>>& _bbx,
        double _margin) override;

    ///@}
    ///@name I/O
    ///@{

    virtual void Read(istream& _is, CountingStreamBuffer& _cbs) override;

    virtual void Write(ostream& _os) const override;

    ///@}
    ///@name CGAL Representation
    ///@{

    virtual CGALPolyhedron CGAL() const override;

    ///@}

    protected:

      ///@name Helpers
      ///@{

      //////////////////////////////////////////////////////////////////////////
      /// Check that the points are in the correct order and fix if necessary.
      void FixPoints();

      array<Vector3d, 6> ComputeEdges() const;

      /// Compute the normals. The first three are for faces touched by point 0.
      array<Vector3d, 4> ComputeNormals() const;

      ///@}
      ///@name Internal State
      ///@{

      array<Point3d, 4> m_points;   ///< The vertices of the tetrahedron.
      array<Vector3d, 4> m_normals; ///< The normals of the tetrahedron.

      ///@}
};

#endif
