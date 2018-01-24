#ifndef TETRAHEDRAL_BOUNDARY_H_
#define TETRAHEDRAL_BOUNDARY_H_

#include "Boundary.h"

////////////////////////////////////////////////////////////////////////////////
/// A tetrahedral bounding region in workspace.
/// @ingroup Environment
////////////////////////////////////////////////////////////////////////////////
class TetrahedralBoundary : public Boundary {

  public:

    ///@name Construction
    ///@{

    /// Construct a TetrahedralBoundary from four points.
    /// @param _pts The points to use.
    /// @param _check Check that the points are correct?
    ///
    /// If no check is used, the first three points must form an outward-facing
    /// facet using the right-hand rule as shown.
    ///
    ///     1
    ///    /|\
    ///   / | \
    ///  2--|--3
    ///   \ | /
    ///    \|/
    ///     0
    explicit TetrahedralBoundary(const std::array<Point3d, 4>& _pts,
        const bool _check = true);

    explicit TetrahedralBoundary(const std::vector<Point3d>& _pts,
        const bool _check = true);

    virtual std::unique_ptr<Boundary> Clone() const override;

    virtual ~TetrahedralBoundary() noexcept;

    ///@}
    ///@name Property Accessors
    ///@{

    virtual Boundary::Space Type() const noexcept override;

    virtual std::string Name() const noexcept override;

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
    ///@name Containment Testing
    ///@{

    virtual bool InBoundary(const Vector3d& _p) const override;

    virtual bool InBoundary(const std::vector<double>& _v) const override;

    virtual bool InBoundary(const Cfg& _c) const override;

    ///@}
    ///@name Clearance Testing
    ///@{

    virtual double GetClearance(const Vector3d& _p) const override;

    virtual Vector3d GetClearancePoint(const Vector3d& _p) const override;

    ///@}
    ///@name Modifiers
    ///@}

    virtual void SetCenter(const std::vector<double>& _c) noexcept override;

    virtual void ApplyOffset(const Vector3d& _v) override;

    virtual void ResetBoundary(const std::vector<pair<double, double>>& _bbx,
        const double _margin) override;

    ///@}
    ///@name I/O
    ///@{

    virtual void Read(std::istream& _is, CountingStreamBuffer& _cbs) override;

    virtual void Write(std::ostream& _os) const override;

    ///@}
    ///@name CGAL Representation
    ///@{

    virtual CGALPolyhedron CGAL() const override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Check that the points are in the correct order and fix if necessary.
    void FixPoints() noexcept;

    /// Compute the edges. The resulting vectors point from lower-index points
    /// higher-index points.
    std::array<Vector3d, 6> ComputeEdges() const;

    /// Compute the normals. The first three are for faces touched by point 0.
    std::array<Vector3d, 4> ComputeNormals() const;

    /// Compute the center.
    std::vector<double> ComputeCenter() const;

    ///@}
    ///@name Internal State
    ///@{

    std::vector<double> m_center;      ///< The center point of the tetrahedron.
    std::array<Point3d, 4> m_points;   ///< The vertices of the tetrahedron.
    std::array<Vector3d, 4> m_normals; ///< The normals of the tetrahedron.

    ///@}
};

#endif
