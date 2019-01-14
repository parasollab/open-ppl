#ifndef PMPL_BOUNDARY_TEST_H_
#define PMPL_BOUNDARY_TEST_H_

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "nonstd.h"

#include "Geometry/Boundaries/TetrahedralBoundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphericalShell.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"


////////////////////////////////////////////////////////////////////////////////
/// Scratch space for testing random stuff in PMPL.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BoundaryTest : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    BoundaryTest();

    BoundaryTest(XMLNode& _node);

    virtual ~BoundaryTest() = default;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    void TestBoundary(Boundary& _b) const;

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
BoundaryTest<MPTraits>::
BoundaryTest() : MPStrategyMethod<MPTraits>() {
  this->SetName("BoundaryTest");
}


template <typename MPTraits>
BoundaryTest<MPTraits>::
BoundaryTest(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("BoundaryTest");
}

/*------------------------------ Interface -----------------------------------*/

template <typename MPTraits>
void
BoundaryTest<MPTraits>::
Iterate() {
  MethodTimer mt(this->GetStatClass(), "Test Iteration");

  auto env = this->GetEnvironment();

  WorkspaceBoundingSphericalShell shell(env->GetBoundary()->GetDimension(), 2, 1);

  WorkspaceBoundingSphere sphere(env->GetBoundary()->GetDimension(), 2);

  WorkspaceBoundingBox box(env->GetBoundary()->GetDimension());
  box.SetRange(0, Range<double>(-1,1));
  box.SetRange(1, Range<double>(-2,2));
  box.SetRange(2, Range<double>(-3,3));

  // GetClearancePoint needs to be implemented before we can test the
  // tetrahedral boundary.
  //TetrahedralBoundary tetra(std::array<Point3d, 4>{Point3d( 1,  1,  1),
  //                                                 Point3d(-1, -1, -1),
  //                                                 Point3d(-1,  1, -1),
  //                                                 Point3d( 1, -1, -1)});

  TestBoundary(shell);
  TestBoundary(sphere);
  TestBoundary(box);
  //TestBoundary(tetra);
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
void
BoundaryTest<MPTraits>::
TestBoundary(Boundary& _b) const {
  auto env = this->GetEnvironment();

  // Translate the shell to some random location.
  const std::vector<double> c = env->GetBoundary()->GetRandomPoint();
  const Vector3d center(c[0], c[1], c[2]);
  _b.SetCenter(c);

  // Sample some points in the shell and check that they are contained by it.
  for(size_t i = 0; i < 1000; ++i) {
    std::vector<double> point = _b.GetRandomPoint();
    Vector3d v(point[0], point[1], point[2]);

    const bool contained = _b.InBoundary(point);
    const Vector3d clearancePoint = _b.GetClearancePoint(v);

    const double clearance = _b.GetClearance(v),
                 altClearance = (v - clearancePoint).norm(),
                 clearanceDiff = std::abs(clearance - altClearance);

    std::cout << "Test " << i << ":"
              << "\n\tSampled point: " << point
              << "\n\tDistance from center: " << (center - v).norm()
              << "\n\tContained: " << (contained ? "yes" : "no")
              << "\n\tClearance point: " << clearancePoint
              << "\n\tClearance: " << clearance
              << "\n\t(Clearance point - point).norm(): " << altClearance
              << "\n\tClearance discrepancy: " << clearanceDiff
              << std::endl;

    nonstd::assert_msg(contained, "The sampled point is outside the boundary!");
    nonstd::assert_msg(nonstd::approx(clearanceDiff, 0.), "Two clearance "
        "computations do not agree!");
  }
}

/*----------------------------------------------------------------------------*/

#endif
