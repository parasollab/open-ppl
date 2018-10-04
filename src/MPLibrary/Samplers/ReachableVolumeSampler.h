#ifndef PMPL_REACHABLE_VOLUME_SAMPLER_H
#define PMPL_REACHABLE_VOLUME_SAMPLER_H

#include "SamplerMethod.h"
#include "Vector.h"
#include "ConfigurationSpace/ReachableVolumes.h"
#include "Geometry/Boundaries/BoundaryIntersection.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphericalShell.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/Chain.h"

#include "nonstd/io.h"


////////////////////////////////////////////////////////////////////////////////
/// Creates a sample in reachable volume (RV) space and translates it into a
/// C-space configuration.
///
/// @warning Several assumptions right now, some of which are due to the support
///          code in ConfigurationSpace/ReachableVolumes and
///          Geometry/Bodies/Chain:
///          - The robot is a single, fixed chain
///          - The robot has volumetric, rotational base movement
///          - The robot has only spherical joints, and has at least one.
///
/// Paper reference:
///   Troy McMahon, Shawna Thomas, Nancy M. Amato. "Sampling Based Motion
///     Planning with Reachable Volumes for High Degree of Freedom
///     Manipulators." IJRR 2018.
///
/// @todo Changing Chain::Bisect so that the second chain takes the extra joint
///       causes this sampler to fail. This is likely an error here, to be
///       investigated later.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ReachableVolumeSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename std::pair<const Connection*, Vector3d> JointPlacement;

    ///@}
    ///@name Construction
    ///@{

    ReachableVolumeSampler();

    ReachableVolumeSampler(XMLNode& _node);

    virtual ~ReachableVolumeSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name SamplerMethod Overrides
    ///@{

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        std::vector<CfgType>& _result, std::vector<CfgType>& _collision)
        override;

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _robotBoundary,
        const Boundary* const _eeBoundary,
        std::vector<CfgType>& _result, std::vector<CfgType>& _collision)
        override;

    ///@}
    ///@name Helper functions
    ///@{

    /// Recursively sample a chain using reachable volumes to generate valid
    /// workspace positions for each joint.
    /// @param _chain The chain to sample.
    /// @param _jointPlacements The workspace point for each joint in _chain.
    /// @param _basePosition
    /// @return False if we fail to generate a valid RV placement.
    bool SampleInternal(const Chain& _chain,
        std::vector<JointPlacement>& _jointPlacements,
        const Vector3d& _basePosition, const Vector3d& _eePosition);

    /// Align the base of a randomly generated configuration so that the base
    /// and first joint match an RV sample. Must be done prior to computing the
    /// c-space joint angles from the RV sample.
    /// @param _cfg A c-space sample. Its base will be reconfigured to match
    ///             _rvPoints.
    /// @param _rvPoints The generated RV sample.
    void AlignBase(CfgType& _cfg, const std::vector<Vector3d>& _rvPoints);

    /// Construct an RV sample from workspace positions for the base,
    /// end-effector, and each joint.
    /// @param _basePosition The position of the base in workspace.
    /// @param _chain The chain structure describing the joint ordering.
    /// @param _jointPlacements A set of workspace positions for each joint in
    ///        _chain.
    /// @param _eePosition The position of the end-effector in workspace.
    std::vector<Vector3d> ConstructRVSample(const Vector3d& _basePosition,
        const Chain& _chain, const std::vector<JointPlacement>& _jointPlacements,
        const Vector3d& _eePosition);

    /// Finds the workspace position of a joint from a set of joint placements.
    /// @param _points A set of workspace positions for each joint.
    /// @param _joint The joint of interest.
    /// @throw An exception if _points has no entry for _joint.
    /// @return The workspace position of _joint in _points.
    Vector3d FindJointPosition(const std::vector<JointPlacement>& _points,
        const Connection* _joint);

    /// Compute a point in the intersection of two reachable volumes.
    /// @param _leftRV The reachable volume of the left chain.
    /// @param _rightRV The reachable volume of the right chain.
    /// @return A point inside both _leftRV and _rightRV. Second element will be
    ///         true iff the sampling succeeded..
    std::pair<Vector3d, bool> ComputePointOnIntersection(
        const WorkspaceBoundingSphericalShell& _leftRV,
        const WorkspaceBoundingSphericalShell& _rightRV);

    /// Convert an RV-space sample to its C-space equivalent.
    /// @param _rvPoints The RV-space sample.
    /// @return A configuration which places the robot at the RV sample.
    CfgType ConvertToCfgSample(const std::vector<Vector3d>& _rvPoints);

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_vcLabel; ///< The validity checker to use.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ReachableVolumeSampler<MPTraits>::
ReachableVolumeSampler() {
  this->SetName("ReachableVolumeSampler");
}


template <typename MPTraits>
ReachableVolumeSampler<MPTraits>::
ReachableVolumeSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("ReachableVolumeSampler");

  m_vcLabel = _node.Read("vcLabel", true, "", "ValidityChecker method to use.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
ReachableVolumeSampler<MPTraits>::
Initialize() {
  // For now we require a three-dimensional environment. Assert that now.
  const size_t dimension = this->GetEnvironment()->GetBoundary()->GetDimension();
  if(dimension != 3)
    throw RunTimeException(WHERE) << "Currently requires 3-d environment, but "
                                  << "this problem has " << dimension
                                  << " workspace dimensions.";
}


template <typename MPTraits>
void
ReachableVolumeSampler<MPTraits>::
Print(std::ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << std::endl;
}

/*-------------------------- SamplerMethod Overrides -------------------------*/

template <typename MPTraits>
bool
ReachableVolumeSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    std::vector<CfgType>& _result, std::vector<CfgType>& _collision) {
  return Sampler(_cfg, _boundary, nullptr, _result, _collision);
}


template <typename MPTraits>
bool
ReachableVolumeSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _robotBoundary,
    const Boundary* const _eeBoundary,
    std::vector<CfgType>& _result, std::vector<CfgType>& _collision) {
  MethodTimer mt(this->GetStatClass(), "ReachableVolumeSampler::Sampler");

  // Decompose the robot into linear chains. Enforce our present assumption that
  // the robot is a single chain.
  std::vector<Chain> chains = Chain::Decompose(_cfg.GetMultiBody());
  if(chains.size() != 1)
    throw RunTimeException(WHERE) << "Only single-chain robots are supported "
                                  << "right now, but Chain::Decompose generated "
                                  << chains.size() << " chains: "
                                  << chains
                                  << std::endl;
  Chain& chain = chains.front();

  if(this->m_debug)
    std::cout << "\nChain: " << chain << std::endl;

  const size_t dimension = this->GetEnvironment()->GetBoundary()->GetDimension();
  const bool threeD = dimension == 3;

  // Find points for the base and end-effector.
  Vector3d basePoint(0, 0, 0),
           eePoint(0, 0, 0);

  // If there is an EE boundary, sample a point from within.
  if(_eeBoundary) {
    // Sample an EE position from its boundary.
    const std::vector<double> eeSample = _eeBoundary->GetRandomPoint();

    // Reverse the chain and find the reachable volume of the base relative to
    // this EE position.
    chain.Reverse();
    const WorkspaceBoundingSphericalShell rv = ComputeReachableVolume(dimension,
        eeSample, chain);
    chain.Reverse();

    // Sample a random base point from this reachable volume.
    const std::vector<double> baseSample = rv.GetRandomPoint();

    basePoint(baseSample[0], baseSample[1], threeD ? baseSample[2] : 0.);
    eePoint(    eeSample[0],   eeSample[1], threeD ?   eeSample[2] : 0.);
  }
  else {
    // If there is no EE boundary, use the sampled base point.
    basePoint = _cfg.GetPoint();

    // Compute the RV of the chain relative to the sampled base point.
    const WorkspaceBoundingSphericalShell rv = ComputeReachableVolume(dimension,
        std::vector<double>{basePoint[0], basePoint[1], threeD ? basePoint[2] : 0.},
        chain);

    // Set the end-effector at a point in the chain's RV.
    const std::vector<double> sample = rv.GetRandomPoint();
    eePoint(sample[0], sample[1], threeD ? sample[2] : 0.);
  }

  // Generate samples for the internal joints. If it fails, return false.
  std::vector<JointPlacement> jointPlacements;
  if(!SampleInternal(chain, jointPlacements, basePoint, eePoint))
    return false;

  // Make a sample in RV space.
  const std::vector<Vector3d> rvSample = ConstructRVSample(basePoint, chain,
      jointPlacements, eePoint);

  // Convert RV sample to C-space sample. Translation was already randomized
  // prior to input.
  _cfg = ConvertToCfgSample(rvSample);

  // Check for validity.
  auto vcm = this->GetValidityChecker(m_vcLabel);
  const bool valid = vcm->IsValid(_cfg, this->GetNameAndLabel() + "::Sampler");
  if(valid)
    _result.push_back(_cfg);
  else
    _collision.push_back(_cfg);

  return valid;
}

/*----------------------------Helper Functions--------------------------------*/

template <typename MPTraits>
void
ReachableVolumeSampler<MPTraits>::
AlignBase(CfgType& _cfg, const std::vector<Vector3d>& _rvPoints) {
  // Set the base at the base point.
  _cfg.SetLinearPosition(_rvPoints.front());
  _cfg.ConfigureRobot();

  // Get the transforms we will need to align the base so that the first joint
  // lands at _rvPoints[1].
  MultiBody* const multibody = _cfg.GetMultiBody();
  const std::vector<std::unique_ptr<Connection>>& mbJoints = multibody->
      GetJoints();

  const Transformation& baseTransformInWorldFrame = multibody->GetBase()->
      GetWorldTransformation();
  const Transformation& baseToFirstJointInBaseFrame = mbJoints[0]->
      GetTransformationToDHFrame();

  // Find the normalized vector v_c which points from the base to the first
  // joint (in the world frame) when the robot is at _cfg.
  const mathtool::Vector3d v_c = (baseTransformInWorldFrame.rotation()
      * baseToFirstJointInBaseFrame.translation()).normalize();

  // Find the normalized vector v_rv which points from the base to the first
  // joint (in the world frame) when the robot is configured at an RV sample
  // described by _rvPoints.
  const mathtool::Vector3d v_rv = (_rvPoints[1] - _rvPoints[0]).normalize();

  // Find an axis of rotation which is parallel to both v_c and v_rv, such that
  // a positive angle represents rotation from v_c towards v_rv.
  const mathtool::Vector3d axis = (v_c % v_rv).normalize();

  // Find the angle needed to rotate v_c to v_rv about axis.
  const double angle = std::acos(v_c * v_rv);

  // Create a rotation matrix and corresponding transformation that describes a
  // rotation of (angle) radians about the axis vector.
  mathtool::Matrix3x3 deltaRotation;
  convertFromEulerVector(deltaRotation, axis * angle);

  // Create a new transformation for the base which aligns v_c with v_rv. This
  // should place the first joint's DH frame at _rvPoints[1].
  const mathtool::Transformation newBaseTransform(
      baseTransformInWorldFrame.translation(),
      mathtool::MatrixOrientation(deltaRotation)
      * baseTransformInWorldFrame.rotation());
  _cfg.SetBaseTransformation(newBaseTransform);
  _cfg.ConfigureRobot();

  // Check stuff.
  if(this->m_debug) {
    std::cout << "\nChecking AlignBase"
              << std::setprecision(4)
              << "\nOriginal v_c =                          " << v_c
              << "\nv_rv =                                  " << v_rv
              << "\nv_c after manually applying transform = "
              << deltaRotation * v_c
              << "\nv_c after applying transform =          "
              << (baseTransformInWorldFrame.rotation() *
                  baseToFirstJointInBaseFrame.translation()).normalize()
              << "\nRV point 0:      " << _rvPoints[0]
              << "\nbase position:   " << baseTransformInWorldFrame.translation()
              << "\nRV point 1:      " << _rvPoints[1]
              << "\njoint0 position: "
              << (baseTransformInWorldFrame
                  * baseToFirstJointInBaseFrame).translation()
              << std::endl;
  }
}


template <typename MPTraits>
typename MPTraits::CfgType
ReachableVolumeSampler<MPTraits>::
ConvertToCfgSample(const std::vector<Vector3d>& _rvPoints) {
  /// @note We will assume that the articulated angle is always positive to
  ///       simplify calculation of the rotational angle. We should eventually
  ///       adjust this to sample the sign of the articulated angle so that we
  ///       can generate samples from the full configuration space (including
  ///       negative articulated angles).

  /// @todo This function should probably be extracted to a reachable volume
  ///       object because it will eventually have to consider additional joint
  ///       types and base movements.

  // Create a zero configuration for the current robot.
  CfgType cfg(this->GetTask()->GetRobot());

  // Align the base so that the base is at the first RV point and the first
  // joint is at the second RV point.
  AlignBase(cfg, _rvPoints);

  // Keep track of the current DOF we are modifying.
  size_t dofIndex = cfg.DOF() - cfg.JointDOF();

  // Compute the angles for every joint (angle from vertical). The current
  // layout for spherical joints has the rotational angle first and articulated
  // angle second.
  for(auto iter1 = _rvPoints.begin(), iter2 = _rvPoints.begin() + 1,
           iter3 = _rvPoints.begin() + 2; iter3 != _rvPoints.end();
           ++iter1, ++iter2, ++iter3) {
    // Diagram of what we are doing here:
    //        iter3
    //        /  |     The first and last _rvPoints are for the base and end-
    // \     /b  |       effector, so iter2 always represents the position of the
    //  \art/    |       current joint for which we are computing the angles.
    //   \ /     |     The 'articulated angle' art is the angle between the
    //   iter2   |c      vectors a and b when both are placed at the same origin.
    //     \     |       This creates a right-hand rotation of the child link
    //      \    |       about the joint's local +x axis.
    //      a\   |     The 'rotational angle' describes the rotation of joint's
    //        \  |       local +x axis relative to the zero configuration.
    //        iter1

    // Compute the relevant vectors from the rv sample. The 'orthogonal'
    // direction here is the rotated +x axis in world coordinates.
    const Vector3d aVec = *iter2 - *iter1,
                   bVec = *iter3 - *iter2,
                   aHat = aVec.normalize(),
                   bHat = bVec.normalize(),
                   orthogonal = aHat % bHat;

    // Use law of cosines to compute the articulated angle (angle away from
    // vertical in radians) and divide by PI for PMPL's normalized representation.
    const double articulatedAngle = std::acos(aHat * bHat) / PI;

    // Transform orthogonal to the joint's local (pre-rotated) frame. It should
    // not have any Z component in the local frame because this is the axis of
    // rotation.
    const size_t jointNumber = std::distance(_rvPoints.begin(), iter2) - 1;
    const Connection* const joint = cfg.GetMultiBody()->GetJoint(jointNumber);
    auto transformation = joint->GetPreviousBody()->GetWorldTransformation()
                        * joint->GetTransformationToDHFrame();

    const double orthNorm = orthogonal.norm();
    const Vector3d orthLocal = -(transformation.rotation()) * orthogonal /
                               orthNorm;

    // Compute the rotational angle (rotation about vertical). If the orthogonal
    // vector is very small, then the rotational angle is approximately zero
    // and atan2 may not give a reasonable answer depending on the
    // implementation (use 0 in this case).
    const double rotationalAngle = nonstd::approx(orthNorm, 0., 1e-8)
                                 ? 0.
                                 : std::atan2(orthLocal[1], orthLocal[0]) / PI;

    // Set the rotational and articulated angles for this joint.
    cfg[dofIndex++] = rotationalAngle;
    cfg[dofIndex++] = articulatedAngle;

    // Make sure the robot is updated with these joint angles for the next
    // iteration. We can probably do this more efficiently by adjusting
    // MultiBody::Configure to resolve lazily, but need to test that carefully.
    cfg.ConfigureRobot();


    if(this->m_debug) {
      std::cout << "\nChecking angle computations:"
                << "\ntransfomation: " << transformation
                << std::setprecision(4)
                << "\naVec = parent -> joint" << jointNumber << ": " << aVec
                << "\nbVec = joint" << jointNumber << " -> child:  " << bVec
                << "\northogonal = aHat % bHat:  " << orthogonal
                << "\northLocal:    " << orthLocal
                << "\nx basis:      " << transformation.rotation().getBasis(0)
                << "\ny basis:      " << transformation.rotation().getBasis(1)
                << "\nz basis:      " << transformation.rotation().getBasis(2)
                << "\nrotationalAngle: " << rotationalAngle
                << "\narticulatedAngle: " << articulatedAngle
                << "\nRV point " << jointNumber + 1 << ":       " << *iter2
                << "\nJoint " << jointNumber << " position: "
                << (joint->GetPreviousBody()->GetWorldTransformation()
                    * joint->GetTransformationToDHFrame()).translation()
                << std::endl;
    }
  }

  if(this->m_debug) {
    std::cout << "\nLast RV point: " << _rvPoints.back()
              << "\nEE position:   "
              << cfg.GetMultiBody()->GetBodies().back().GetWorldTransformation().
                 translation()
              << "\nCfg generated: " << cfg.PrettyPrint()
              << std::endl;
  }

  return cfg;
}


template <typename MPTraits>
std::pair<Vector3d, bool>
ReachableVolumeSampler<MPTraits>::
ComputePointOnIntersection(const WorkspaceBoundingSphericalShell& _leftRV,
    const WorkspaceBoundingSphericalShell& _rightRV) {
  auto stats = this->GetStatClass();

  // Ensure there is some overlap between the reachable volumes.
  const double leftRadius  = _leftRV.GetOuterRadius(),
               rightRadius = _rightRV.GetOuterRadius();

  const Vector3d leftCenter(_leftRV.GetCenter().data()),
                 rightCenter(_rightRV.GetCenter().data()),
                 direction(rightCenter - leftCenter);

  const double distance = direction.norm(),
               overlap = leftRadius + rightRadius - distance;
  if(overlap < 0)
    throw RunTimeException(WHERE) << "No overlap between RVs! "
                                  << "This should not be possible.";
  /// @todo If overlap is greater than but close to 0, we should just pick
  ///       the overlap point as it will be very hard to sample.

  // Sample the splitting joint placement.
  std::pair<Vector3d, bool> jointPosition{Vector3d(), false};

  // If either RV has volume, we must presently use rejection sampling.
  if(_rightRV.GetOuterRadius() != _rightRV.GetInnerRadius() or
      _leftRV.GetOuterRadius() != _leftRV.GetInnerRadius())
  {
    MethodTimer mt(stats, "ReachableVolumeSampler::RejectionSampling");

    constexpr const size_t maxAttempts = 1000;
    size_t attempts = 0;
    const std::vector<double> sample = RejectionSampleIntersection(
        &_leftRV, &_rightRV, maxAttempts, &attempts);

    auto& success = stats->GetAverage(
        "ReachableVolumeSampler::RejectionSamplingSuccessRate");

    // The sampling failed if the result is empty.
    if(sample.empty()) {
      success.AddSummedValues(0, attempts);
      if(this->m_debug)
        std::cout << "No valid point in both RVs." << std::endl;
      return jointPosition;
    }

    // Otherwise we successfully generated a sample.
    success.AddSummedValues(1, attempts);
    jointPosition.first = Vector3d(sample.data());
    jointPosition.second = true;
  }
  // If both RVs have no volume, then they intersect along a single circle.
  // Compute this directly and sample it.
  else {
    MethodTimer mt(stats, "ReachableVolumeSampler::CircleSampling");

    // Compute the intersecting circle.
    const double dd  = distance * distance,
                 lrr = leftRadius * leftRadius,
                 rrr = rightRadius * rightRadius,
                 h   = .5 + (lrr - rrr) / (2 * dd),
                 intersectingRadius  = std::sqrt(lrr - h*h * dd);

    const Vector3d intersectingCenter = leftCenter + h * direction;

    // Find two orthogonal vectors in the plane of the intersecting circle.
    const Vector3d normal = direction / distance;
    Vector3d random(0,1,0);
    if(normal * random > .9)
      random(1,0,0);
    const Vector3d localX = (random % normal).selfNormalize(),
                   localY = (normal % localX).selfNormalize();

    // Sample a random unit vector in the frame of localX, localY.
    const double radians = DRand() * TWOPI;
    const Vector3d localUnit = localX * std::cos(radians)
                             + localY * std::sin(radians);

    // Compute a point on the intersection circle using the center, radius, and
    // random unit vector.
    jointPosition.first = intersectingCenter + intersectingRadius * localUnit;
    jointPosition.second = true;
  }

  if(this->m_debug)
    std::cout << "Sampled intersecting point: " << jointPosition.first
              << std::endl;

  return jointPosition;
}


template <typename MPTraits>
std::vector<mathtool::Vector3d>
ReachableVolumeSampler<MPTraits>::
ConstructRVSample(const mathtool::Vector3d& _basePosition, const Chain& _chain,
    const std::vector<JointPlacement>& _jointPlacements,
    const mathtool::Vector3d& _eePosition) {
  // Create storage for a reachable volume sample. Reserve space for each link.
  std::vector<Vector3d> rvSample;
  rvSample.reserve(_chain.Size());

  // The first RV point is the location of the base.
  rvSample.push_back(_basePosition);

  // Find the position for each joint in the chain.
  /// @todo Fix this, we shouldn't need to search the entire joint placement
  ///       list each time. We should either force the order of _jointPlacements
  ///       to match _chain or use a map from Connection* to position.
  for(auto iter = _chain.begin(); iter != _chain.end(); ++iter)
    rvSample.push_back(FindJointPosition(_jointPlacements, *iter));

  // The last point is the location of the end-effector.
  rvSample.push_back(_eePosition);
  return rvSample;
}


template <typename MPTraits>
mathtool::Vector3d
ReachableVolumeSampler<MPTraits>::
FindJointPosition(const std::vector<JointPlacement>& _points,
    const Connection* _joint) {
  // Check each joint placement to look for _joint, and return its position if
  // found.
  for(size_t i = 0; i < _points.size(); ++i)
    if(_points[i].first == _joint)
      return _points[i].second;

  throw RunTimeException(WHERE) << "Joint not found in the list of sampled points.";
}


template <typename MPTraits>
bool
ReachableVolumeSampler<MPTraits>::
SampleInternal(const Chain& _chain, std::vector<JointPlacement>& _jointPlacements,
    const Vector3d& _basePosition, const Vector3d& _eePosition) {
  // If the chain is just one link, we have already placed all of the attached
  // joints.
  if(_chain.IsSingleLink())
    return true;

  // Split the chain orient the subchains (left forward-oriented (base -> EE)
  // and right backward-oriented (EE -> base)).
  std::pair<Chain, Chain> splitChain = _chain.Bisect();
  Chain* leftChain,
       * rightChain;
  if(_chain.IsForward()) {
    leftChain = &splitChain.first;
    rightChain = &splitChain.second.Reverse();
  }
  else {
    leftChain = &splitChain.second.Reverse();
    rightChain = &splitChain.first;
  }

  // Find the root point for each chain.
  const Vector3d& leftRoot = leftChain->GetFrontBody()
      ? _basePosition
      : FindJointPosition(_jointPlacements, *(leftChain->begin()));
  const Vector3d& rightRoot = rightChain->GetFrontBody()
      ? _eePosition
      : FindJointPosition(_jointPlacements, *(rightChain->begin()));

  const size_t dimension = this->GetEnvironment()->GetBoundary()->GetDimension();
  const bool threeD = dimension == 3;

  // Compute reachable volumes for both chains relative to their root points.
  const WorkspaceBoundingSphericalShell rvl = ComputeReachableVolume(dimension,
      { leftRoot[0],  leftRoot[1], threeD ?  leftRoot[2] : 0.}, *leftChain);
  const WorkspaceBoundingSphericalShell rvr = ComputeReachableVolume(dimension,
      {rightRoot[0], rightRoot[1], threeD ? rightRoot[2] : 0.}, *rightChain);

  if(this->m_debug) {
    std::cout << "\nLeft chain: " << *leftChain
              << "\nLeft RV:"
              << "\n\troot: " << leftRoot
              << "\n\tInner radius: " << rvl.GetInnerRadius()
              << "\n\tOuter radius: " << rvl.GetOuterRadius()
              << "\n\tThickness: " << rvl.GetOuterRadius() - rvl.GetInnerRadius()
              << "\nRight chain: " << *rightChain
              << "\nRight RV:"
              << "\n\troot: " << rightRoot
              << "\n\tInner radius: " << rvr.GetInnerRadius()
              << "\n\tOuter radius: " << rvr.GetOuterRadius()
              << "\n\tThickness: " << rvr.GetOuterRadius() - rvr.GetInnerRadius()
              << std::endl;
  }

  // Compute a point which lies in both reachable volumes.
  const auto jointPosition = ComputePointOnIntersection(rvl, rvr);

  // If no point was found, the sampling process has failed.
  if(!jointPosition.second)
    return false;

  _jointPlacements.emplace_back(leftChain->GetLastJoint(), jointPosition.first);

  if(!SampleInternal(*leftChain, _jointPlacements, _basePosition, _eePosition) or
      !SampleInternal(*rightChain, _jointPlacements, _basePosition, _eePosition))
    return false;

  return true;
}

/*----------------------------------------------------------------------------*/

#endif
