#ifndef PMPL_REACHABLE_VOLUME_SAMPLER_H
#define PMPL_REACHABLE_VOLUME_SAMPLER_H

#include "SamplerMethod.h"
#include "Vector.h"
#include "ConfigurationSpace/ReachableVolumes.h"
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

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        std::vector<CfgType>& _result, std::vector<CfgType>& _collision)
        override;

    /// @return False if we fail to generate a valid RV placement.
    bool SampleInternal(Chain& _chain,
        std::vector<JointPlacement>& _jointPlacements,
        Vector3d& _center, Vector3d& _endEffectorPoint);

    ///@}
    ///@name Helper functions
    ///@{

    /// Align the base of a randomly generated configuration so that the base
    /// and first joint match an RV sample. Must be done prior to computing the
    /// c-space joint angles from the RV sample.
    /// @param _cfg A randomly generated c-space sample, which will be
    ///             re-oriented to match _rvPoints.
    /// @param _rvPoints The generated RV sample.
    void AlignBase(CfgType& _cfg, const std::vector<Vector3d>& _rvPoints);

    /// Construct an RV sample from workspace positions for the base,
    /// end-effector, and each joint.
    /// @param _basePosition The position of the base in workspace.
    /// @param _chain The chain structure describing the joint ordering.
    /// @param _jointPlacements A set of workspace positions for each joint in
    ///        _chain.
    /// @param _endEffectorPoint The position of the end-effector in workspace.
    std::vector<Vector3d> ConstructRVSample(const Vector3d& _basePosition,
        const Chain& _chain, const std::vector<JointPlacement>& _jointPlacements,
        const Vector3d& _endEffectorPoint);

    /// Finds the workspace position of a joint from a set of joint placements.
    /// @param _points A set of workspace positions for each joint.
    /// @param _joint The joint of interest.
    /// @throw An exception if _points has no entry for _joint.
    /// @return The workspace position of _joint in _points.
    Vector3d FindJointPosition(const std::vector<JointPlacement>& _points,
        const Connection* _joint);

    Vector3d ComputePointonIntersection(
        const std::pair<Vector3d, double>& sphere1,
        const std::pair<Vector3d, double>& sphere2);

    void ConvertToCfgSample(CfgType& _cfg,
        const std::vector<Vector3d>& _rvPoints);

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
Print(std::ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << std::endl;
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
void
ReachableVolumeSampler<MPTraits>::
ConvertToCfgSample(CfgType& _cfg, const std::vector<Vector3d>& _rvPoints) {
  /// @note We will assume that the articulated angle is always positive to
  ///       simplify calculation of the rotational angle. We should eventually
  ///       adjust this to sample the sign of the articulated angle so that we
  ///       can generate samples from the full configuration space (including
  ///       negative articulated angles).

  /// @todo This function should probably be extracted to a reachable volume
  ///       object because it will eventually have to consider additional joint
  ///       types and base movements.

  // Align the base so that the base is at the first RV point and the first
  // joint is at the second RV point.
  AlignBase(_cfg, _rvPoints);

  // Initialize the joint angles at zero.
  _cfg.SetJointData(std::vector<double>(_cfg.JointDOF(), 0));
  _cfg.ConfigureRobot();

  // Keep track of the current DOF we are modifying.
  size_t dofIndex = _cfg.DOF() - _cfg.JointDOF();

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
    const Connection* const joint = _cfg.GetMultiBody()->GetJoint(jointNumber);
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
    _cfg[dofIndex++] = rotationalAngle;
    _cfg[dofIndex++] = articulatedAngle;

    // Make sure the robot is updated with these joint angles for the next
    // iteration. We can probably do this more efficiently by adjusting
    // MultiBody::Configure to resolve lazily, but need to test that carefully.
    _cfg.ConfigureRobot();


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
              << _cfg.GetMultiBody()->GetBodies().back().GetWorldTransformation().
                 translation()
              << "\nCfg generated: " << _cfg.PrettyPrint()
              << std::endl;
  }
}


template <typename MPTraits>
Vector3d
ReachableVolumeSampler<MPTraits>::
ComputePointonIntersection(const std::pair<Vector3d, double>& sphere1,
        const std::pair<Vector3d, double>& sphere2) {
  //calculate the center and radius of intersection circle
  const Vector3d& center1 = sphere1.first;
  const Vector3d& center2 = sphere2.first;

  double radius1 = sphere1.second;
  double radius2 = sphere2.second;

  //get distance between centers
  const Vector3d direction = (center2 - center1);
  double d = direction.norm();
  //get center of the intersecting circle
  double h = (1/2.0) + (radius1*radius1 - radius2*radius2) /
        (2 * d*d);

  double ri = std::sqrt(radius1*radius1 - h*h * d*d); //compute radius of intersecting circle
  Vector3d ci = center1 + h * direction; //compute center of intersect circle

  //calculate point on intersecting circle
  Vector3d axis(0,1,0);
  const Vector3d normal = direction / d;
  if(normal * axis > .9) {
    axis(1,0,0);
  }
  Vector3d tangent = (axis % normal).normalize();
  Vector3d bitangent = tangent % normal; //compute perp of tangent and normal

  //compute random point on this circle
  double rad = DRand() * (2.0 * M_PI);
  Vector3d point = ci + (ri * (tangent * cos(rad) + bitangent * sin(rad)));
  return point;
}


template <typename MPTraits>
std::vector<mathtool::Vector3d>
ReachableVolumeSampler<MPTraits>::
ConstructRVSample(const mathtool::Vector3d& _basePosition, const Chain& _chain,
                  const std::vector<JointPlacement>& _jointPlacements,
                  const mathtool::Vector3d& _endEffectorPoint) {
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
  rvSample.push_back(_endEffectorPoint);
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

/*------------------------------ Sampler Rule --------------------------------*/

template <typename MPTraits>
bool
ReachableVolumeSampler<MPTraits>::
SampleInternal(Chain& _chain, std::vector<JointPlacement>& _jointPlacements,
            Vector3d& _center, Vector3d& _endEffectorPoint) {
  //TODO: adapt this code to work with the given dimensions; right now only
  //3 dimensions is supported

  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "ReachableVolume::SampleInternal");

  // TODO
  // If the chain is just one link?
  if(_chain.IsSingleLink())
    return true;

  // Split the chain orient the subchains (left forward-oriented (base -> EE)
  // and right backward-oriented (EE -> base)).
  std::pair<Chain, Chain> splitChain = _chain.Bisect();
  Chain* leftChain, * rightChain;
  if(_chain.IsForward()) {
    leftChain = &splitChain.first;
    rightChain = &splitChain.second.Reverse();
  }
  else {
    leftChain = &splitChain.second.Reverse();
    rightChain = &splitChain.first;
  }

  if(this->m_debug) {
    std::cout << "\nleft chain: " << *leftChain
              << "\nright chain: " << *rightChain
              <<  std::endl;
  }

  // Find the base point for each chain.
  const Vector3d& leftPoint = leftChain->GetFrontBody()
      ? _center
      : FindJointPosition(_jointPlacements, *(leftChain->begin()));
  const Vector3d& rightPoint = rightChain->GetFrontBody()
      ? _endEffectorPoint
      : FindJointPosition(_jointPlacements, *(rightChain->begin()));

  const size_t dimension = this->GetEnvironment()->GetBoundary()->GetDimension();
  std::vector<double> pl(dimension);
  std::vector<double> pr(dimension);

  for(size_t i = 0; i < dimension; ++i) {
    pl[i] = leftPoint[i];
    pr[i] = rightPoint[i];
  }

  // Compute reachable volumes for both chains.
  WorkspaceBoundingSphericalShell rvl = ComputeReachableVolume(dimension, pl,
      *leftChain);
  WorkspaceBoundingSphericalShell rvr = ComputeReachableVolume(dimension, pr,
      *rightChain);

  if(this->m_debug) {
    std::cout << "\nLeft point: " << leftPoint
              << "\nRight point: " << rightPoint
              << "\nOuter radius left rv: " << rvl.GetOuterRadius()
              << "\nLength of left rv: "
              << rvl.GetOuterRadius() - rvl.GetInnerRadius()
              << "\nOuter radius right rv: " << rvr.GetOuterRadius()
              << "\nLength of right rv: "
              << rvr.GetOuterRadius() - rvr.GetInnerRadius()
              << std::endl;
  }


  //if the spherical shell has length 0 then do this
  Vector3d jointSample;
  // TODO: this style of sampling is temporary and is not efficient. We need a
  // more precise way to sample in intersections
  double radius1 = rvl.GetOuterRadius();
  double radius2 = rvr.GetOuterRadius();
  std::vector<double> center1 = rvl.GetCenter();
  std::vector<double> center2 = rvr.GetCenter();

  Vector3d c1;
  c1[0] = center1[0];
  c1[1] = center1[1];
  c1[2] = center1[2];

  Vector3d c2;
  c2[0] = center2[0];
  c2[1] = center2[1];
  c2[2] = center2[2];

  std::pair<Vector3d, double> sphere1 = make_pair(c1, radius1);
  std::pair<Vector3d, double> sphere2 = make_pair(c2, radius2);

  const double overlap = radius1 + radius2 - (c2 - c1).norm();

  if(overlap < 0)
    throw RunTimeException(WHERE) << "No overlap between RVs! "
                                  << "This should not be possible.";

  if(rvr.GetOuterRadius() == rvr.GetInnerRadius() and
     rvl.GetOuterRadius() == rvl.GetInnerRadius()) {
    //compute the intersection and sample on that intersection
    jointSample = ComputePointonIntersection(sphere1, sphere2);

    if(this->m_debug)
      std::cout << "intersecting point (circle) : " << jointSample << std::endl;
  }
  else {
     std::vector<double> jSample;
     constexpr const size_t maxAttempts = 1000;
     size_t attempts = 0;
     do {
       jSample = rvl.GetRandomPoint();
       stats->IncStat("ReachableVolumeSampler::SampleInternalAttempts");
     }
     while(!rvr.InBoundary(jSample) and ++attempts < maxAttempts);

     stats->IncStat("ReachableVolumeSampler::SampleInternalCompleted");
     stats->SetStat("ReachableVolumeSampler::SampleInternalAverage",
        stats->GetStat("ReachableVolumeSampler::SampleInternalAttempts") /
        stats->GetStat("ReachableVolumeSampler::SampleInternalCompleted"));

     // If we used up all the attempts, this sample failed.
     if(attempts == maxAttempts) {
       std::cout << "No valid point in both RVs." << std::endl;
       return false;
     }

     for(size_t i = 0; i < jSample.size(); ++i)
       jointSample[i] = jSample[i];

     if(this->m_debug)
       std::cout << "intersecting point: " << jointSample
                 << std::endl;
  }

  //record where the last joint goes
  _jointPlacements.emplace_back(leftChain->GetLastJoint(), jointSample);

  if(!SampleInternal(*leftChain, _jointPlacements, _center, _endEffectorPoint) or
      !SampleInternal(*rightChain, _jointPlacements, _center, _endEffectorPoint))
    return false;

  return true;
}

template <typename MPTraits>
bool
ReachableVolumeSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    std::vector<CfgType>& _result, std::vector<CfgType>& _collision) {
  const std::string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vcm = this->GetValidityChecker(m_vcLabel);
  const size_t dimension = this->GetEnvironment()->GetBoundary()->GetDimension();

  MultiBody* const multibody = _cfg.GetMultiBody(); //get multibody of cfg
  const std::vector<std::unique_ptr<Connection>>& mbJoints = multibody->GetJoints();


  //assume joints are ordered consecutively based on index
  const size_t numJoints = mbJoints.size();
  std::deque<Connection*> joints(numJoints, nullptr); //initialize empty joint list
  for(size_t i = 0; i < numJoints; ++i)
    joints[i] = multibody->GetJoint(i); //populate joint list

  // Decompose the robot into linear chains. Enforce our present assumption that
  // the robot is a single chain.
  std::vector<Chain> chains = Chain::Decompose(multibody);
  if(chains.size() != 1)
    throw RunTimeException(WHERE) << "Only single-chain robots are supported "
                                  << "right now, but Chain::Decompose generated "
                                  << chains.size() << " chains: "
                                  << chains
                                  << std::endl;
  Chain& chain = chains.front();

  std::cout << "\nChain: " << chain << std::endl;

  std::vector<JointPlacement> jointPlacements;
  //set origin as the reference point forparent RV
  WorkspaceBoundingSphericalShell rv =
    ComputeReachableVolume(dimension, std::vector<double>(dimension, 0.), chain);

  //----------------------------------------------------------------------------
  // DO NOT DELETE THIS. This section is for unconstrained planning.

  //sample a point for the end effector in its reachable volume
  //std::vector<double> endEffectorPoint = rv.GetRandomPoint();
  //std::vector<double> center = rv.GetCenter();

  //Vector3d pEnd;
  //for(size_t i = 0; i < dimension; ++i)
  //  pEnd[i] = endEffectorPoint[i];

  //Vector3d pCenter;
  //for(size_t i = 0; i < dimension; ++i)
  //  pCenter[i] = center[i];

  ////generate samples forthe internal joints
  //SampleInternal(chain, jointPlacements, pCenter, pEnd);

  ////make the RV sample
  //std::vector<Vector3d> rvSample =
  //  ConstructRVSample(pCenter, chain, jointPlacements, pEnd);

  ////convert RV sample to C-Space sample with random translational and orientation
  //ConvertToCfgSample(_cfg, rvSample);
  //----------------------------------------------------------------------------


  //sample a random end effector point in RV
  const std::vector<double> endEffectorPoint = rv.GetRandomPoint();
  const std::vector<double> efCenter(3, 0); //point to constrain end effector on

  //just doing one point (center) for now
  Vector3d constraintCenter;
  constraintCenter[0] = efCenter[0];
  constraintCenter[1] = efCenter[1];
  constraintCenter[2] = efCenter[2];

  //random end effector point
  Vector3d endPoint;
  endPoint[0] = endEffectorPoint[0];
  endPoint[1] = endEffectorPoint[1];
  endPoint[2] = endEffectorPoint[2];

  //transform reachable volume
  Vector3d center = constraintCenter - endPoint;

  // Generate samples for the internal joints. If it fails, return false.
  if(!SampleInternal(chain, jointPlacements, center, constraintCenter))
    return false;

  //make the RV sample
  std::vector<Vector3d> rvSample =
    ConstructRVSample(center, chain, jointPlacements, constraintCenter);

  // Convert RV sample to C-Space sample with random translational and orientation
  ConvertToCfgSample(_cfg, rvSample);

  if(vcm->IsValid(_cfg, callee))
    _result.push_back(_cfg);
  else
    _collision.push_back(_cfg);

  return true;

}

/*----------------------------------------------------------------------------*/

#endif
