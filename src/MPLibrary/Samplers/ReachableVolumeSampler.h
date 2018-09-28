#ifndef PMPL_REACHABLE_VOLUME_SAMPLER_H
#define PMPL_REACHABLE_VOLUME_SAMPLER_H

#include "SamplerMethod.h"
#include "Vector.h"
#include "ConfigurationSpace/ReachableVolumes.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphericalShell.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/Chain.h"


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
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ReachableVolumeSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename std::pair<Connection*, Vector3d> JointPlacement;

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
        Vector3d& _center, Vector3d& _endEffectorPoint, const size_t _dim);

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
  const std::vector<std::unique_ptr<Connection>>& mbJoints = multibody->GetJoints();

  const Transformation& baseTransformInWorldFrame = multibody->GetBase()->GetWorldTransformation();
  const Transformation& baseToFirstJointInBaseFrame = mbJoints[0]->GetTransformationToDHFrame();

  // Find the normalized vector v_c which points from the base to the first
  // joint (in the world frame) when the robot is at _cfg.
  const mathtool::Vector3d v_c = (baseTransformInWorldFrame.rotation() * baseToFirstJointInBaseFrame.translation()).normalize();

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
  const mathtool::Transformation newBaseTransform(baseTransformInWorldFrame.translation(),
      mathtool::MatrixOrientation(deltaRotation) * baseTransformInWorldFrame.rotation());
  _cfg.SetBaseTransformation(newBaseTransform);
  _cfg.ConfigureRobot();

  // Check stuff.
  //if(this->m_debug) {
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
              << (baseTransformInWorldFrame * baseToFirstJointInBaseFrame).translation()
              << std::endl;
  //}
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
              << std::endl;

    // Set the rotational and articulated angles for this joint.
    _cfg[dofIndex++] = rotationalAngle;
    _cfg[dofIndex++] = articulatedAngle;

    // Make sure the robot is updated with these joint angles for the next
    // iteration. We can probably do this more efficiently by adjusting
    // MultiBody::Configure to resolve lazily, but need to test that carefully.
    _cfg.ConfigureRobot();

    std::cout << "\nRV point " << jointNumber + 1 << ":       " << *iter2
              << "\nJoint " << jointNumber << " position: "
              << (joint->GetPreviousBody()->GetWorldTransformation()
                  * joint->GetTransformationToDHFrame()).translation()
              << std::endl;
  }

  std::cout << "\nLast RV point: " << _rvPoints.back()
            << "\nEE position:   "
            << _cfg.GetMultiBody()->GetBodies().back().GetWorldTransformation().
               translation()
            << std::endl;
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
  rvSample.reserve(2 + _chain.Size());

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
            Vector3d& _center, Vector3d& _endEffectorPoint, const size_t _dim) {
  //TODO: adapt this code to work with the given dimensions; right now only
  //3 dimensions is supported

  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "ReachableVolume::SampleInternal");
  //check ifchain is just one link
  if(_chain.IsSingleLink())
    return true;

  std::pair<Chain, Chain> splitChain = _chain.Bisect();
  Chain cl = splitChain.first;
  Chain cr = splitChain.second;
  if(this->m_debug) {
    std::cout << "left chain size: " << cl.Size() <<  std::endl;
    std::cout << "right chain size: " << cr.Size() <<  std::endl;
  }

  //make sure the chains are facing each other
  cl.IsForward() ? cl : cl.Reverse();
  cr.IsForward() ? cr.Reverse() : cr;

  //find the points to build rv relative to (the base of the chains)
  const Vector3d& leftPoint = cl.GetBase() ? _center :
            FindJointPosition(_jointPlacements, *(cl.begin()));
  const Vector3d& rightPoint = cr.GetBase() ? _endEffectorPoint :
            FindJointPosition(_jointPlacements, *(cr.begin()));

  std::vector<double> pl(_dim);
  std::vector<double> pr(_dim);

  for(size_t i = 0; i < _dim; ++i) {
    pl[i] = leftPoint[i];
    pr[i] = rightPoint[i];
  }

  //compute reachable volumes for both partitions
  WorkspaceBoundingSphericalShell rvl = ComputeReachableVolume(_dim, pl, cl);
  WorkspaceBoundingSphericalShell rvr = ComputeReachableVolume(_dim, pr, cr);

  if(this->m_debug) {
    std::cout << "Left point: " << leftPoint << std::endl;
    std::cout << "Right point: " << rightPoint << std::endl;
    std::cout << "Outer radius left rv: " << rvl.GetOuterRadius() << std::endl;
    std::cout << "Length of left rv: " <<
        rvl.GetOuterRadius() - rvl.GetInnerRadius() << std::endl;
    std::cout << "Outer radius right rv: " << rvr.GetOuterRadius() << std::endl;
    std::cout << "Length of right rv: " <<
        rvr.GetOuterRadius() - rvr.GetInnerRadius() << std::endl;
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
                                  << "This should not be possible";

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
  }

  //record where the last joint goes
  _jointPlacements.push_back(std::make_pair(cl.GetEnd(), jointSample));

  if(!SampleInternal(cl, _jointPlacements, _center, _endEffectorPoint, _dim)
      or !SampleInternal(cr, _jointPlacements, _center, _endEffectorPoint, _dim))
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
  const size_t dim = this->GetEnvironment()->GetBoundary()->GetDimension();

  MultiBody* const multibody = _cfg.GetMultiBody(); //get multibody of cfg
  const std::vector<std::unique_ptr<Connection>>& mbJoints = multibody->GetJoints();


  //assume joints are ordered consecutively based on index
  const size_t numJoints = mbJoints.size();
  std::deque<Connection*> joints(numJoints, nullptr); //initialize empty joint list
  for(size_t i = 0; i < numJoints; ++i) {
    joints[i] = (multibody->GetJoints()[i]).get(); //populate joint list
  }

  //if(Chain::Decompose(multibody1).size() != 1)
    //throw RunTimeException(WHERE) << "Only chain robots are supported right now.";
  //create a chain that corresponds to the cfg
  Chain chain(multibody, std::move(joints), multibody->GetBase(),
      multibody->GetBody(multibody->GetNumBodies() - 1), true);
  std::vector<JointPlacement> jointPlacements;
  //set origin as the reference point forparent RV
  WorkspaceBoundingSphericalShell rv =
    ComputeReachableVolume(dim, std::vector<double>(dim, 0.), chain);

  //----------------------------------------------------------------------------
  // DO NOT DELETE THIS. This section is for unconstrained planning.

  //sample a point for the end effector in its reachable volume
  //std::vector<double> endEffectorPoint = rv.GetRandomPoint();
  //std::vector<double> center = rv.GetCenter();

  //Vector3d pEnd;
  //for(size_t i = 0; i < dim; ++i)
  //  pEnd[i] = endEffectorPoint[i];

  //Vector3d pCenter;
  //for(size_t i = 0; i < dim; ++i)
  //  pCenter[i] = center[i];

  ////generate samples forthe internal joints
  //SampleInternal(chain, jointPlacements, pCenter, pEnd, dim);

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
  if(!SampleInternal(chain, jointPlacements, center, constraintCenter, dim))
    return false;

  //make the RV sample
  std::vector<Vector3d> rvSample =
    ConstructRVSample(center, chain, jointPlacements, constraintCenter);

  // Convert RV sample to C-Space sample with random translational and orientation
  ConvertToCfgSample(_cfg, rvSample);

  _cfg.ConfigureRobot();
  auto mb = _cfg.GetMultiBody();
  Vector3d eePoint = mb->GetBodies().back().GetWorldTransformation().translation();
  Vector3d basePoint = mb->GetBodies().front().GetWorldTransformation().translation();

  if(this->m_debug) {
    std::cout << "End Effector Random point: " << endEffectorPoint
              << "\nEE point: " << eePoint
              << "\nmatches: " << (rvSample.back() == constraintCenter)
              << std::endl;

    for(size_t i = 0; i < rvSample.size(); ++i)
      std::cout << "Point " << i << " in rv: " << rvSample[i] << std::endl;

    std::cout << "ee matches: " << (eePoint == rvSample.back())
              << " ("
              << (rvSample.back() - eePoint).norm()
              << " units apart)"
              << "\nbase matches: " << (basePoint == rvSample.front())
              << " ("
              << (rvSample.front() - basePoint).norm()
              << " units apart)"
              << std::endl;

    std::cout << "dist between two rv samples: ";
    for(size_t i = 1; i < rvSample.size(); ++i)
      std::cout << (rvSample[i] - rvSample[i - 1]).norm() << " ";
    std::cout << std::endl;

    std::cout << "Cfg generated: " << _cfg.PrettyPrint() << std::endl;
  }

  if(vcm->IsValid(_cfg, callee))
    _result.push_back(_cfg);
  else
    _collision.push_back(_cfg);

  return true;

}

/*----------------------------------------------------------------------------*/

#endif
