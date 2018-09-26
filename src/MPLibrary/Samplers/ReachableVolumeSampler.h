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
/// @todo Add paper reference.
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

    /// Makes the RV samples given the joint placements of the joints in RV-space.
    std::vector<Vector3d> ConstructRVSample(const Vector3d& _center,
        const Chain& _chain, const std::vector<JointPlacement>& _jointPlacements,
        const Vector3d& _endEffectorPoint);

    /// Finds the position of the joint in RV-space if it is already sampled.
    /// Throws error if joint cannot be found.
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
  std::cout << "\nOriginal v_c =                          " << v_c
            << "\nv_rv =                                  " << v_rv
            << "\nv_c after manually applying transform = "
            << deltaRotation * v_c
            << "\nv_c after applying transform =          "
            << (baseTransformInWorldFrame.rotation() * baseToFirstJointInBaseFrame.translation()).normalize()
            << std::endl;
}


template <typename MPTraits>
void
ReachableVolumeSampler<MPTraits>::
ConvertToCfgSample(CfgType& _cfg, const std::vector<Vector3d>& _rvPoints) {
  //TODO: need to find a way to orient the joint angles.
  //Since finding the articulated and spherical joint angles
  //both involve using acos(theta) we don't know which quadrant
  //the angle is suppose to lie in, or how we determine the quadrants
  //in the first place. This is fundamental.

  AlignBase(_cfg, _rvPoints);

  //takes the consecutive points in RV space and computes the joint angles
  std::vector<double> jointData;
  //compute articulated angle for every joint
  for(auto iter1 = _rvPoints.begin(), iter2 = _rvPoints.begin() + 1,
           iter3 = _rvPoints.begin() + 2; iter3 != _rvPoints.end();
           iter1++, iter2++, iter3++) {
    //compute length of the sides of triangle
    double a = (*iter2 - *iter1).norm();
    double b = (*iter3 - *iter2).norm();
    double c = (*iter3 - *iter1).norm();
    std::cout << "a: " << a << std::endl << "b: " << b << std::endl
              << "c: " << c << std::endl;

    //use law of cosine to find joint angles and normalize it to be from 0 to 1
    double jointAngle1 = acos((c*c - a*a - b*b) / (-2 * a * b)) / M_PI;
    jointData.push_back(jointAngle1);
  }

  //compute rotational angle of first link
  Vector3d vector1(_rvPoints[0] - _rvPoints[1]);
  Vector3d vector2(_rvPoints[2] - _rvPoints[1]);
  Vector3d orthogonal = vector1 % vector2;

  //angle between orthogonal std::vector and upward std::vector
  auto base = _cfg.GetMultiBody()->GetBase();
  auto& connection = base->GetForwardConnection(0);
  auto transformation = base->GetWorldTransformation()
                      * connection.GetTransformationToDHFrame();

  // We aren't sure what the 'upward' direction is supposed to be here, but it
  // should be one of the basis vectors in the first joint's actuation frame.
  // The Z basis appears to generate no valid samples, although X and Y do not
  // appear to put the EE in the right place.
  const Vector3d upward = transformation.rotation().getBasis(0);

  //compute angle betweem upward and orthogonal
  double jointAngle2 = acos((upward * orthogonal) /
                            (upward.norm() * orthogonal.norm()));
  jointData.insert(jointData.begin(), jointAngle2);

  Vector3d oldOrth = orthogonal;
  //compute rotational angle for remaining links
  int i = 2;
  for(auto iter1 = _rvPoints.begin() + 1, iter2 = _rvPoints.begin() + 2,
           iter3 = _rvPoints.begin() + 3; iter3 != _rvPoints.end();
           iter1++, iter2++, iter3++) {
    //compute length of the sides of triangle
    vector1 = *iter1 - *iter2;
    vector2 = *iter3 - *iter2;
    orthogonal = vector1 % vector2;
    jointAngle2 = acos((oldOrth * orthogonal) /
      (oldOrth.norm() * orthogonal.norm())) / M_PI;
    //use law of cosine to find joint angles
    jointData.insert(jointData.begin() + i, jointAngle2);
    i += 2;
  }

  _cfg.SetJointData(jointData);
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
std::vector<Vector3d>
ReachableVolumeSampler<MPTraits>::
ConstructRVSample(const Vector3d& _center, const Chain& _chain,
                  const std::vector<JointPlacement>& _jointPlacements,
                  const Vector3d& _endEffectorPoint) {
  //constructs the rvSample by making a
  //std::vector of points corresponding to joints on the chain
  std::vector<Vector3d> rvSample;
  rvSample.reserve(2 + _chain.Size());
  rvSample.push_back(_center);
  for(auto iter = _chain.begin(); iter != _chain.end(); ++iter) {
    for(auto jointIter = _jointPlacements.begin();
             jointIter != _jointPlacements.end(); ++jointIter) {
      if(*iter == jointIter->first) {
        rvSample.push_back(jointIter->second);
      }
    }
  }
  rvSample.push_back(_endEffectorPoint);
  return rvSample;
}

template <typename MPTraits>
Vector3d
ReachableVolumeSampler<MPTraits>::
FindJointPosition(const std::vector<JointPlacement>& _points,
                  const Connection* _joint) {
    for(size_t i = 0; i < _points.size(); ++i) {
      if(_points[i].first == _joint)
  return _points[i].second;
    }
    throw RunTimeException(WHERE) << "Joint not found in the list of sampled points";
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

  std::pair <Chain, Chain> splitChain = _chain.Bisect();
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
  Chain chainCfg(multibody, std::move(joints), multibody->GetBase(),
      multibody->GetBody(multibody->GetNumBodies() - 1), true);
  std::vector<JointPlacement> jointPlacements;
  //set origin as the reference point forparent RV
  WorkspaceBoundingSphericalShell rv =
    ComputeReachableVolume(dim, std::vector<double>(dim, 0.), chainCfg);

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
  //SampleInternal(chainCfg, jointPlacements, pCenter, pEnd, dim);

  ////make the RV sample
  //std::vector<Vector3d> rvSample =
  //  ConstructRVSample(pCenter, chainCfg, jointPlacements, pEnd);

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
  if(!SampleInternal(chainCfg, jointPlacements, center, constraintCenter, dim))
    return false;

  //make the RV sample
  std::vector<Vector3d> rvSample =
    ConstructRVSample(center, chainCfg, jointPlacements, constraintCenter);

  // Convert RV sample to C-Space sample with random translational and orientation
  ConvertToCfgSample(_cfg, rvSample);

  _cfg.SetLinearPosition(center);
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
