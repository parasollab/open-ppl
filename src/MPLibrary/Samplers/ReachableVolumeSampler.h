#ifndef PMPL_RV_SAMPLER_H
#define PMPL_RV_SAMPLER_H

//#include "CGAL/Exact_spherical_kernel_3.h"
//#include "CGAL/Object.h"
//#include "CGAL/Circle_3.h"
////#include "CGAL/intersections.h"
//#include "CGAL/Spherical_kernel_intersections.h"

//#include <cmath>
#include "SamplerMethod.h"
#include "Vector.h"
#include "ConfigurationSpace/ReachableVolumes.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphericalShell.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/Chain.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Creates a sample in ReachableVolumeSpace and translates it into a C-space configuration.
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
//typedef CGAL::Exact_spherical_kernel_3         Spherical_k;
//typedef CGAL::Point_3<Spherical_k>             Point_3;
//typedef CGAL::Sphere_3<Spherical_k>            Sphere_3;
//typedef CGAL::Circle_3<Spherical_k>            Circle_3;

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

    ReachableVolumeSampler(string _vcLabel = "");
    ReachableVolumeSampler(XMLNode& _node);
    virtual ~ReachableVolumeSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        std::vector<CfgType>& _result, std::vector<CfgType>& _collision)
        override;

    virtual bool Sampler(CfgType& _cfg,
        const Boundary* const _endEffectorConstraint,
	const Boundary* const _boundary,
        std::vector<CfgType>& _result, std::vector<CfgType>& _collision);

    void SampleInternal(Chain& _chain,
        std::vector<JointPlacement>& _jointPlacements,
        Vector3d& _center, Vector3d& _endEffectorPoint, const size_t _dim);

    ///@}
    ///@name Helper functions
    ///@{

    //makes the RV samples given the joint placements of the joints in RV-space
    std::vector<Vector3d> ConstructRVSample(const Vector3d& _center,
        const Chain& _chain, const std::vector<JointPlacement>& _jointPlacements,
        const Vector3d& _endEffectorPoint);

    //Finds the position of the joint in RV-space ifit is already sample; throws error ifjoint cannot be found
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

    string m_vcLabel;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ReachableVolumeSampler<MPTraits>::
ReachableVolumeSampler(string _vcLabel) : m_vcLabel(_vcLabel) {
  this->SetName("ReachableVolumeSampler");
}


template <typename MPTraits>
ReachableVolumeSampler<MPTraits>::
ReachableVolumeSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("ReachableVolumeSampler");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
ReachableVolumeSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
}

/*----------------------------Helper Functions--------------------------------*/

template <typename MPTraits>
void
ReachableVolumeSampler<MPTraits>::
ConvertToCfgSample(CfgType& _cfg, const std::vector<Vector3d>& _rvPoints) {
  //TODO: need to find a way to orient the joint angles.
  //Since finding the articulated and spherical joint angles
  //both involve using acos(theta) we don't know which quadrant
  //the angle is suppose to lie in, or how we determine the quadrants
  //in the first place. This is fundamental.

  //takes the consecutive points in RV space and computes the joint angles
  std::vector<double> jointData;
  //compute articulated angle for every joint
  for(auto iter1 = _rvPoints.begin(),
	  iter2 = _rvPoints.begin() + 1,
	  iter3 = _rvPoints.begin() + 2;
	  iter3 != _rvPoints.end(); iter1++, iter2++, iter3++) {
    //compute length of the sides of triangle
    double a = (*iter2 - *iter1).norm();
    double b = (*iter3 - *iter2).norm();
    double c = (*iter3 - *iter1).norm();
    cout << "a: " << a
	 << "b: " << b
	 << "c: " << c << endl;

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

  // We aren't sure what the 'upward' direction is supposed to be here, this
  // line assumes it is the +Z direction of the first joint's actuation frame.
  const Vector3d upward = transformation.rotation().getBasis(2);

  //compute angle betweem upward and orthogonal
  double jointAngle2 = acos((upward * orthogonal) /
		      (upward.norm() * orthogonal.norm()));
  jointData.insert(jointData.begin(), jointAngle2);

  Vector3d oldOrth = orthogonal;
  //compute rotational angle for remaining links
  int i = 2;
  for(auto iter1 = _rvPoints.begin() + 1,
            iter2 = _rvPoints.begin() + 2,
	    iter3 = _rvPoints.begin() + 3;
	  iter3 != _rvPoints.end();
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
  if(normal * axis > .9){
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
ConstructRVSample(const Vector3d& _center,
		  const Chain& _chain,
		  const std::vector<JointPlacement>& _jointPlacements,
		  const Vector3d& _endEffectorPoint) {
  //constructs the rvSample by making a
  //std::vector of points corresponding to joints on the chain
  std::vector<Vector3d> rvSample;
  rvSample.reserve(2 + _chain.Size());
  rvSample.push_back(_center);
  for(auto iter = _chain.begin();
	    iter != _chain.end(); ++iter) {
      for(auto jointIter = _jointPlacements.begin();
		jointIter != _jointPlacements.end(); ++jointIter){
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
void
ReachableVolumeSampler<MPTraits>::
SampleInternal(Chain& _chain, std::vector<JointPlacement>& _jointPlacements,
		Vector3d& _center, Vector3d& _endEffectorPoint, const size_t _dim) {
  //TODO: adapt this code to work with the given dimensions; right now only
  //3 dimensions is supported

  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "ReachableVolume::SampleInternal");
  //check ifchain is just one link
  if(_chain.IsSingleLink())
    return;

  std::pair <Chain, Chain> splitChain = _chain.Bisect();
  Chain cl = splitChain.first;
  Chain cr = splitChain.second;
  if(this->m_debug) {
    cout << "left chain size: " << cl.Size() <<  endl;
    cout << "right chain size: " << cr.Size() <<  endl;
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

  //compute reachable volumes forboth partitions
  WorkspaceBoundingSphericalShell rvl = ComputeReachableVolume(_dim, pl, cl);
  WorkspaceBoundingSphericalShell rvr = ComputeReachableVolume(_dim, pr, cr);

  if(this->m_debug) {
    cout << "Left point: " << leftPoint << endl;
    cout << "Right point: " << rightPoint << endl;
    cout << "Outer radius left rv: " << rvl.GetOuterRadius() << endl;
    cout << "Length of left rv: " <<
	      rvl.GetOuterRadius() - rvl.GetInnerRadius() << endl;
    cout << "Outer radius right rv: " << rvr.GetOuterRadius() << endl;
    cout << "Length of right rv: " <<
	      rvr.GetOuterRadius() - rvr.GetInnerRadius() << endl;
  }


  //ifthe spherical shell has length 0 then do this
  Vector3d jointSample;
  //TODO: this style of sampling is temporary and
  //is not efficient. We need a more precise way to
  //sample in intersections
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

  if(rvr.GetOuterRadius() == rvr.GetInnerRadius() &&
      rvl.GetOuterRadius() == rvl.GetInnerRadius()) {
    //compute the intersection and sample on that intersection

    jointSample = ComputePointonIntersection(sphere1, sphere2);

    if(this->m_debug)
      cout << "intersecting point (circle) : " << jointSample << "\n" << endl;

  }
   else {
     std::vector<double> jSample;
     do {
       jSample = rvl.GetRandomPoint();
       stats->IncStat("ReachableVolumeSampler::SampleInternalAttempts");
     }
     while (!rvr.InBoundary(jSample));
     stats->IncStat("ReachableVolumeSampler::SampleInternalCompleted");
     stats->SetStat("ReachableVolumeSampler::SampleInternalAverage",
		    stats->GetStat("ReachableVolumeSampler::SampleInternalAttempts") /
		    stats->GetStat("ReachableVolumeSampler::SampleInternalCompleted"));
     for(size_t i = 0; i < jSample.size(); ++i)
       jointSample[i] = jSample[i];
  }

  //record where the last joint goes
  _jointPlacements.push_back(std::make_pair(cl.GetEnd(), jointSample));

  SampleInternal(cl, _jointPlacements, _center, _endEffectorPoint, _dim);
  SampleInternal(cr, _jointPlacements, _center, _endEffectorPoint, _dim);
}

template <typename MPTraits>
bool
ReachableVolumeSampler<MPTraits>::
Sampler(CfgType& _cfg,
	const Boundary* const _boundary,
	std::vector<CfgType>& _result, std::vector<CfgType>& _collision) {

  //initialize variables
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vcm = this->GetValidityChecker(m_vcLabel);
  const size_t dim = this->GetEnvironment()->GetBoundary()->GetDimension();

  MultiBody* multibody1 = _cfg.GetMultiBody(); //get multibody of cfg

  //assume joints are ordered consecutively based on index
  const size_t numJoints = multibody1->GetJoints().size();
  std::deque<Connection*> joints(numJoints, nullptr); //initialize empty joint list
  for(size_t i = 0; i < numJoints; ++i) {
    joints[i] = (multibody1->GetJoints()[i]).get(); //populate joint list
  }

  //if(Chain::Decompose(multibody1).size() != 1)
    //throw RunTimeException(WHERE) << "Only chain robots are supported right now.";
  //create a chain that corresponds to the cfg
  Chain chainCfg(multibody1, std::move(joints), multibody1->GetBase(),
		  multibody1->GetBody(multibody1->GetNumBodies() - 1), true);
  std::vector<JointPlacement> jointPlacements;
  //set origin as the reference point forparent RV
  WorkspaceBoundingSphericalShell rv =
    ComputeReachableVolume(dim, std::vector<double>(dim, 0.), chainCfg);

  /*DO NOT DELETE THIS. This section is for unconstrained planning.

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
  //ConvertToCfgSample(_cfg, rvSample); */

  //sample a random end effector point in RV
  std::vector<double> endEffectorPoint = rv.GetRandomPoint();
  std::vector<double> efCenter(3, 0); //point to constrain end effector on

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

  //generate samples for the internal joints
  SampleInternal(chainCfg, jointPlacements, center, constraintCenter, dim);

  //make the RV sample
  std::vector<Vector3d> rvSample =
    ConstructRVSample(center, chainCfg, jointPlacements, constraintCenter);

  //convert RV sample to C-Space sample with random translational and orientation
  ConvertToCfgSample(_cfg, rvSample);
  _cfg.SetLinearPosition(center);
  _cfg.ConfigureRobot();
  auto mb = _cfg.GetMultiBody();
  Vector3d eePoint = mb->GetBodies().back().GetWorldTransformation().translation();
  Vector3d basePoint = mb->GetBodies().front().GetWorldTransformation().translation();
  cout << "EE point: " << eePoint << endl;
  cout << "matches: " << (rvSample.back() == constraintCenter) << endl;
  for(size_t i = 0; i < rvSample.size(); ++i){
    cout << "Point " << i << " in rv: " << rvSample[i] << endl;
  }
  if (eePoint == rvSample.back()){
    cout << "ee matches: ";
    cout << (rvSample.back() - eePoint).norm() << " units apart" << endl;
  }
  if (basePoint != rvSample.front())
    cout << "back doesn't match" << endl;

  cout << "dist between two rv samples: ";
  for(size_t i = 1; i < rvSample.size(); ++i){
      cout << (rvSample[i] - rvSample[i - 1]).norm() << " ";
  }
  cout << endl;

  if(this->m_debug)
    cout << "Cfg generated: " << _cfg.PrettyPrint() << endl;

  //_result.push_back(_cfg);

  if(vcm->IsValid(_cfg, callee))
    _result.push_back(_cfg);
  else
    _collision.push_back(_cfg);

  return true;

}

template <typename MPTraits>
bool
ReachableVolumeSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _endEffectorConstraint,
	const Boundary* const _boundary,
	std::vector<CfgType>& _result, std::vector<CfgType>& _collision) {

  //initialize variables
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vcm = this->GetValidityChecker(m_vcLabel);
  int dim = this->GetEnvironment()->GetBoundary()->GetDimension();

  MultiBody* multibody1 = _cfg.GetMultiBody(); //get multibody of cfg

  //assume joints are ordered consecutively based on index
  const size_t numJoints = multibody1->GetJoints().size();
  std::deque<Connection*> joints(numJoints, nullptr); //initialize empty joint list
  for(size_t i = 0; i < numJoints; ++i) {
    joints[i] = (multibody1->GetJoints()[i]).get(); //populate joint list
  }

  //if(Chain::Decompose(multibody1).size() != 1)
    //throw RunTimeException(WHERE) << "Only chain robots are supported right now.";
  //create a chain that corresponds to the cfg
  Chain chainCfg(multibody1, std::move(joints), multibody1->GetBase(),
		  multibody1->GetBody(multibody1->GetNumBodies() - 1), true);
  std::vector<JointPlacement> jointPlacements;
  //set origin as the reference point forparent RV
  WorkspaceBoundingSphericalShell rv =
    ComputeReachableVolume(dim, std::vector<double>(dim, 0.), chainCfg);

  //sample a point forthe end effector in its reachable volume
  std::vector<double> endEffectorPoint = rv.GetRandomPoint();
  std::vector<double> efCenter = _endEffectorConstraint->GetCenter();

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

  //generate samples forthe internal joints
  SampleInternal(chainCfg, jointPlacements, center, constraintCenter, dim);

  //make the RV sample
  std::vector<Vector3d> rvSample =
    ConstructRVSample(center, chainCfg, jointPlacements, constraintCenter);

  //convert RV sample to C-Space sample with random translational and orientation
  ConvertToCfgSample(_cfg, rvSample);

  if(this->m_debug)
    cout << "Cfg generated: " << _cfg.PrettyPrint() << endl;

  //_result.push_back(_cfg);

  if(vcm->IsValid(_cfg, callee))
    _result.push_back(_cfg);
  else
    _collision.push_back(_cfg);

  return true;

}


/*----------------------------------------------------------------------------*/

#endif
