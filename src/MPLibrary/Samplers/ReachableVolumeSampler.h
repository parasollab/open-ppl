#ifndef RV_SAMPLER_H
#define RV_SAMPLER_H

#include "CGAL/Exact_spherical_kernel_3.h"
#include "CGAL/Object.h"
#include "CGAL/Circle_3.h"
//#include "CGAL/intersections.h"
#include "CGAL/Spherical_kernel_intersections.h"

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
typedef CGAL::Exact_spherical_kernel_3         Spherical_k;
typedef CGAL::Point_3<Spherical_k>             Point_3;
typedef CGAL::Sphere_3<Spherical_k>            Sphere_3;
typedef CGAL::Circle_3<Spherical_k>            Circle_3;

template <typename MPTraits>
class ReachableVolumeSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    //typedef std::deque<const Connection*> JointList;

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
        vector<CfgType>& _result, vector<CfgType>& _collision) override;


    void SampleInternal(Chain& _chain, vector<std::pair<Connection, Vector3d > >& _jointPlacements, 
			Vector3d& _center, Vector3d& _endEffectorPoint, const size_t _dim);

    ///@}

    ///@name Helper functions
    ///@{

    //makes the RV samples given the joint placements of the joints in RV-space
    vector<Vector3d> ConstructRVSample(const Vector3d _center, const Chain& _chain,
					      const vector<std::pair<Connection, Vector3d > > _jointPlacements,
					      const Vector3d _endEffectorPoint); 
    //Finds the position of the joint in RV-space if it is already sample; throws error if joint cannot be found
    Vector3d FindJointPosition(vector<std::pair<Connection, Vector3d > >& _points, const Connection& _joint);
    //Compute euclidean distance between two vectors; this might already be implemented somewhere else but idk
    double ComputeDistance(Vector3d _a, Vector3d _b);

    //Compute the Vector3D of
    Vector3d ComputeVector(Vector3d _a, Vector3d _b);
    bool isSingleLink(Chain& _chain);
    Vector3d ComputePointonIntersection(std::pair<Vector3d, double> sphere1, std::pair<Vector3d, double> sphere2);
    void ConvertToCfgSample(CfgType& _cfg, vector<Vector3d > _rvPoints);

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

//template <typename MPTraits>
//Vector3d
//ReachableVolumeSampler<MPTraits>::
//ComputeVector(Vector3d _a, Vector3d _b) {
//  //substracts two Vector3ds and creates a vector3d
//  Vector3d out;
//  for (int i = 0; i < _a.size(); ++i){
//    out[i] = _b[i] - _a[i];
//  }
//  return out;
//}

//template <typename MPTraits>
//double
//ReachableVolumeSampler<MPTraits>::
//ComputeDistance(Vector3d _a, Vector3d _b) {
//  //computes the distance between two vectors
//  double sum = 0;
//  for (int i = 0; i < _a.size(); i++) {
//      sum += pow(_a[i] - _b[i], 2);
//  }
//  return pow(sum, (1/2.0));
//}

template <typename MPTraits>
void
ReachableVolumeSampler<MPTraits>::
ConvertToCfgSample(CfgType& _cfg, vector<Vector3d> _rvPoints) {
  //TODO: need to find a way to orient the joint angles.
  //Since finding the articulated and spherical joint angles
  //both involve using acos(theta) we don't know which quadrant
  //the angle is suppose to lie in, or how we determine the quadrants
  //in the first place. This is fundamental.

  //takes the consecutive points in RV space and computes the joint angles
  vector<double> jointData;
  //compute articulated angle for every joint
  for (auto iter1 = _rvPoints.begin(), iter2 = _rvPoints.begin() + 1, iter3 = _rvPoints.begin() + 2; 
	  iter3 != _rvPoints.end(); iter1++, iter2++, iter3++ ) {
    //compute length of the sides of triangle
    double a = (*iter2 - *iter1).norm();
    double b = (*iter3 - *iter2).norm();
    double c = (*iter3 - *iter1).norm();

    //use law of cosine to find joint angles and normalize it to be from 0 to 1
    double jointAngle1 = acos((pow(c, 2) - pow(a, 2) - pow(b, 2)) / (-2 * a * b)) / M_PI;
    jointData.push_back(jointAngle1); 
  }

  //compute rotational angle of first link 
  Vector3d vector1(_rvPoints[0] - _rvPoints[1]);
  Vector3d vector2(_rvPoints[2] - _rvPoints[1]);
  Vector3d orthogonal = vector1 % vector2;

  //angle between orthogonal vector and upward vector
  Vector3d upward;
  upward[0] = 0;
  upward[1] = 1;
  upward[2] = 0;

  //compute angle betweem upward and orthogonal
  double jointAngle2 = acos((upward * orthogonal) / (upward.norm() * orthogonal.norm()));
  jointData.insert(jointData.begin() + 1, jointAngle2);
  
  Vector3d old_orth = orthogonal;
  //compute rotational angle for remaining links
  int i = 2;
  for (auto iter1 = _rvPoints.begin() + 1, 
            iter2 = _rvPoints.begin() + 2, 
	    iter3 = _rvPoints.begin() + 3; 
	  iter2 != _rvPoints.end() - 1; iter1++, iter2++, iter3++) {
    //compute length of the sides of triangle
    vector1 = *iter1 - *iter2;
    vector2 = *iter3 - *iter2;
    orthogonal = vector1 % vector2;
    jointAngle2 = acos((old_orth * orthogonal) / (old_orth.norm() * orthogonal.norm()));
    //use law of cosine to find joint angles
    jointData.insert(jointData.begin() + i, jointAngle2); 
    i += 2;
  }

  _cfg.SetJointData(jointData);
}

template <typename MPTraits>
Vector3d
ReachableVolumeSampler<MPTraits>::
ComputePointonIntersection(std::pair<Vector3d, double> sphere1, 
			  std::pair<Vector3d, double> sphere2) {
  //calculate the center and radius of intersection circle
    Vector3d center1 = sphere1.first;
    Vector3d center2 = sphere2.first;

    double radius1 = sphere1.second;
    double radius2 = sphere2.second;

    //get distance between centers
    double d = (center2 - center1).norm();
    //get center of the intersecting circle
    double h = (1/2.0) + (std::pow(radius1, 2) - std::pow(radius2, 2)) /
		(2 * std::pow(d, 2));

    double r_i = std::pow(std::pow(radius1, 2) - std::pow(h, 2) * std::pow(d, 2), (1/2.0));
    Vector3d c_i = center1 + h * (center2 - center1); //compute center of intersect circle

  //calculate point on intersecting circle
    Vector3d axis;
    axis[0] = 0;
    axis[1] = 1;
    axis[2] = 0;
    Vector3d point;
    Vector3d n_i = (center2 - center1) / d;
    Vector3d t_i = (axis % n_i).normalize();
    Vector3d b_i = t_i % n_i; //compute perp of tangent and bitangent

  //compute random point on this circle
    double rad = ((double) rand() / RAND_MAX) * (2.0 * M_PI);
    point = c_i + (r_i * (t_i * cos(rad) + b_i * sin(rad)));
      
  return point;
}

template <typename MPTraits>
vector<Vector3d>
ReachableVolumeSampler<MPTraits>::
ConstructRVSample(const Vector3d _center, 
		  const Chain& _chain, 
		  const vector<std::pair<Connection, Vector3d > > _jointPlacements, 
		  const Vector3d _endEffectorPoint) {
  //constructs the rvSample by making a vector of points corresponding to joints on the chain
  vector<Vector3d > rvSample;
  rvSample.push_back(_center);
  for (auto iter = _chain.begin(); iter != _chain.end(); ++iter) {
      for (auto jointIter = _jointPlacements.begin(); jointIter != _jointPlacements.end(); ++jointIter){
	  if (**iter == jointIter->first) {
	     rvSample.push_back(jointIter->second); 
	  }
      }
  }
  rvSample.push_back(_endEffectorPoint);
  return rvSample;
}

template <typename MPTraits>
bool
ReachableVolumeSampler<MPTraits>::
isSingleLink(Chain& _chain) {
  //check for all single link cases
  Body* lastBody = _chain.GetLastBody();
  Body* base = _chain.GetBase();
  int size = _chain.Size(); //number of connection types stored in m_joints
  return (size == 2 && !base && !lastBody) ||
	  (size == 1 && lastBody && !base) ||
	  (size == 1 && !lastBody && base);
       
}

template <typename MPTraits>
Vector3d
ReachableVolumeSampler<MPTraits>::
FindJointPosition(vector<std::pair<Connection, Vector3d > >& _points, const Connection& _joint) {
    for (int i = 0; i < _points.size(); ++i) {
      if (_points[i].first == _joint)
	return _points[i].second;
    }
    throw RunTimeException(WHERE) << "Joint not found in the list of sampled points";
}

/*------------------------------ Sampler Rule --------------------------------*/

template <typename MPTraits>
void
ReachableVolumeSampler<MPTraits>::
SampleInternal(Chain& _chain, vector<std::pair<Connection, Vector3d > >& _jointPlacements, 
		Vector3d& _center, Vector3d& _endEffectorPoint, const size_t _dim) {
  //TODO: adapt this code to work with the given dimensions; right now only
  //3 dimensions is supported

  //check if chain is just one link
  if (isSingleLink(_chain))
    return;

  std::pair <Chain, Chain> separated_chain = _chain.Bisect();
  Chain cl = separated_chain.first;
  Chain cr = separated_chain.second;
  if (this->m_debug) {
    cout << "left chain size: " << cl.Size() <<  endl;
    cout << "right chain size: " << cr.Size() <<  endl;
  }

  //make sure the chains are facing each other
  cl.IsForward() ? cl : cl.Reverse();
  cr.IsForward() ? cr.Reverse() : cr;

  //find the points to build rv relative to (the base of the chains)
  Vector3d leftPoint = cl.GetBase() ? _center : FindJointPosition(_jointPlacements, **(cl.begin()));
  Vector3d rightPoint = cr.GetBase() ? _endEffectorPoint : FindJointPosition(_jointPlacements, **(cr.begin()));

  vector<double> p_l(_dim);
  vector<double> p_r(_dim);

  for (int i = 0; i < _dim; ++i) {
    p_l[i] = leftPoint[i];
    p_r[i] = rightPoint[i];
  }

  //compute reachable volumes for both partitions
  WorkspaceBoundingSphericalShell rvl = ComputeReachableVolume(_dim, p_l, cl);
  WorkspaceBoundingSphericalShell rvr = ComputeReachableVolume(_dim, p_r, cr);

  if (this->m_debug) {
    cout << "Outer radius left rv: " << rvl.GetOuterRadius() << endl;
    cout << "Length of left rv: " << rvl.GetOuterRadius() - rvl.GetInnerRadius() << endl;
    cout << "Outer radius right rv: " << rvr.GetOuterRadius() << endl;
    cout << "Length of right rv: " << rvr.GetOuterRadius() - rvr.GetInnerRadius() << endl;
  }


  //if the spherical shell has length 0 then do this
  Vector3d jointSample;
  //TODO: this style of sampling is temporary and
  //is not efficient. We need a more precise way to
  //sample in intersections
  if (rvr.GetOuterRadius() == rvr.GetInnerRadius()) {
    //compute the intersection and sample on that intersection
    double radius1 = rvl.GetOuterRadius();
    double radius2 = rvr.GetOuterRadius();
    vector<double> center1 = rvl.GetCenter();
    vector<double> center2 = rvr.GetCenter();

    Vector3d c_1;
    c_1[0] = center1[0];
    c_1[1] = center1[1];
    c_1[2] = center1[2];

    Vector3d c_2;
    c_2[0] = center2[0];
    c_2[1] = center2[1];
    c_2[2] = center2[2];

    std::pair<Vector3d, double> sphere1 = make_pair(c_1, radius1);
    std::pair<Vector3d, double> sphere2 = make_pair(c_2, radius2);

    Vector3d point = ComputePointonIntersection(sphere1, sphere2);

    if (this->m_debug)
      cout << "intersecting point (circle) : " << point << "\n" << endl;

    jointSample[0] = point[0]; 
    jointSample[1] = point[1]; 
    jointSample[2] = point[2]; 


  }
   else {
     do {
       vector<double> jSample = rvl.GetRandomPoint();
       for (int i = 0; i < jSample.size(); ++i)
	 jointSample[i] = jSample[i];
     }
     while (!rvr.InBoundary(jointSample));
  }

  //record where the last joint goes
  _jointPlacements.push_back(std::make_pair(*(cl.GetEnd()), jointSample));
  
  SampleInternal(cl, _jointPlacements, _center, _endEffectorPoint, _dim);
  SampleInternal(cr, _jointPlacements, _center, _endEffectorPoint, _dim);
}

template <typename MPTraits>
bool
ReachableVolumeSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  //initialize variables
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vcm = this->GetValidityChecker(m_vcLabel);
  int dim = this->GetEnvironment()->GetBoundary()->GetDimension();

  //TODO: i declare this twice between some functions take const args
  //and some don't, I didn't have time to make everything consistent.
  const MultiBody* cfg_multibody = _cfg.GetMultiBody(); //get const multibody of cfg
  MultiBody* multibody1 = _cfg.GetMultiBody(); //get multibody of cfg

  //assume joints are ordered consecutively based on index
  int numJoints = cfg_multibody->GetJoints().size();
  std::deque<Connection*> joints(numJoints, nullptr); //initialize empty joint list
  for (int i = 0; i < numJoints; ++i) {
    joints[i] = (cfg_multibody->GetJoints()[i]).get(); //populate joint list
  }

  Chain chain_cfg(cfg_multibody, std::move(joints), multibody1->GetBase(), 
		  multibody1->GetBody(cfg_multibody->GetNumBodies() - 1), true); //create a chain that corresponds to the cfg

  vector<std::pair<Connection, Vector3d > > jointPlacements;
  WorkspaceBoundingSphericalShell rv = ComputeReachableVolume(dim, vector<double>(dim, 0.), chain_cfg); //set origin as the reference point for parent RV

  //sample a point for the end effector in its reachable volume
  vector<double> endEffectorPoint = rv.GetRandomPoint();
  vector<double> center = rv.GetCenter();

  Vector3d p_end;
  for (int i = 0; i < dim; ++i)
    p_end[i] = endEffectorPoint[i];

  Vector3d p_center;
  for (int i = 0; i < dim; ++i)
    p_center[i] = center[i];

  //generate samples for the internal joints
  SampleInternal(chain_cfg, jointPlacements, p_center, p_end, dim);

  //make the RV sample
  vector<Vector3d> rvSample = ConstructRVSample(p_center, chain_cfg, jointPlacements, p_end);

  //convert RV sample to C-Space sample with random translational and orientation
  ConvertToCfgSample(_cfg, rvSample);

  if (this->m_debug)
    cout << "Cfg generated: " << _cfg << endl;

  //check if the sample created was valid
  _result.push_back(_cfg);

  //if (vcm->IsValid(_cfg, callee))
  //  _result.push_back(_cfg);
  //else
  //  _collision.push_back(_cfg);

  return true;

}

/*----------------------------------------------------------------------------*/

#endif
