#include "Geometry/Bodies/Chain.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphericalShell.h"
#include "MPProblem/Robot/Robot.h"

#include "ReachableVolumes.h"
#include "Transformation.h"

#include <queue>


/// Compute the unconstrained reachable distance of a link relative to a direct
/// parent in the forward or reverse direction.
/// @param _dimension The workspace dimension.
/// @param _joint The link to compute the RV for.
/// @param _parent The parent of link to use.
/// @param _forward Is this a forward or backward traversal?
/// @return The reachable volume of _link relative to _parent.
double
ComputeReachableVolumeOfSingleLink(const size_t _dimension,
     const Connection* _frontJoint, const Connection* _backJoint, const Chain* _chain) {

  //cout << "in rv single link..." << endl;
  // Check that the joint type is supported.
  const Connection::JointType frontJointType = _frontJoint->GetConnectionType();
  //cout << "after type" << endl;
  switch(frontJointType) {
    case Connection::JointType::Revolute:
      {
        if(_dimension == 3)
          throw RunTimeException(WHERE) << "Revolute joints are not supported "
                                        << "for 3D workspaces (requires "
                                        << "Directed Reachable Volumes).";
        break;
      }
    case Connection::JointType::Spherical:
      {
        if(_dimension == 2)
          throw RunTimeException(WHERE) << "Spherical joints are not supported "
                                        << "for 2D workspaces.";
        break;
      }
    default:
      {
        /// @todo We can probably support some non-actuated joints here, but
        ///       need to verify with the theory.
        throw RunTimeException(WHERE) << "Unsupported joint type '" << frontJointType
                                      << "' connecting bodies "
                                      << _frontJoint->GetPreviousBodyIndex() << " and "
                                      << _frontJoint->GetNextBodyIndex() << ".";
      }
  }

  // Make sure the joint does not have a translational offset in the DH frame.
  //cout << "before dh" << endl;
  const auto& dh = _frontJoint->GetDHParameters();
  //cout << "after dh" << endl;

  if(dh.m_a != 0 or dh.m_d != 0)
    throw RunTimeException(WHERE) << "Reachable volumes do not handle joints "
                                  << "with offsets in the DH params. "
                                  << "Connection between bodies "
                                  << _frontJoint->GetNextBodyIndex() << " and "
                                  << _frontJoint->GetPreviousBodyIndex() << " has a = " << dh.m_a
                                  << ", d = " << dh.m_d << ".";

  // Get the distance from one joint to the next. If m_lastBody is valid then use that as the next (and last) joint
  //the min RD is the max RD in a single link
  double dist;

  //this segment is needed because when the direction of chain changes, 
  //the next and previous bodies that connection store to do not.
  bool chain_dir = _chain->IsForward();
  const Body* nextBody = chain_dir ? _frontJoint->GetNextBody() : _frontJoint->GetPreviousBody();

  if (nextBody == _chain->GetLastBody()){
    //handle joint to end effector
    //cout << "in if" << endl;
    dist =  _frontJoint->GetTransformationToDHFrame().translation().norm() +
	    _backJoint->GetTransformationToBody2().translation().norm() +
            _frontJoint->GetTransformationToBody2().translation().norm();
  }
  else {
    //handle joint to joint
    //cout << "in else" << endl;
    dist =  _frontJoint->GetTransformationToDHFrame().translation().norm() +
	    _backJoint->GetTransformationToBody2().translation().norm();
  }


  return dist;
}


WorkspaceBoundingSphericalShell
ComputeReachableVolume(const size_t _dimension, const vector<double>& _center, const Chain& _chain) {
  // Initialize minimum and maximum reachable distances.
  double min = 0,
         max = 0;

  //computes the base to the first joint
  if (_chain.GetBase()) {
    max += (*(_chain.begin()))->GetTransformationToDHFrame().translation().norm();
    min = max;
    //cout << "GetBase(): " << max << endl;
  }

  //compute RV of internal joint
  for(auto iter1 = _chain.begin(), iter2 = _chain.begin() + 1; 
      iter2 != _chain.end(); ++iter1, ++iter2) {
    //start from first joint and stop when front joint is last

    // Compute the unconstrained reachable distance of the next link
    const double rd = ComputeReachableVolumeOfSingleLink(
        _dimension, *iter2, *iter1, &_chain);

    //cout << "updating rv..." << endl;
    // Update the chain's minimum and maximum reachable distance.
    min = std::max(0., (min > rd) ? min - rd
				  : rd - max);
    max += rd;
  }

 // cout << "max: " << max << endl;
 // cout << "min: " << min << endl;
 // cout << "returning reachable volume..." << endl;
  return WorkspaceBoundingSphericalShell(_center, max, min);
}

/*----------------------------------------------------------------------------*/
