#include "Geometry/Bodies/Chain.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphericalShell.h"
#include "MPProblem/Robot/Robot.h"

#include "Transformation.h"

#include <queue>


/// Compute the unconstrained reachable distance of a link relative to a direct
/// parent in the forward or reverse direction.
/// @param _dimension The workspace dimension.
/// @param _body The link to compute the RV for.
/// @param _parent The parent of link to use.
/// @param _forward Is this a forward or backward traversal?
/// @return The reachable volume of _link relative to _parent.
std::pair<double, double>
ComputeReachableDistanceRelativeToParent(const size_t _dimension,
    const Body* const _body, const Body* const _parent, const bool _forward) {
  // If the body has no parent, its unconstrained RV is the origin in its local
  // frame.
  if(_parent == nullptr)
    return {0., 0.};

  // Get the connection from _parent to _body.
  const Connection* const joint = _body->GetConnectionTo(_parent);
  if(!joint)
    throw RunTimeException(WHERE) << "Could not find connection from body "
                                  << _body->GetIndex() << " to parent "
                                  << _parent->GetIndex() << ".";

  // Check that the joint type is supported.
  const Connection::JointType type = joint->GetConnectionType();
  switch(type) {
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
        throw RunTimeException(WHERE) << "Unsupported joint type '" << type
                                      << "' connecting bodies "
                                      << joint->GetPreviousBodyIndex() << " and "
                                      << joint->GetNextBodyIndex() << ".";
      }
  }

  // Make sure the joint does not have a translational offset in the DH frame.
  const auto& dh = joint->GetDHParameters();
  if(dh.m_a != 0 or dh.m_d != 0)
    throw RunTimeException(WHERE) << "Reachable volumes do not handle joints "
                                  << "with offsets in the DH params. "
                                  << "Connection between bodies "
                                  << _body->GetIndex() << " and "
                                  << _parent->GetIndex() << " has a = " << dh.m_a
                                  << ", d = " << dh.m_d << ".";

  // Get the total offset distance for this body relative to its parent (along
  // each linkage).
  double toJoint = joint->GetTransformationToDHFrame().translation().norm(),
         fromJoint = joint->GetTransformationToBody2().translation().norm();

  // If this is not a forward traversal, the distances are reversed.
  if(!_forward)
    std::swap(toJoint, fromJoint);

  // Determine the minimum and maximum reachable distances of _body from _parent.
  const double minimumRD = std::max(toJoint - fromJoint, 0.),
               maximumRD = toJoint + fromJoint;

  return {minimumRD, maximumRD};
}


WorkspaceBoundingSphericalShell
ComputeReachableVolume(const size_t _dimension, const Chain& _chain) {
  // Initialize minimum and maximum reachable distances.
  double min = 0,
         max = 0;

  for(auto iter1 = _chain.begin(), iter2 = _chain.begin() + 1;
      iter2 != _chain.end(); ++iter1, ++iter2) {
    const Body* const body   = *iter2,
              * const parent = *iter1;

    // Compute the unconstrained reachable distance relative to the parent.
    const auto rd = ComputeReachableDistanceRelativeToParent(
        _dimension, body, parent, _chain.IsForward());

    // Update the chain's minimum and maximum reachable distance.
    min = std::max(0., (min > rd.first) ? min - rd.second
                                        : rd.first - max);
    max += rd.second;
  }

  return WorkspaceBoundingSphericalShell(_dimension, max, min);
}

/*----------------------------------------------------------------------------*/
