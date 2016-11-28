#include "Actuator.h"

#include <algorithm>
#include <limits>

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/PMPLExceptions.h"

#include "BulletDynamics/Featherstone/btMultiBody.h"

/*------------------------------- Construction -------------------------------*/

Actuator::
Actuator(Robot* const _r) : m_robot(_r) {
  const size_t dof = _r->GetMultiBody()->DOF();
  m_mask.resize(dof, false);
  m_limits.resize(dof, Range<double>(0, 0));
  m_maxForce = std::numeric_limits<double>::max();
}


Actuator::
~Actuator() = default;

/*-------------------------- Actuator Properties -----------------------------*/

void
Actuator::
SetLimits(const std::vector<double>& _min, const std::vector<double>& _max) {
  if(_min.size() != m_limits.size() || _max.size() != m_limits.size())
    throw RunTimeException(WHERE, "Actuator limits vector must have exactly one"
        " element for each DOF! Robot has " + std::to_string(m_limits.size()) +
        " DOFs, but new limits have " + std::to_string(_min.size()) + ", " +
        std::to_string(_max.size()) + " DOFs.");

  for(size_t i = 0; i < m_limits.size(); ++i)
    m_limits[i] = Range<double>(_min[i], _max[i]);
}


void
Actuator::
SetMaxForce(const double _total) {
  m_maxForce = _total;
}

/*--------------------------- Planning Interface -----------------------------*/

std::vector<bool>
Actuator::
ControlMask() const {
  return m_mask;
}


std::vector<double>
Actuator::
ComputeForce(const Control::Signal& _s) const {
  const size_t dof = m_mask.size();

  // Compute the generalized force vector and total force.
  std::vector<double> force(dof, 0);
  double total = 0;
  for(size_t i = 0; i < dof; ++i) {
    auto limit = _s[i] > 0 ? m_limits[i].max : m_limits[i].min ;
    force[i] = _s[i] * limit;
    total += force[i] * force[i];
  }
  total = std::sqrt(total);

  // If the total force exceeds the actuator's limit, scale it back to what we
  // can produce.
  if(total > m_maxForce) {
    auto rescale = [this, total](double& _d) {_d *= this->m_maxForce / total;};
    std::for_each(force.begin(), force.end(), rescale);
  }

  return force;
}

/*-------------------------- Simulation Interface ----------------------------*/

void
Actuator::
Execute(const Control::Signal& _s) const {
  /// @TODO Generalize this so that the robot's starting point can be taken into
  ///       account. Currently we assume that the generated force is independent
  ///       of starting state.
  auto f = ComputeForce(_s);

  /// @TODO Generalize this to accomodate multi-link robots. We are assuming
  ///       robot is 6-dof rigid body for the moment
  auto model = m_robot->GetDynamicsModel();
  model->addBaseForce(btVector3(f[0], f[1], f[2]));
  model->addBaseTorque(btVector3(f[3], f[4], f[5]));
}

/*----------------------------------------------------------------------------*/
