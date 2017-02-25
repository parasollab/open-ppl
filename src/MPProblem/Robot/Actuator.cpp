#include "Actuator.h"

#include <algorithm>
#include <limits>

#include "DynamicsModel.h"
#include "Robot.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
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

  for(size_t i = 0; i < m_limits.size(); ++i) {
    m_limits[i] = Range<double>(_min[i], _max[i]);
    if(_min[i] != 0 || _max[i] != 0)
      m_mask[i] = true;
  }
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
  /// @TODO Generalize this so that the robot's starting point can be taken into
  ///       account. Currently we assume that the generated force is independent
  ///       of starting state.
  const size_t dof = m_mask.size();

  // Compute the generalized force vector and total force.
  std::vector<double> force(dof, 0);
  double total = 0;
  for(size_t i = 0; i < dof; ++i) {
    auto limit = _s[i] > 0 ? m_limits[i].max : m_limits[i].min ;
    force[i] = std::abs(_s[i]) * limit;
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
  Execute(_s, m_robot->GetDynamicsModel()->Get());
}


void
Actuator::
Execute(const Control::Signal& _s, btMultiBody* const _model) const {
  auto f = ComputeForce(_s);
  auto iter = f.begin();

  auto mb = m_robot->GetMultiBody();
  const size_t numPos = mb->PosDOF();
  const size_t numJoints = mb->NumJoints();
  const size_t numOri = mb->DOF() - numJoints - numPos;
  btVector3 scratch(0, 0, 0);

  // Set base force.
  for(size_t i = 0; i < numPos; ++i, ++iter)
    scratch[i] = *iter;
  _model->worldDirToLocal(-1, scratch);
  _model->addBaseForce(scratch);

  // Set base torque.
  scratch.setValue(0, 0, 0);
  if(numOri == 1) {
    // This is a planar rotational robot. We only want a torque in the Z
    // direction.
    scratch[2] = *iter++;
  }
  else {
    // This is a volumetric rotational robot. We need all three torque
    // directions.
    for(size_t i = 0; i < numOri; ++i, ++iter)
      scratch[i] = *iter;
  }
  _model->worldDirToLocal(-1, scratch);
  _model->addBaseTorque(scratch);

  for(size_t i = 0; i < numJoints; ++i, ++iter)
    // Multiply by PI because bullet uses [ -PI : PI ].
    _model->addJointTorque(i, *iter * PI);
}

/*----------------------------------------------------------------------------*/
