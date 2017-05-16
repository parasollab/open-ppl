#include "Actuator.h"

#include <algorithm>
#include <limits>
#include <sstream>
#include <string>

#include "DynamicsModel.h"
#include "Robot.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

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
Actuator(Robot* const _r, XMLNode& _node) : m_robot(_r) {
  const size_t dof = _r->GetMultiBody()->DOF();

  // Read the force limits.
  const std::string limitString = _node.Read("limits", true, "", "Force limits "
      "of this actuator");
  std::istringstream limits(limitString);
  Range<double> temp;
  while(limits >> temp)
    m_limits.push_back(temp);

  // Assert that we read a limit for each DOF.
  if(m_limits.size() != dof)
    throw ParseException(WHERE, "Read a different number of force limits (" +
        std::to_string(m_limits.size()) + ") than DOFs (" + std::to_string(dof) +
        ").");

  // Set the mask based on the force limits.
  for(const auto& limit : m_limits)
    m_mask.push_back(limit.Length() != 0 or limit.Center() != 0);

  // Read the maximum magnitude.
  m_maxForce = _node.Read("maxMagnitude", true, 0., 0.,
      std::numeric_limits<double>::max(), "The maximum total force that this "
      "actuator can produce");

  // Read the type.
  std::string type = _node.Read("type", false, "force", "The type of "
      "dynamics modeled {force, velocity}");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  if(type == "force")
    m_type = Force;
  else if(type == "velocity")
    m_type = Velocity;
  else
    throw ParseException(_node.Where(), "Unknown dynamics type '" + type + "'.");
}

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
  btVector3 scratch(0, 0, 0);

  // Set base force.
  const size_t numPos = mb->PosDOF();
  for(size_t i = 0; i < numPos; ++i, ++iter)
    scratch[i] = *iter;
  scratch = _model->localDirToWorld(-1, scratch);

  switch(m_type) {
    case Force:
      _model->addBaseForce(scratch);
      break;
    case Velocity:
      _model->setBaseVel(scratch);
      break;
    default:;
  }

  // Set base torque.
  scratch.setValue(0, 0, 0);
  switch(mb->OrientationDOF()) {
    case 1:
      // This is a planar rotational robot. We only want a torque in the Z
      // direction.
      scratch[2] = *iter++;
      break;
    case 3:
      // This is a volumetric rotational robot. We need all three torque
      // directions.
      for(size_t i = 0; i < 3; ++i, ++iter)
        scratch[i] = *iter;
      break;
    default:;
  }
  scratch = _model->localDirToWorld(-1, scratch);

  switch(m_type) {
    case Force:
      _model->addBaseTorque(scratch);
      break;
    case Velocity:
      _model->setBaseOmega(scratch);
      break;
    default:;
  }

  const size_t numJoints = mb->JointDOF();

  // Multiply by PI because bullet uses [ -PI : PI ].
  switch(m_type) {
    case Force:
      for(size_t i = 0; i < numJoints; ++i, ++iter)
        _model->addJointTorque(i, *iter * PI);
      break;
    case Velocity:
      for(size_t i = 0; i < numJoints; ++i, ++iter)
        _model->setJointVel(i, *iter * PI);
      break;
    default:;
  }
}

/*---------------------------------- Debug -----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Actuator& _a) {
  _os << "Actuator (robot " << _a.m_robot << "):"
      << "\n\tType: " << _a.m_type
      << "\n\tMaxForce: " << _a.m_maxForce
      << "\n\tMask: ";
  for(const auto& m : _a.m_mask)
    _os << " " << m;
  _os << "\n\tLimits:";
  for(const auto& limit : _a.m_limits)
    _os << " " << limit;
  return _os << std::endl;
}

/*----------------------------------------------------------------------------*/
