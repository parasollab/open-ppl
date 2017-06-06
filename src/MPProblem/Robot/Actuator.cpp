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
Actuator(Robot* const _r, const std::string& _label, const DynamicsType _t)
    : m_robot(_r), m_label(_label), m_type(_t),
    m_controlSpace(m_robot->GetMultiBody()->DOF())
{
  const size_t dof = _r->GetMultiBody()->DOF();
  m_mask.resize(dof, false);
  m_limits.resize(dof, Range<double>(0, 0));
  m_maxForce = std::numeric_limits<double>::max();

  ComputeControlSpace();
}


Actuator::
Actuator(Robot* const _r, XMLNode& _node) : m_robot(_r),
    m_controlSpace(m_robot->GetMultiBody()->DOF())
{
  const size_t dof = _r->GetMultiBody()->DOF();

  // Get the label
  m_label = _node.Read("label", true, "", "Label of this actuator");

  // The label must not be called 'coast' as this is reserved for parsing
  // enumerated control sets.
  if(m_label == "coast")
    throw ParseException(_node.Where(), "Actuator cannot be labeled 'coast' "
        "because this name is reserved for parsing enumerated control sets.");

  // Read the force limits.
  const std::string limitString = _node.Read("limits", true, "", "Force limits "
      "of this actuator");
  std::istringstream limits(limitString);
  Range<double> temp;
  while(limits >> temp)
    m_limits.push_back(temp);

  // Assert that we read a limit for each DOF.
  if(m_limits.size() != dof)
    throw ParseException(_node.Where(), "Read a different number of force "
        "limits (" + std::to_string(m_limits.size()) + ") than DOFs ("
        + std::to_string(dof) + "). Each actuator must have limits for all DOFs."
        " To indicate no effectiveness in that DOF, set the limits to 0.");

  // Check that each limit's minimum is <= its maximum.
  for(const auto& limit : m_limits)
    if(limit.min > limit.max)
      throw ParseException(_node.Where(), "The minimum force limit cannot "
          "exceed the maximum force limit.");

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

  ComputeControlSpace();
}

/*-------------------------- Actuator Properties -----------------------------*/

void
Actuator::
SetLimits(const std::vector<double>& _min, const std::vector<double>& _max) {
  if(_min.size() != m_limits.size() or _max.size() != m_limits.size())
    throw RunTimeException(WHERE, "Actuator limits vector must have exactly one"
        " element for each DOF. Robot has " + std::to_string(m_limits.size()) +
        " DOFs, but new limits have " + std::to_string(_min.size()) + ", " +
        std::to_string(_max.size()) + " DOFs.");

  for(size_t i = 0; i < m_limits.size(); ++i) {
    m_limits[i] = Range<double>(_min[i], _max[i]);
    m_mask[i] = m_limits[i].Length() != 0 or m_limits[i].Center() != 0;
  }

  ComputeControlSpace();
}


void
Actuator::
SetMaxForce(const double _total) {
  m_maxForce = _total;
}


std::string
Actuator::
GetLabel() const {
  return m_label;
}


Robot*
Actuator::
GetRobot() const {
  return m_robot;
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
    const auto limit = _s[i] > 0 ? m_limits[i].max : m_limits[i].min ;
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


Control::Signal
Actuator::
ComputeNearestSignal(const std::vector<double>& _force) const {
  // Initialize an empty control signal.
  Control::Signal signal(_force.size(), 0);

  // Convert the requested force to control space representation.
  double largestSignal = 0;
  for(size_t i = 0; i < _force.size(); ++i) {
    const auto limit = _force[i] > 0 ? m_limits[i].max : m_limits[i].min ;
    signal[i] = std::abs(_force[i]) / limit;

    largestSignal = std::max(largestSignal, std::abs(signal[i]));
  }

  // If the largest signal exceeds 1, then we must re-scale our controls to fit
  // within our space.
  if(largestSignal > 1)
    for(auto& value : signal)
      value /= largestSignal;

  // If control lies in this actuator's control space, return it. Otherwise,
  // return the clearance point.
  if(m_controlSpace.Contains(signal))
    return signal;
  else
    return m_controlSpace.ClearancePoint(signal);
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
  std::vector<double> force = ComputeForce(_s);
  m_robot->GetDynamicsModel()->LocalToWorld(force, _model);

#if 0
  // Debug check for the applied force in the world frame.
  std::cout << "Checking execution force: ";
  for(const auto& val : force)
    std::cout << val << " ";
  std::cout << std::endl;
#endif

  auto iter = force.begin();
  auto mb = m_robot->GetMultiBody();

  btVector3 scratch(0, 0, 0);

  // Set base force.
  const size_t numPos = mb->PosDOF();
  for(size_t i = 0; i < numPos; ++i, ++iter)
    scratch[i] = *iter;

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
      for(size_t i = 0; i < 2; ++i, ++iter)
        scratch[i] = *iter;
      break;
    default:
      // This robot does not rotate.
      ;
  }

  switch(m_type) {
    case Force:
      _model->addBaseTorque(scratch);
      break;
    case Velocity:
      _model->setBaseOmega(scratch);
      break;
    default:;
  }

  // Finally, add forces/velocities to the joints.
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

/*-------------------------------- Helpers -----------------------------------*/

void
Actuator::
ComputeControlSpace() {
  // Set the control range for each DOF.
  for(size_t i = 0; i < m_limits.size(); ++i) {
    double min = 0, max = 0;

    // If the actuator can affect this DOF, compute the appropriate signal
    // range.
    if(m_mask[i]) {
      // If the minimum and maximum force have the same sign, the control space
      // is unidirectional (either forward or reverse only). Compute the larger
      // limit as 1 and the smaller one as the appropriate fraction.
      if(min > 0 || max < 0) {
        const double largestForce = std::max(std::abs(m_limits[i].min),
                                             std::abs(m_limits[i].max));
        min = m_limits[i].min / largestForce;
        max = m_limits[i].max / largestForce;
      }
      // Otherwise, the minimum and maximum forces are in opposite directions.
      // Both control limits are then 1.
      else {
        min = -1;
        max = 1;
      }
    }

    m_controlSpace.SetRange(i, min, max);
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
  _os << "\n\tControl space: " << _a.m_controlSpace;
  return _os << std::endl;
}

/*----------------------------------------------------------------------------*/
