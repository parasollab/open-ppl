#include "Robot.h"
#include "Geometry/Bodies/MultiBody.h"

void
Robot::
ReadURDF(const std::string& _filename, std::string _worldLink) {
  // Convert the urdf model to multibody representation.
  m_multibody = std::unique_ptr<MultiBody>(new MultiBody(MultiBody::Type::Active));
  m_multibody->TranslateURDF(_filename, _worldLink, m_fixed);

  // Initialize the DOF limits and set the robot to a zero starting configuration.
  InitializePlanningSpaces();
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));
  if(m_fixed) {
    m_multibody->GetBase()->Configure(m_multibody->GenerateBaseTransformation(m_basePosition,true));
  }
}
