#include "Robot.h"
#include "Geometry/Bodies/MultiBody.h"

void
Robot::
ReadURDF(const std::string& _filename, std::string _worldLink) {
  // Convert the urdf model to multibody representation.
  m_multibody = std::unique_ptr<MultiBody>(new MultiBody(MultiBody::Type::Active));

  Body::Type bodyType;
  Body::MovementType movementType;
  if(m_fixed){
    bodyType = Body::Type::Fixed;
    movementType = Body::MovementType::Fixed;
  } else {
      bodyType = Body::Type::Planar;
      movementType = Body::MovementType::Rotational;
  }
  m_multibody->TranslateURDFFile(_filename, _worldLink, bodyType, movementType);

  // Initialize the DOF limits and set the robot to a zero starting configuration.
  InitializePlanningSpaces();
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));
  if(m_fixed) {
    m_multibody->GetBase()->Configure(m_multibody->GenerateBaseTransformation(m_basePosition,true));
  }
}
