#include "Robot.h"

#include "Geometry/Boundaries/Boundary.h"

/// Temporary stand-in.
class Controller {};

/*------------------------------ Construction --------------------------------*/

Robot::
Robot(XMLNode& _node) {
  // Get the unique robot label.
  m_label = _node.Read("label", true, "", "Unique robot label");

  // Get the filename and make sure it exists.
  const std::string path = GetPathName(_node.Filename());
  const std::string file = _node.Read("filename", true, "", "Robot file name");
  const std::string filename = path + file;
  if(!FileExists(filename))
    throw ParseException(filename, "File does not exist");

  // Open the file.
  CountingStreamBuffer cbs(filename);
  std::istream ifs(&cbs);

  // Read the file.
  m_multibody.Read(ifs, cbs);
}


Robot::
~Robot() {
  delete m_controller;
}

/*--------------------------- Geometry Accessors -----------------------------*/

void
Robot::
SetBoundary(const Boundary* _b) {
  m_multibody.InitializeDOFs(_b);
}


ActiveMultiBody*
Robot::
GetMultiBody() {
  return &m_multibody;
}


const ActiveMultiBody*
Robot::
GetMultiBody() const {
  return &m_multibody;
}

/*--------------------------- Controller Accessors ---------------------------*/

Controller*
Robot::
GetController() {
  return m_controller;
}


void
Robot::
SetController(Controller* const _c) {
  delete m_controller;
  m_controller = _c;
}

/*----------------------------------------------------------------------------*/
