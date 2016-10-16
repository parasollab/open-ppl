#include "Robot.h"

#include "Geometry/Boundaries/Boundary.h"

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

/*--------------------------- Geometry Accessors -----------------------------*/

void
Robot::
SetBoundary(const Boundary* _b) {
  m_multibody.InitializeDOFs(_b);
}

/*----------------------------------------------------------------------------*/
