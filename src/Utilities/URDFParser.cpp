#include "URDFParser.h"
#include "PMPLExceptions.h"

#define URDF

urdf::Model
ParseURDF(const std::string _filename) {
  urdf::Model model;
  if(!model.initFile(_filename)) {
    throw RunTimeException(WHERE) << "Unable to parse urdf filename: "
                                  << _filename 
                                  << std::endl;
  }
  return model;
}

