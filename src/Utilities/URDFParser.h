#ifndef _PPL_URDF_PARSING_H_
#define _PPL_URDF_PARSING_H_

//#include "/opt/ros/melodic/include/urdf/model.h"
#include <urdf/model.h>

urdf::Model
ParseURDF(const std::string _filename);

#endif
