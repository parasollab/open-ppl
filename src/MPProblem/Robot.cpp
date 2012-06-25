#include "Robot.h"
#include <iostream>
#include <cstdlib>

Robot::Robot(Base _base, BaseMovement _baseMovement, 
    JointMap _joints, int _bodyIndex) : 
  m_base(_base), m_baseMovement(_baseMovement), 
  m_joints(_joints), m_bodyIndex(_bodyIndex) {
  }

Robot::Base Robot::GetBaseFromTag(const string _tag){
  if(_tag == "PLANAR")
    return Robot::PLANAR;
  else if(_tag == "VOLUMETRIC")
    return Robot::VOLUMETRIC;
  else if(_tag == "FIXED")
    return Robot::FIXED;
  else if(_tag == "JOINT")
    return Robot::JOINT;
  else{
    cerr << "Error::Incorrect Base Type Specified::" << _tag << endl;
    cerr << "Choices are:: Planar, Volumetric, Fixed, or Joint" << endl;
    exit(1);
  }
}

Robot::BaseMovement Robot::GetMovementFromTag(const string _tag){
  if(_tag == "ROTATIONAL")
    return Robot::ROTATIONAL;
  else if (_tag == "TRANSLATIONAL")
    return Robot::TRANSLATIONAL;
  else {
    cerr << "Error::Incorrect Movement Type Specified::" << _tag << endl;
    cerr << "Choices are:: Rotational or Translational" << endl;
    exit(1);
  }
}

Robot::JointType Robot::GetJointTypeFromTag(const string _tag){
  if(_tag == "REVOLUTE")
    return Robot::REVOLUTE;
  else if (_tag == "SPHERICAL")
    return Robot::SPHERICAL;
  else {
    cerr << "Error::Incorrect Joint Type Specified::" << _tag << endl;
    cerr << "Choices are:: Revolute or Spherical" << endl;
    exit(1);
  }
}

//Begin section getTag

string
Robot::GetTagFromBase(const Robot::Base& _b){
  switch(_b){
    case PLANAR:
      return "PLANAR";
    case VOLUMETRIC:
      return "VOLUMETRIC";
    case FIXED:
      return "FIXED";
    case JOINT:
      return "JOINT";
    default:
      return "Unknown Base Type";
  }
}

string
Robot::GetTagFromMovement(const Robot::BaseMovement& _bm){
  switch(_bm){
    case ROTATIONAL:
      return "ROTATIONAL";
    case TRANSLATIONAL:
      return "TRANSLATIONAL";
    default:
      return "Unknown Base Movement";
  }
}

string
Robot::GetTagFromJointType(const Robot::JointType& _jt){
  switch(_jt){
    case REVOLUTE:
      return "REVOLUTE";
    case SPHERICAL:
      return "SPHERICAL";
    default:
      return "Unknown Joint Type";
  }
} 
