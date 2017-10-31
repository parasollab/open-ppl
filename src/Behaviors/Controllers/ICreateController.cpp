#include "ICreateController.h"

#include <algorithm>

#include "ConfigurationSpace/Cfg.h"
#include "Utilities/XMLNode.h"

//TEMP
#include <unistd.h>
/*------------------------------ Construction --------------------------------*/

ICreateController::
ICreateController(Robot* const _r, const double _gain, const double _max)
  : ControllerMethod(_r), m_gain(_gain), m_max(_max) { }


ICreateController::
ICreateController(Robot* const _r, XMLNode& _node) : ControllerMethod(_r, _node) {
  m_gain = _node.Read("gain", false, 0.,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The proportional gain");

  m_max = _node.Read("maxMagnitude", false,
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The maximum force/velocity magnitude to request");
}

/*----------------------------- Control Selection ----------------------------*/

std::vector<double>
ICreateController::
ComputeDesiredForce(const Cfg& _current, const Cfg& _target, const double) {

 
  double xDist = (_target[0] - _current[0]);
  double yDist = (_target[1] - _current[1]);
  double translateAmt = sqrt(pow(xDist,2) + pow(yDist,2));
  translateAmt = translateAmt;
  double rotAmt = atan2(yDist,xDist) - _current[2]*M_PI;

  //Normalize rotAmt
  if (rotAmt > PI)
    rotAmt -= (2.0 * PI);
  else if (rotAmt < -PI)
    rotAmt += (2.0 * PI);

  //cout << "cur point " << curPoints[0] <<", " << curPoints[1] << " goal: " << goalPoints[0] << ", " << goalPoints[1]\
    << " \nx dist " << xDist << "\ny dist " << yDist << "\nRotation amount: " << rotAmt << "\nTranslation amount: " << translateAmt << endl;
  
  vector<double> RotationAndTranslation;
  if(abs(rotAmt) > 0.06) { 
    RotationAndTranslation.push_back(0);
    RotationAndTranslation.push_back(0);
    RotationAndTranslation.push_back(rotAmt);
  }
  else {
    //throw RunTimeException(WHERE, "STOPPING AFTER ROTATION");
    //usleep(100e6);
    RotationAndTranslation.push_back(translateAmt);
    RotationAndTranslation.push_back(0);
    RotationAndTranslation.push_back(0);
  }

  //cout << "Setting the force {" << RotationAndTranslation[0] << ", " << RotationAndTranslation[1] << ", " << RotationAndTranslation[2] << "}" << endl;
  
  return RotationAndTranslation;
}

/*----------------------------------------------------------------------------*/
