#include "ICreateController.h"

#include <algorithm>

#include "ConfigurationSpace/Cfg.h"
#include "Utilities/XMLNode.h"

//TEMP
#include <unistd.h>
/*------------------------------ Construction --------------------------------*/

ICreateController::
ICreateController(Robot* const _r, const double _gain, const double _max)
  : ControllerMethod(_r) { }


ICreateController::
ICreateController(Robot* const _r, XMLNode& _node) : ControllerMethod(_r, _node) {
}

/*----------------------------- Control Selection ----------------------------*/

std::vector<double>
ICreateController::
ComputeDesiredForce(const Cfg& _current, const Cfg& _target, const double) {
  const double xDist = (_target[0] - _current[0]);
  const double yDist = (_target[1] - _current[1]);
  const double translateAmt = sqrt(pow(xDist,2) + pow(yDist,2));
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
