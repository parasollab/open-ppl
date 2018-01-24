#include "ICreateController.h"

#include "ConfigurationSpace/Cfg.h"
#include "Utilities/XMLNode.h"

#include <algorithm>


/*------------------------------ Construction --------------------------------*/

ICreateController::
ICreateController(Robot* const _r, const double _gain, const double _max)
  : ControllerMethod(_r) { }


ICreateController::
ICreateController(Robot* const _r, XMLNode& _node) : ControllerMethod(_r, _node) {
}


ICreateController::
ICreateController(Robot* const _r, const ICreateController& _c)
  : ControllerMethod(_r, _c)
{ }


std::unique_ptr<ControllerMethod>
ICreateController::
Clone(Robot* const _r) const {
  return std::unique_ptr<ICreateController>(new ICreateController(_r, *this));
}


ICreateController::
~ICreateController() = default;

/*----------------------------- Control Selection ----------------------------*/

std::vector<double>
ICreateController::
ComputeDesiredForce(const Cfg& _current, const Cfg& _target, const double) {
  const double xDist = (_target[0] - _current[0]);
  const double yDist = (_target[1] - _current[1]);
  const double translateAmt = sqrt(pow(xDist,2) + pow(yDist,2));
  double rotAmt = atan2(yDist,xDist) - _current[2]*M_PI;

  // Normalize the rotation to within [-2PI, 2PI].
  if(rotAmt > PI)
    rotAmt -= (2 * PI);
  else if(rotAmt < -PI)
    rotAmt += (2 * PI);

  vector<double> RotationAndTranslation;
  if(abs(rotAmt) > 0.06) {
    RotationAndTranslation.push_back(0);
    RotationAndTranslation.push_back(0);
    RotationAndTranslation.push_back(rotAmt);
  }
  else {
    RotationAndTranslation.push_back(translateAmt);
    RotationAndTranslation.push_back(0);
    RotationAndTranslation.push_back(0);
  }

  return RotationAndTranslation;
}

/*----------------------------------------------------------------------------*/
