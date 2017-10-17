#include "ICreateController.h"

#include <algorithm>

#include "ConfigurationSpace/Cfg.h"
#include "Utilities/XMLNode.h"


/*------------------------------ Construction --------------------------------*/

ICreateController::
ICreateController(Robot* const _r, const double _gain, const double _max)
  : ControllerMethod(_r), m_gain(_gain), m_max(_max) { }


ICreateController::
ICreateController(Robot* const _r, XMLNode& _node) : ControllerMethod(_r, _node) {
  m_gain = _node.Read("gain", true, 0.,
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
ComputeDesiredForce(const Cfg& _current, const Cfg& _target, const double headingAngle) {

  vector<double> curPoints = _current.GetData();
  vector<double> goalPoints = _target.GetData();
  
  double xDist = (goalPoints[0] - curPoints[0]);
  double yDist = (goalPoints[1] - curPoints[1]);
  double translateAmt = sqrt(pow(xDist,2) + pow(yDist,2));
  double rotAmt = atan2(yDist,xDist) - headingAngle;

  //Normalize rotAmt
  if (rotAmt > PI)
    rotAmt -= (2.0 * PI);
  else if (rotAmt < -PI)
    rotAmt += (2.0 * PI);

  cout << "cur point " << curPoints[0] <<", " << curPoints[1] << " goal: " << goalPoints[0] << ", " << goalPoints[1]\
    << " \nx dist " << xDist << "\ny dist " << yDist << "\nRoation amount: " << rotAmt << "\nTranslation amount: " << translateAmt << endl;
  
  vector<double> RotationAndTranslation;
  RotationAndTranslation.push_back(rotAmt);
  RotationAndTranslation.push_back(translateAmt);
  
  return RotationAndTranslation;
}

/*----------------------------------------------------------------------------*/
