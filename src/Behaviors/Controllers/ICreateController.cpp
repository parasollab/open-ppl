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
  /// @TODO Make robot turn/translate at max speed as often as possible, rather
  ///       than using something proportional to the error.
  const double x = _target[0] - _current[0],
               y = _target[1] - _current[1],
               a = _target[2] - _current[2],
               translation = std::sqrt(x*x + y*y),
               preRotation = atan2(y, x)/PI - _current[2];

  static constexpr double threshold = 1e-4;

  // If we're still here, we are at the target. Line up with the end Cfg.
  if(translation <= threshold)
  {
    std::cout << "End rotation" << std::endl;
    return {0, 0, a};
  }
  // Line up to move towards target.
  else if(std::abs(preRotation) > threshold)
  {
    std::cout << "Pre rotation" << std::endl;
    return {0, 0, preRotation};
  }
  // If we're still here, we are lined up. Go to the target.
  else
  {
    std::cout << "Translation" << std::endl;
    return {translation, 0, 0};
  }
}

/*----------------------------------------------------------------------------*/
