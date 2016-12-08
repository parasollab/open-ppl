#include "SimpleController.h"

#include "ConfigurationSpace/Cfg.h"


/*------------------------------ Construction --------------------------------*/

SimpleController::
SimpleController(Robot* const _r, const double _gain) : ControllerMethod(_r),
    m_gain(_gain) { }

/*----------------------------- Control Selection ----------------------------*/

std::vector<double>
SimpleController::
ComputeDesiredForce(const Cfg& _current, const Cfg& _target, const double) {
  return ((_target - _current) * m_gain).GetData();
}

/*----------------------------------------------------------------------------*/
