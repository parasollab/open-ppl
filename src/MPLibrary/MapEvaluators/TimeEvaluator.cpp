#include "TimeEvaluator.h"

#include <limits>

/*------------------------------ Construction --------------------------------*/

TimeEvaluator::
TimeEvaluator(const double _timeout) : MapEvaluatorMethod(),
    m_timeout(_timeout) {
  this->SetName("TimeEvaluator");
}


TimeEvaluator::
TimeEvaluator(XMLNode& _node) : MapEvaluatorMethod(_node) {
  this->SetName("TimeEvaluator");
  m_timeout = _node.Read("timeout", true, 0., 0.,
      std::numeric_limits<double>::max(), "Maximum allowed running time");
}

/*------------------------- MPBaseObject Interface ---------------------------*/

void
TimeEvaluator::
Initialize() {
  if(this->m_debug)
    std::cout << "TimeEvaluator::Initialize()" << std::endl;
  m_started = false;
  m_clock.ClearClock();
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

bool
TimeEvaluator::
operator()() {
  if(!m_started) {
    m_clock.StartClock();
    m_started = true;
  }
  m_clock.StopClock();
  m_clock.StartClock();
  return m_clock.GetSeconds() >= m_timeout;
}

/*----------------------------------------------------------------------------*/
