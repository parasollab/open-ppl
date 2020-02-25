#ifndef TIME_EVALUATOR_H_
#define TIME_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

#include <limits>

#include "Utilities/MetricUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// A stop-watch like evaluator that returns false after a set amount of time.
///
/// @TODO This should be removed, as it's redundant with just using the more
///       flexibile ConditionalEvaluator with the TimeMetric.
///
/// @WARNING This evaluator uses an rusage-based clock just like our StatClass.
///          The two probably aren't safe to access concurrently. We can adjust
///          this by switching to a C++11 chrono-based clock when the need
///          arises.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TimeEvaluator : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    TimeEvaluator(const double _timeout = 10);

    TimeEvaluator(XMLNode& _node);

    virtual ~TimeEvaluator() = default;

    ///@}
    ///@name MPBaseObject Interface
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluatorMethod Interface
    ///@{

    virtual bool operator()() override;

    ///@}

    static void Reset();

  private:

    ///@name Internal State
    ///@{

    static bool m_started; ///< Has timing started?
    double     m_timeout;        ///< Timeout after this long.
    static ClockClass m_clock;          ///< Internal timer.

    ///@}
};

template <typename MPTraits>
bool TimeEvaluator<MPTraits>::m_started = false;

template <typename MPTraits>
ClockClass TimeEvaluator<MPTraits>::m_clock;

template <typename MPTraits>
void
TimeEvaluator<MPTraits>::
Reset() {
  m_started = false;
  m_clock.ClearClock();
}


/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
TimeEvaluator<MPTraits>::
TimeEvaluator(const double _timeout) : MapEvaluatorMethod<MPTraits>(),
    m_timeout(_timeout) {
  this->SetName("TimeEvaluator");
}


template <typename MPTraits>
TimeEvaluator<MPTraits>::
TimeEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("TimeEvaluator");
  m_timeout = _node.Read("timeout", true, 0., 0.,
      std::numeric_limits<double>::max(), "Maximum allowed running time");
}

/*------------------------- MPBaseObject Interface ---------------------------*/

template <typename MPTraits>
void
TimeEvaluator<MPTraits>::
Initialize() {
  if(this->m_debug)
    std::cout << "TimeEvaluator::Initialize()" << std::endl;
  m_started = false;
  m_clock.ClearClock();
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

template <typename MPTraits>
bool
TimeEvaluator<MPTraits>::
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

#endif
