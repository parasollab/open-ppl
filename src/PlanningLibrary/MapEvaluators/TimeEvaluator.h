#ifndef TIME_EVALUATOR_H_
#define TIME_EVALUATOR_H_

#include "MapEvaluatorMethod.h"
#include "Utilities/MetricUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// \ingroup MapEvaluators
/// \brief   A stop-watch like evaluator that returns false after a set amount
///          of time.
/// \tparam  MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TimeEvaluator : public MapEvaluatorMethod<MPTraits> {

  private:

    // Local types
    typedef typename MPTraits::MPProblemType MPProblemType;

    // Member data
    ClockClass m_clock;          ///< The timing object.
    bool       m_started{false}; ///< Has timing started?
    double     m_timeout;        ///< Timeout after this long.

  public:

    // Construction
    TimeEvaluator(double _timeout = 10) : MapEvaluatorMethod<MPTraits>(),
        m_timeout(_timeout) {
      this->SetName("TimeEvaluator");
      m_clock.SetName("TimeEvaluatorClock");
    }
    TimeEvaluator(MPProblemType* _problem, XMLNode& _node)
        : MapEvaluatorMethod<MPTraits>(_problem, _node) {
      this->SetName("TimeEvaluator");
      m_clock.SetName("TimeEvaluatorClock");
      ParseXML(_node);
    }
    virtual ~TimeEvaluator() = default;
    void ParseXML(XMLNode& _node) {
      m_timeout = _node.Read("timeout", true, 0., 0., MAX_DBL, "Timeout");
    }

    virtual bool operator()() {
      if(!m_started) {
        m_clock.StartClock();
        m_started = true;
      }
      m_clock.StopClock();
      m_clock.StartClock();
      return m_clock.GetSeconds() >= m_timeout;
    }
};

#endif
