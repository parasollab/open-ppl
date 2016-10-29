#ifndef TIME_EVALUATOR_H_
#define TIME_EVALUATOR_H_

#include "MapEvaluatorMethod.h"
#include "Utilities/MetricUtils.h"

#include <chrono>

////////////////////////////////////////////////////////////////////////////////
/// \ingroup MapEvaluators
/// \brief   A stop-watch like evaluator that returns false after a set amount
///          of time.
/// \tparam  MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TimeEvaluator : public MapEvaluatorMethod<MPTraits> {

  private:

    ///@name Local types
    ///@{

    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@}
    ///@name Internal State
    ///@{

    double    m_timeout; ///< Timeout after this number of seconds.

    ///@}

  public:

    // Construction
    TimeEvaluator(double _timeout = 10) : MapEvaluatorMethod<MPTraits>(),
        m_timeout(_timeout) {
      this->SetName("TimeEvaluator");
    }
    TimeEvaluator(MPProblemType* _problem, XMLNode& _node)
        : MapEvaluatorMethod<MPTraits>(_problem, _node) {
      this->SetName("TimeEvaluator");
      m_timeout = _node.Read("timeout", true, 0., 0., MAX_DBL,
          "Timeout in seconds.");
    }
    virtual ~TimeEvaluator() = default;

    virtual bool operator()() {
      static const string ClockName = "TotalExecutionTime";
      auto stats = this->GetStatClass();
      stats->StopClock(ClockName);
      stats->StartClock(ClockName);

      return stats->GetSeconds(ClockName) >= m_timeout;
    }

};

#endif
