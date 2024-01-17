#ifndef PMPL_TIME_EVALUATOR_H_
#define PMPL_TIME_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

#include "Utilities/MetricUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// A stop-watch like evaluator that returns false after a set amount of time.
///
/// This class maintains a separate clock for each instance. For a global clock
/// use ConditionalEvaluator with the TimeMetric.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class TimeEvaluator : public MapEvaluatorMethod {

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

  private:

    ///@name Internal State
    ///@{

    bool m_started{false}; ///< Has timing started?
    double     m_timeout;  ///< Timeout after this long.
    ClockClass m_clock;    ///< Internal timer.

    ///@}
};

#endif
