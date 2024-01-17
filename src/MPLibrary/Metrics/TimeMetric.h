#ifndef PMPL_TIME_METRIC_H_
#define PMPL_TIME_METRIC_H_

#include "MetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// This metric measures the r-usage time for the entire MPLibrary run.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
class TimeMetric : virtual public MetricMethod {

  public:

    ///@name Construction
    ///@{

    TimeMetric();

    TimeMetric(XMLNode& _node);

    virtual ~TimeMetric() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MetricMethod Overrides
    ///@{

    virtual double operator()() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    static std::string s_clockName; ///< The clock name.

    ///@}

};

#endif
