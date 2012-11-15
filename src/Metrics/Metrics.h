#ifndef METRICS_H
#define METRICS_H

#include <boost/mpl/list.hpp>
#include "MPUtils.h"
#include "NumNodesMetric.h"
#include "NumEdgesMetric.h"
#include "CoverageMetric.h"
#include "ConnectivityMetric.h"
#include "DiameterMetric.h"
#include "CCDistanceMetric.h"
#include "TimeMetric.h"
#include "CoverageDistanceMetric.h"

namespace pmpl_detail {
  typedef boost::mpl::list<
    NumNodesMetric,
    NumEdgesMetric,
    TimeMetric,
    CoverageMetric<CfgType, WeightType>,
    ConnectivityMetric<CfgType, WeightType>,
    DiameterMetric<CfgType, WeightType>,
    CCDistanceMetric<CfgType, WeightType>,
    CoverageDistanceMetric<CfgType, WeightType>
    > MetricMethodList;
}

class Metric : private ElementSet<MetricsMethod>, public MPBaseObject {
  public:
    typedef shared_ptr<MetricsMethod> MetricMethodPtr;
    typedef ElementSet<MetricsMethod>::MethodPointer MetricPointer;

    template<typename MethodList>
    Metric() : ElementSet<MetricsMethod>(MethodList()) {}

    Metric() : ElementSet<MetricsMethod>(pmpl_detail::MetricMethodList()) {}

    template <typename MethodList>
    Metric(XMLNodeReader& _node, MPProblem* _problem, MethodList const&)
      : ElementSet<MetricsMethod>(MethodList()), MPBaseObject(_problem) {
      for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
        if(!AddElement(citr->getName(), *citr, _problem))
	  citr->warnUnknownNode();
      }
    }

    Metric(XMLNodeReader& _node, MPProblem* _problem)
      : ElementSet<MetricsMethod>(pmpl_detail::MetricMethodList()), MPBaseObject(_problem) {
      for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
	if(!AddElement(citr->getName(), *citr, _problem))
	  citr->warnUnknownNode();
      }
    }

    virtual ~Metric();

    MetricPointer GetMethod(string _label);
  
    void PrintOptions(ostream& _os) const;
};

#endif
