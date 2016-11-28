#ifndef CC_DISTANCE_METRIC_H
#define CC_DISTANCE_METRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief TODO.
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CCDistanceMetric : public MetricMethod<MPTraits> {

  public:

    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    CCDistanceMetric(string _dm="");
    CCDistanceMetric(XMLNode& _node);
    virtual ~CCDistanceMetric() {}

    virtual void Print(ostream& _os) const;

    double operator()();

  protected:
    string m_dmLabel;
};

template <typename MPTraits>
CCDistanceMetric<MPTraits>::
CCDistanceMetric(string _dm)
  : m_dmLabel(_dm) {
    this->SetName("CCDistanceMetric");
}

template <typename MPTraits>
CCDistanceMetric<MPTraits>::
CCDistanceMetric(XMLNode& _node)
  : MetricMethod<MPTraits>(_node) {
    this->SetName("CCDistanceMetric");

    m_dmLabel = _node.Read("dmLabel", true, "", "distance metric method");
}

template <typename MPTraits>
void
CCDistanceMetric<MPTraits>::
Print(ostream& _os) const {
  _os << "CC distance" << endl;
  _os << "\tdistance metric = " << m_dmLabel << endl;
}

template <typename MPTraits>
double
CCDistanceMetric<MPTraits>::
operator()() {

  vector<double> distance;
  double ccDistance;
  RoadmapType* rmap = this->GetRoadmap();
  GraphType* pMap = rmap->GetGraph();

  //get ccs
  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map<GraphType, size_t > cmap;
  get_cc_stats(*pMap, cmap, ccs);

  //filter out singletons
  vector<pair<size_t, VID> > filteredCCs;
  typename vector<pair<size_t, VID> >::iterator cci, ccj;
  for(cci = ccs.begin(); cci != ccs.end(); ++cci) {
    if(cci->first > 1)
      filteredCCs.push_back(*cci);
  }

  //compute new inter cc distances
  for(cci = filteredCCs.begin(); cci+1 < filteredCCs.end(); ++cci) {
    vector<VID> cciVids;
    cmap.reset();
    get_cc(*pMap, cmap, cci->second, cciVids);

    for(ccj = cci+1; ccj != filteredCCs.end(); ++ccj) {
      vector<VID> ccjVids;
      cmap.reset();
      get_cc(*pMap, cmap, ccj->second, ccjVids);

      vector<pair<VID, VID> > pairs;
      distance.push_back(this->GetDistanceMetric(m_dmLabel)->Distance(
                                                                                 pMap->GetVertex(pairs[0].first),
                                                                                 pMap->GetVertex(pairs[0].second)));
    }
  }
  ccDistance = *(distance.begin());
  for(vector<double>::iterator I = distance.begin(); I != distance.end(); I++)
    ccDistance = max(ccDistance, *(I+1));

  return ccDistance;
}

#endif
