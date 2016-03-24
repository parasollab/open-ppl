#ifndef CCDISTANCEMETRIC_H
#define CCDISTANCEMETRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class CCDistanceMetric : public MetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;

    CCDistanceMetric(string _dm="");
    CCDistanceMetric(MPProblemType* _problem, XMLNode& _node);
    virtual ~CCDistanceMetric() {}

    virtual void Print(ostream& _os) const;

    double operator()();

  protected:
    string m_dmLabel;
};

template<class MPTraits>
CCDistanceMetric<MPTraits>::
CCDistanceMetric(string _dm)
  : m_dmLabel(_dm) {
    this->SetName("CCDistanceMetric");
}

template<class MPTraits>
CCDistanceMetric<MPTraits>::
CCDistanceMetric(MPProblemType* _problem, XMLNode& _node)
  : MetricMethod<MPTraits>(_problem, _node) {
    this->SetName("CCDistanceMetric");

    m_dmLabel = _node.Read("dmLabel", true, "", "distance metric method");
}

template<class MPTraits>
void
CCDistanceMetric<MPTraits>::
Print(ostream& _os) const {
  _os << "CC distance" << endl;
  _os << "\tdistance metric = " << m_dmLabel << endl;
}

template<class MPTraits>
double
CCDistanceMetric<MPTraits>::
operator()() {

  vector<double> distance;
  double ccDistance;
  RoadmapType* rmap = this->GetMPProblem()->GetRoadmap();
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
      distance.push_back(this->GetMPProblem()->GetDistanceMetric(m_dmLabel)->Distance(
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
