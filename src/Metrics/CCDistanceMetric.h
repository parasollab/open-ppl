#ifndef CCDISTANCEMETRIC_H
#define CCDISTANCEMETRIC_H

#include "MetricsMethod.h"
#include "DistanceMetrics.h"

template <class CFG, class WEIGHT>
class CCDistanceMetric : public MetricsMethod {
  public:

    CCDistanceMetric();
    CCDistanceMetric(string _dm, string _nf);
    CCDistanceMetric(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~CCDistanceMetric() {}

    virtual void PrintOptions(ostream& _os);

    virtual double operator()();

  protected:
    string m_dm;
    string m_nf;
};

template <class CFG, class WEIGHT>
CCDistanceMetric<CFG, WEIGHT>::CCDistanceMetric() {
  this->SetName("CCDistanceMetric");
}

template <class CFG, class WEIGHT>
CCDistanceMetric<CFG, WEIGHT>::CCDistanceMetric(string _dm, string _nf)
  : m_dm(_dm), m_nf(_nf) {
    this->SetName("CCDistanceMetric");
}

template <class CFG, class WEIGHT>
CCDistanceMetric<CFG, WEIGHT>::CCDistanceMetric(XMLNodeReader& _node, MPProblem* _problem)
  : MetricsMethod(_node, _problem) {
    this->SetName("CCDistanceMetric");

    m_dm = _node.stringXMLParameter("dmMethod", true, "", "distance metric method");
    m_nf = _node.stringXMLParameter("nfMethod", true, "", "neighborhood finder method");

    if(m_debug) PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void CCDistanceMetric<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "CC distance" << endl;
  _os << "\tdistance metric = " << m_dm << endl;
  _os << "\tneighborhood finder = " << m_nf << endl;
}

template <class CFG, class WEIGHT>
double CCDistanceMetric<CFG, WEIGHT>::operator()() {

  typedef RoadmapGraph<CFG, WEIGHT> RoadmapGraphType;
  typedef typename RoadmapGraphType::VID VID;
  typedef pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;
  
  vector<double> distance;
  double ccDistance;
  Roadmap<CFG, WEIGHT>* rmap = GetMPProblem()->GetRoadmap();
  RoadmapGraphType* pMap = rmap->m_pRoadmap;
  NeighborhoodFinder::NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder()->GetMethod(m_nf);

  Environment* pEnv = rmap->GetEnvironment();

  //get ccs
  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map< RoadmapGraph<CFG, WEIGHT>, size_t > cmap;
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
      nf->KClosestPairs(rmap, cciVids.begin(), cciVids.end(), ccjVids.begin(), ccjVids.end(), 1, back_inserter(pairs));
      distance.push_back(this->GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm)->Distance(pEnv,
                                                                                              GetCfg()(pMap, pairs[0].first),
                                                                                              GetCfg()(pMap, pairs[0].second)));
    }
  }
  ccDistance = *(distance.begin());
  for(vector<double>::iterator I = distance.begin(); I != distance.end(); I++)
    ccDistance = max(ccDistance, *(I+1));

  return ccDistance;
}

#endif
