#ifndef DIAMETERMETRIC_H
#define DIAMETERMETRIC_H

#include "MetricsMethod.h"
#include <graph/algorithms/diameter.h>

template <class CFG, class WEIGHT>
class DiameterMetric : public MetricsMethod {
  public:

    DiameterMetric();
    DiameterMetric(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~DiameterMetric() {}

    virtual void PrintOptions(ostream& _os);

    virtual double operator()();
};

template <class CFG, class WEIGHT>
DiameterMetric<CFG, WEIGHT>::DiameterMetric() {
  this->SetName("DiameterMetric");
}

template <class CFG, class WEIGHT>
DiameterMetric<CFG, WEIGHT>::DiameterMetric(XMLNodeReader& _node, MPProblem* _problem)
  : MetricsMethod(_node, _problem) {
    this->SetName("DiameterMetric");

    if(m_debug) PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void DiameterMetric<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "CC diameter" << endl;
}

template <class CFG, class WEIGHT>
double DiameterMetric<CFG, WEIGHT>::operator()() {
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

  //vector<double> prev_diameter, new_diameter;
  vector<double> diameter;
  double ccDiameter;
  Roadmap<CFG, WEIGHT>* rmap = GetMPProblem()->GetRoadmap();
  RoadmapGraph<CFG, WEIGHT>* pMap = rmap->m_pRoadmap;

  //get ccs
  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map< RoadmapGraph<CFG, WEIGHT>, size_t > cmap;
  get_cc_stats(*pMap, cmap, ccs);

  //fileter out singletons
  vector<pair<size_t, VID> > filtered_ccs;
  typename vector<pair<size_t, VID> >::iterator CC;
  for(CC = ccs.begin(); CC != ccs.end(); ++CC) {
    if(CC->first > 1)
      filtered_ccs.push_back(*CC);
  }

  //compute new cc diameters
  vector<VID> cc_vids;
  typename vector<VID>::iterator V;
  vector<CFG> cc_data;
  for(CC = filtered_ccs.begin(); CC != filtered_ccs.end(); ++CC) {
    cc_vids.clear();
    cc_data.clear();
    cmap.reset();
    get_cc(*pMap, cmap, CC->second, cc_vids);
    for(V = cc_vids.begin(); V != cc_vids.end(); ++V)
      cc_data.push_back((*(pMap->find_vertex(*V))).property());
// Beware- this diameter is hop count (weight =1), proper fix is needed
    diameter.push_back(stapl::sequential::diameter(*pMap, CC->second));
  }

  ccDiameter = *(diameter.begin());
  for(vector<double>::iterator I = diameter.begin(); I != diameter.end(); I++)
    ccDiameter = max(ccDiameter, *(I+1));

  return ccDiameter;
}

#endif
