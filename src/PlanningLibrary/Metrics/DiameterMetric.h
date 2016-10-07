#ifndef DIAMETERMETRIC_H
#define DIAMETERMETRIC_H

#include "MetricMethod.h"
#include "containers/sequential/graph/algorithms/diameter.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class DiameterMetric : public MetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;

    DiameterMetric();
    DiameterMetric(MPProblemType* _problem, XMLNode& _node);
    virtual ~DiameterMetric() {}

    virtual void Print(ostream& _os) const;

    double operator()();
};

template<class MPTraits>
DiameterMetric<MPTraits>::
DiameterMetric() {
  this->SetName("DiameterMetric");
}

template<class MPTraits>
DiameterMetric<MPTraits>::
DiameterMetric(MPProblemType* _problem, XMLNode& _node)
  : MetricMethod<MPTraits>(_problem, _node) {
    this->SetName("DiameterMetric");
}

template<class MPTraits>
void
DiameterMetric<MPTraits>::
Print(ostream& _os) const {
  _os << "CC diameter" << endl;
}

template<class MPTraits>
double
DiameterMetric<MPTraits>::
operator()() {

  //vector<double> prev_diameter, new_diameter;
  vector<double> diameter;
  double ccDiameter;
  RoadmapType* rMap = this->GetMPProblem()->GetRoadmap();
  GraphType* rGraph = rMap->GetGraph();

  //get ccs
  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  get_cc_stats(*rGraph, cMap, ccs);

  //fileter out singletons
  vector<pair<size_t, VID> > filteredCCs;
  typename vector<pair<size_t, VID> >::iterator CC;
  for(CC = ccs.begin(); CC != ccs.end(); ++CC) {
    if(CC->first > 1)
      filteredCCs.push_back(*CC);
  }

  //compute new cc diameters
  vector<VID> ccVIDs;
  typename vector<VID>::iterator v;
  vector<CfgType> ccData;
  for(CC = filteredCCs.begin(); CC != filteredCCs.end(); ++CC) {
    ccVIDs.clear();
    ccData.clear();
    cMap.reset();
    get_cc(*rGraph, cMap, CC->second, ccVIDs);
    for(v = ccVIDs.begin(); v != ccVIDs.end(); ++v)
      ccData.push_back((*(rGraph->find_vertex(*v))).property());
#ifndef _PARALLEL
    // Beware- this diameter is hop count (weight =1), proper fix is needed
    diameter.push_back(stapl::sequential::diameter(*rGraph, CC->second));
#else
    // TODO: this is to be implemented by STAPL team
    // diameter.push_back(stapl::diameter(*rGraph, CC->second));
#endif
  }

  ccDiameter = *(diameter.begin());
  for(vector<double>::iterator i = diameter.begin(); i != diameter.end(); i++)
    ccDiameter = max(ccDiameter, *(i+1));

  return ccDiameter;
}

#endif
