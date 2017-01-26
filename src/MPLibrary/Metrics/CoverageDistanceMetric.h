#ifndef COVERAGE_DISTANCE_METRIC_H
#define COVERAGE_DISTANCE_METRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @TODO
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits, class Set>
class CoverageDistanceMetric : public MetricMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    CoverageDistanceMetric(const Set& _samples = Set(), string _dmLabel = "");

    CoverageDistanceMetric(XMLNode& _node);

    virtual ~CoverageDistanceMetric() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Metric Interface
    ///@{

    virtual double operator()() override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    Set m_samples;
    string m_dmLabel;
    ofstream m_history;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template<class MPTraits, class Set>
CoverageDistanceMetric<MPTraits, Set>::
CoverageDistanceMetric(const Set& _samples, string _dmLabel) :
    m_samples(_samples), m_dmLabel(_dmLabel) {
  this->SetName("CoverageDistanceMetric" + Set::GetName());
}


template<class MPTraits, class Set>
CoverageDistanceMetric<MPTraits, Set>::
CoverageDistanceMetric(XMLNode& _node) :
    MetricMethod<MPTraits>(_node), m_samples(_node) {
  this->SetName("CoverageDistanceMetric" + Set::GetName());
  m_dmLabel = _node.Read("dmLabel", false, "default", "Distance Metric Method");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template<class MPTraits, class Set>
void
CoverageDistanceMetric<MPTraits, Set>::
Print(ostream& _os) const {
  _os << "Distance Metric: " << m_dmLabel << std::endl
      << "Coverage set size: " << m_samples.size() << std::endl;
}

/*---------------------------- Metric Interface ------------------------------*/

template<class MPTraits, class Set>
double
CoverageDistanceMetric<MPTraits, Set>::
operator()() {
  static size_t numCalls = 0;
  if(numCalls == 0)
    m_history.open(this->GetBaseFilename() + ".coverage");

  vector <double> disVec;
  // From each sample in the coverage set, distance to the nearest nodes in the
  // roadmap is calculated
  for(auto i = m_samples.begin(); i != m_samples.end(); ++i) {
    vector<pair<VID, double> > kClosest;
    BruteForceNF<MPTraits> bfnf(m_dmLabel, false, 1);
    bfnf.SetMPLibrary(this->GetMPLibrary());
    bfnf.Initialize();
    bfnf.SetLabel("__CoverageDistanceMetricNF");
    RoadmapType* r = this->GetRoadmap();
    GraphType* g = r->GetGraph();
    bfnf.FindNeighbors(r, g->begin(), g->end(), true, *i,
        back_inserter(kClosest));
    CfgType nearest = g->GetVertex(kClosest[0].first);
    disVec.push_back(kClosest[0].second);
  }
  //average of distance vector and standard deviation is calculated
  double avgSum = 0;
  for(size_t i = 0; i < disVec.size(); ++i)
    avgSum += disVec[i];
  avgSum = avgSum / disVec.size();

  double stdDev = 0;
  for(size_t i = 0; i < disVec.size(); ++i) {
    const auto temp = avgSum - disVec[i];
    stdDev += temp * temp;
  }
  stdDev = std::sqrt(stdDev / disVec.size());

  m_history << "Average of distances: " << avgSum << endl
            << "Standard Dev of distances: " << stdDev << endl;

  return avgSum;
}

/*----------------------------------------------------------------------------*/

#endif
