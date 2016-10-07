#ifndef COVERAGEDISTANCEMETRIC_H
#define COVERAGEDISTANCEMETRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
/// @tparam Set Container type of Cfgs to compare against
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits, class Set>
class CoverageDistanceMetric : public MetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID VID;

    CoverageDistanceMetric(const Set& _samples = Set(), string _dmLabel = "");
    CoverageDistanceMetric(MPProblemType* _problem, XMLNode& _node);
    virtual ~CoverageDistanceMetric() {}

    virtual void Print(ostream& _os) const;

    double operator()();

  protected:
    //input
    Set m_samples;
    string m_dmLabel;
    ofstream m_history;
};

template<class MPTraits, class Set>
CoverageDistanceMetric<MPTraits, Set>::
CoverageDistanceMetric(const Set& _samples, string _dmLabel) :
  m_samples(_samples), m_dmLabel(_dmLabel) {
    this->SetName("CoverageDistanceMetric" + Set::GetName());
  }

template<class MPTraits, class Set>
CoverageDistanceMetric<MPTraits, Set>::
CoverageDistanceMetric(MPProblemType* _problem, XMLNode& _node) :
  MetricMethod<MPTraits>(_problem, _node), m_samples(_node) {

    this->SetName("CoverageDistanceMetric" + Set::GetName());
    m_dmLabel = _node.Read("dmLabel", false, "default", "Distance Metric Method");
  }

template<class MPTraits, class Set>
void
CoverageDistanceMetric<MPTraits, Set>::
Print(ostream& _os) const {
  _os << "Distance Metric:" << m_dmLabel<< endl;
  _os << "Coverage set size:" << m_samples.size() << endl;
}

template<class MPTraits, class Set>
double
CoverageDistanceMetric<MPTraits, Set>::
operator()() {

  static size_t numCalls = 0;
  if(numCalls == 0)
    m_history.open(this->GetBaseFilename() + ".coverage");

  unsigned int i;
  vector <double> disVec;
  //from each sample in the coverage set, distance to the nearest nodes in the roadmap is calculated
  for(typename Set::Iterator i = m_samples.begin(); i!=m_samples.end(); ++i){
    vector<pair<VID, double> > kClosest;
    BruteForceNF<MPTraits> bfnf(m_dmLabel, false, 1);
    bfnf.SetMPProblem(this->GetMPProblem());
    bfnf.SetLabel("__CoverageDistanceMetricNF");
    RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
    bfnf.FindNeighbors(rdmp,
        rdmp->GetGraph()->begin(), rdmp->GetGraph()->end(), true,
        *i, back_inserter(kClosest));
    CfgType nearest = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(kClosest[0].first);
    //distance = this->GetMPProblem()->GetDistanceMetric(m_dmLabel)->Distance(env, *i, nearest);
    disVec.push_back(kClosest[0].second);
  }
  //average of distance vector and standard deviation is calculated
  double avgSum = 0, stdDev = 0.0;
  for(i = 0; i<disVec.size(); i++){
    avgSum += disVec[i];
  }
  avgSum = avgSum/disVec.size();
  for(i = 0; i<disVec.size(); i++){
    stdDev += (avgSum - disVec[i]) * (avgSum - disVec[i]);
  }
  stdDev = stdDev/disVec.size();
  stdDev = sqrt(stdDev);
  m_history << "Average of distances:" << avgSum << endl;
  m_history << "Standard Dev of distances:" << stdDev << endl;
  return avgSum;
}

#endif

