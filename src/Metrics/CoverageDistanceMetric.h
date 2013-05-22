#ifndef COVERAGEDISTANCEMETRIC_H
#define COVERAGEDISTANCEMETRIC_H

#include "MetricMethod.h"

template<class MPTraits, class Set>
class CoverageDistanceMetric : public MetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID VID;

    CoverageDistanceMetric(const Set& _samples = Set(), string _dmLabel = "", string _outFileName = "");
    CoverageDistanceMetric(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~CoverageDistanceMetric();

    virtual void PrintOptions(ostream& _os);

    double operator()();

  protected:
    //input
    Set m_samples;
    string m_dmLabel, m_outFileName;
    ofstream output;
};

template<class MPTraits, class Set>
CoverageDistanceMetric<MPTraits, Set>::CoverageDistanceMetric(
    const Set& _samples, string _dmLabel, string _outFileName) 
  : m_samples(_samples), m_dmLabel(_dmLabel), m_outFileName(_outFileName) {
    this->SetName("CoverageDistanceMetric" + Set::GetName());
  }

template<class MPTraits, class Set>
CoverageDistanceMetric<MPTraits, Set>::CoverageDistanceMetric(MPProblemType* _problem, XMLNodeReader& _node)
  : MetricMethod<MPTraits>(_problem, _node), m_samples(_node) {

  this->SetName("CoverageDistanceMetric" + Set::GetName());
  m_outFileName = _node.stringXMLParameter("outfilename", true, "", "filename for recording results");
  m_dmLabel = _node.stringXMLParameter("dmLabel", false, "default", "Distance Metric Method");
  output.open((m_outFileName+".coverage").c_str());
}

template<class MPTraits, class Set>
CoverageDistanceMetric<MPTraits, Set>::~CoverageDistanceMetric() {
}

template<class MPTraits, class Set>
void
CoverageDistanceMetric<MPTraits, Set>::PrintOptions(ostream& _os) {
  _os << "Distance Metric:" << m_dmLabel<< endl;
  _os << "Coverage set size:" << m_samples.size() << endl;
}

template<class MPTraits, class Set>
double
CoverageDistanceMetric<MPTraits, Set>::operator()() {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  unsigned int i;
  vector <double> disVec;
  //from each sample in the coverage set, distance to the nearest nodes in the roadmap is calculated
  for(typename Set::Iterator i = m_samples.begin(); i!=m_samples.end(); ++i){
    double distance;
    vector<VID> kClosest;
    BruteForceNF<MPTraits> bfnf(this->GetMPProblem(), m_dmLabel, "__CoverageDistanceMetricNF");
    RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
    bfnf.KClosest(rdmp, rdmp->GetGraph()->begin(), rdmp->GetGraph()->end(), *i, 1, back_inserter(kClosest));
    CfgType nearest = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(kClosest[0]);
    distance = this->GetMPProblem()->GetDistanceMetric(m_dmLabel)->Distance(env, *i, nearest);
    disVec.push_back(distance);
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
  output << "Average of distances:" << avgSum << endl;
  output << "Standard Dev of distances:" << stdDev << endl;
  return avgSum;
}

#endif

