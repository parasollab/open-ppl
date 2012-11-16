#ifndef COVERAGEDISTANCEMETRIC_H
#define COVERAGEDISTANCEMETRIC_H

#include "MetricsMethod.h"

template <class CFG, class WEIGHT>
class CoverageDistanceMetric : public MetricsMethod {
  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

    CoverageDistanceMetric();
    CoverageDistanceMetric(string _dmLabel, string _fileName);
    CoverageDistanceMetric(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~CoverageDistanceMetric() {}

    virtual void PrintOptions(ostream& _os);

    virtual double operator()();

  protected:
    //input
    vector<CFG> m_samples;
    string m_dmLabel;
};

template <class CFG, class WEIGHT>
CoverageDistanceMetric<CFG, WEIGHT>::CoverageDistanceMetric() {
  this->SetName("CoverageDistanceMetric");
  m_dmLabel = "default";
}

template <class CFG, class WEIGHT>
CoverageDistanceMetric<CFG, WEIGHT>::CoverageDistanceMetric(string _dmLabel, string _fileName) {
  this->SetName("CoverageDistanceMetric");
  m_dmLabel = _dmLabel;
  Roadmap<CFG, WEIGHT> covRdmp;
  m_samples.clear();
  covRdmp.ReadRoadmapGRAPHONLY(_fileName.c_str());
  covRdmp.m_pRoadmap->GetVerticesData(m_samples); 
}


template <class CFG, class WEIGHT>
CoverageDistanceMetric<CFG, WEIGHT>::CoverageDistanceMetric(XMLNodeReader& _node, MPProblem* _problem)
  : MetricsMethod(_node, _problem) {
  this->SetName("CoverageDistanceMetric");
  string coveragefilename = _node.stringXMLParameter("filename", true, "", "roadmap filename containing witness samples");
  m_dmLabel = _node.stringXMLParameter("dmLabel", false, "default", "Distance Metric Method");
  //read in samples
  Roadmap<CFG, WEIGHT> covRdmp;
  m_samples.clear();
  covRdmp.ReadRoadmapGRAPHONLY(coveragefilename.c_str());
  covRdmp.m_pRoadmap->GetVerticesData(m_samples);
  if(m_debug) PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void
CoverageDistanceMetric<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "Distance Metric:" <<m_dmLabel<< endl;
  _os << "Coverage set size:" <<m_samples.size() <<endl;
}

template <class CFG, class WEIGHT>
double
CoverageDistanceMetric<CFG, WEIGHT>::operator()() {
  Environment* env = GetMPProblem()->GetRoadmap()->GetEnvironment();
  unsigned int i;
  vector <double> disVec;
  //from each sample in the coverage set, distance to the nearest nodes in the roadmap is calculated
  for( typename vector<CFG>::iterator cfgit = m_samples.begin(); cfgit!=m_samples.end(); ++cfgit){
    double distance;
    vector<VID> kClosest;
    BruteForceNF bfnf(m_dmLabel, "__CoverageDistanceMetricNF", GetMPProblem());
    bfnf.KClosest(GetMPProblem()->GetRoadmap(), GetMPProblem()->GetRoadmap()->m_pRoadmap->descriptor_begin(), GetMPProblem()->GetRoadmap()->m_pRoadmap->descriptor_end(), *cfgit, 1, back_insert_iterator<vector<VID> >(kClosest));
    CFG nearest = GetMPProblem()->GetRoadmap()->m_pRoadmap->find_vertex(kClosest[0])->property();
    distance = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel)->Distance(env, *cfgit, nearest);
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
  if(m_debug){ 
    cout<<"Average of distances:"<<avgSum<<endl;
    cout<<"Standard Dev of distances:"<<stdDev<<endl;
  }
  return avgSum;
}

#endif

