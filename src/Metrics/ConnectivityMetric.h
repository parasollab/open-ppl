#ifndef CONNECTIVITYMETRIC_H
#define CONNECTIVITYMETRIC_H

#include "CoverageMetric.h"

template <class CFG, class WEIGHT>
class ConnectivityMetric : public CoverageMetric<CFG, WEIGHT> {
  public:

    ConnectivityMetric();
    ConnectivityMetric(vector<CFG>& _samples, vector<string> _nodeConnection, bool _computeAllCCs = false);
    ConnectivityMetric(XMLNodeReader& _node, MPProblem* _problem, bool _computeAllCCs = false);
    virtual ~ConnectivityMetric() {}

    virtual void PrintOptions(ostream& _os);

    virtual double operator()() {
      return operator()(this->GetMPProblem()->CreateMPRegion());
    }
    virtual double operator()(int _regionID);
};

template <class CFG, class WEIGHT>
ConnectivityMetric<CFG, WEIGHT>::ConnectivityMetric() {
  this->SetName("ConnectivityMetric");
}

template <class CFG, class WEIGHT>
ConnectivityMetric<CFG, WEIGHT>::ConnectivityMetric(vector<CFG>& _samples, vector<string> _nodeConnection, bool _computeAllCCs)
  : CoverageMetric<CFG, WEIGHT>(_samples, _nodeConnection, _computeAllCCs){
  this->SetName("ConnectivityMetric");
}

template <class CFG, class WEIGHT>
ConnectivityMetric<CFG, WEIGHT>::ConnectivityMetric(XMLNodeReader& _node, MPProblem* _problem, bool _computeAllCCs)
  : CoverageMetric<CFG, WEIGHT>(_node, _problem, _computeAllCCs) {
    this->SetName("ConnectivityMetric");
    if(this->m_debug) PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void ConnectivityMetric<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "Percentage of queries solved" << endl;
}

template <class CFG, class WEIGHT>
double ConnectivityMetric<CFG, WEIGHT>::operator()(int _regionID) {
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

  int numQueries = 0;
  for(size_t i=0; i<this->m_connections.size(); ++i)
    sort(this->m_connections[i].begin(), this->m_connections[i].end());
  for(size_t i=0; i<this->m_connections.size(); ++i) {
    for(size_t j=i+1; j<this->m_connections.size(); ++j) {
      vector<VID> intersection;
      set_intersection(this->m_connections[i].begin(), this->m_connections[i].end(),
                       this->m_connections[j].begin(), this->m_connections[j].end(),
                       back_insert_iterator<vector<VID> >(intersection));
      if(!(intersection.empty()))
        numQueries++;
    }
  }
  return (((double)numQueries)/((double)(this->m_connections.size()*(this->m_connections.size()-1))/2.0));
}

#endif
