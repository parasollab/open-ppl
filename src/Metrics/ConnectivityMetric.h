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

    virtual double operator()();
    
  private:
    ofstream output;
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

    output.open((this->m_outFileName+".connectivity").c_str());

    if(this->m_debug) PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void ConnectivityMetric<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "Percentage of queries solved" << endl;
}

template <class CFG, class WEIGHT>
double ConnectivityMetric<CFG, WEIGHT>::operator()() {
  CoverageMetric<CFG, WEIGHT>::operator()(); // Call CoverageMetric first

  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  static size_t numcalls = 0;
  int numQueries = 0;
  size_t sz = this->m_connections.size();

  for(size_t i=0; i<sz; ++i)
    sort(this->m_connections[i].begin(), this->m_connections[i].end());
  for(size_t i=0; i<sz; ++i) {
    for(size_t j=i+1; j<sz; ++j) {
      vector<VID> intersection;
      set_intersection(this->m_connections[i].begin(), this->m_connections[i].end(),
                       this->m_connections[j].begin(), this->m_connections[j].end(),
                       back_insert_iterator<vector<VID> >(intersection));
      if(!(intersection.empty()))
        numQueries++;
    }
  }

  double connectivityAmt = (double(numQueries))/(double(sz*(sz-1))/2.0);
  output << numcalls++ << "\t" << connectivityAmt << endl;

  return connectivityAmt;
}

#endif
