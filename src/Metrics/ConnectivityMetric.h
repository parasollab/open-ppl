#ifndef CONNECTIVITYMETRIC_H
#define CONNECTIVITYMETRIC_H

#include "CoverageMetric.h"

template<class MPTraits>
class ConnectivityMetric : public CoverageMetric<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;

    ConnectivityMetric(const vector<CfgType>& _samples = vector<CfgType>(), 
        const vector<string>& _connectorLabels = vector<string>(), 
        bool _computeAllCCs = false);
    ConnectivityMetric(MPProblemType* _problem, XMLNodeReader& _node, bool _computeAllCCs = false);
    
    virtual ~ConnectivityMetric();

    virtual void PrintOptions(ostream& _os);

    double operator()();
    
  private:
    ofstream output;
};

template<class MPTraits>
ConnectivityMetric<MPTraits>::ConnectivityMetric(const vector<CfgType>& _samples, const vector<string>& _connectorLabels, bool _computeAllCCs)
  : CoverageMetric<MPTraits>(_samples, _connectorLabels, _computeAllCCs){
  this->SetName("ConnectivityMetric");
}

template<class MPTraits>
ConnectivityMetric<MPTraits>::ConnectivityMetric(MPProblemType* _problem, XMLNodeReader& _node, bool _computeAllCCs)
  : CoverageMetric<MPTraits>(_problem, _node, _computeAllCCs) {
    this->SetName("ConnectivityMetric");

    output.open((this->m_outFileName+".connectivity").c_str());
}

template<class MPTraits>
ConnectivityMetric<MPTraits>::~ConnectivityMetric() {
}

template<class MPTraits>
void 
ConnectivityMetric<MPTraits>::PrintOptions(ostream& _os) {
  _os << "Percentage of queries solved" << endl;
}

template<class MPTraits>
double 
ConnectivityMetric<MPTraits>::operator()() {
  CoverageMetric<MPTraits>::operator()(); // Call CoverageMetric first

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
