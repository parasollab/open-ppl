#ifndef CONNECTIVITYMETRIC_H
#define CONNECTIVITYMETRIC_H

#include "CoverageMetric.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
/// @tparam Set Container type of Cfgs to compare against
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits, class Set>
class ConnectivityMetric : public CoverageMetric<MPTraits, Set> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;

    ConnectivityMetric(const Set& _samples = Set(),
        const vector<string>& _connectorLabels = vector<string>(),
        bool _computeAllCCs = false);
    ConnectivityMetric(MPProblemType* _problem, XMLNode& _node, bool _computeAllCCs = false);

    virtual ~ConnectivityMetric() {}

    virtual void Print(ostream& _os) const;

    double operator()();

  private:
    ofstream m_history;
};

template<class MPTraits, class Set>
ConnectivityMetric<MPTraits, Set>::
ConnectivityMetric(const Set& _samples, const vector<string>& _connectorLabels, bool _computeAllCCs)
  : CoverageMetric<MPTraits, Set>(_samples, _connectorLabels, _computeAllCCs){
  this->SetName("ConnectivityMetric" + Set::GetName());
}

template<class MPTraits, class Set>
ConnectivityMetric<MPTraits, Set>::
ConnectivityMetric(MPProblemType* _problem, XMLNode& _node, bool _computeAllCCs)
  : CoverageMetric<MPTraits, Set>(_problem, _node, _computeAllCCs) {
    this->SetName("ConnectivityMetric" + Set::GetName());
}

template<class MPTraits, class Set>
void
ConnectivityMetric<MPTraits, Set>::
Print(ostream& _os) const {
  _os << "Percentage of queries solved" << endl;
}

template<class MPTraits, class Set>
double
ConnectivityMetric<MPTraits, Set>::
operator()() {
  CoverageMetric<MPTraits, Set>::
    operator()(); // Call CoverageMetric first

  static size_t numCalls = 0;
  if(numCalls == 0)
    m_history.open(this->GetBaseFilename() + ".connectivity");

  int numQueries = 0;
  size_t sz = this->m_connections.size();

  for(size_t i=0; i<sz; ++i)
    sort(this->m_connections[i].begin(), this->m_connections[i].end());
  for(size_t i=0; i<sz; ++i) {
    for(size_t j=i+1; j<sz; ++j) {
      vector<VID> intersection;
      set_intersection(
          this->m_connections[i].begin(), this->m_connections[i].end(),
          this->m_connections[j].begin(), this->m_connections[j].end(),
          back_insert_iterator<vector<VID> >(intersection));
      if(!(intersection.empty()))
        numQueries++;
    }
  }

  double connectivityAmt = (double(numQueries))/(double(sz*(sz-1))/2.0);
  m_history << numCalls++ << "\t" << connectivityAmt << endl;

  return connectivityAmt;
}

#endif
