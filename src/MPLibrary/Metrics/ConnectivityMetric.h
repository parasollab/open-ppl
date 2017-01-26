#ifndef CONNECTIVITY_METRIC_H
#define CONNECTIVITY_METRIC_H

#include "CoverageMetric.h"

////////////////////////////////////////////////////////////////////////////////
/// @TODO
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits, class Set>
class ConnectivityMetric : public CoverageMetric<MPTraits, Set> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID      VID;

    ///@}
    ///@name Construction
    ///@{

    ConnectivityMetric(const Set& _samples = Set(),
        const vector<string>& _connectorLabels = vector<string>(),
        bool _computeAllCCs = false);

    ConnectivityMetric(XMLNode& _node, bool _computeAllCCs = false);

    virtual ~ConnectivityMetric() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Metric Interface
    ///@{

    virtual double operator()() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    ofstream m_history;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template<class MPTraits, class Set>
ConnectivityMetric<MPTraits, Set>::
ConnectivityMetric(const Set& _samples, const vector<string>& _connectorLabels,
    bool _computeAllCCs) :
    CoverageMetric<MPTraits, Set>(_samples, _connectorLabels, _computeAllCCs) {
  this->SetName("ConnectivityMetric" + Set::GetName());
}


template<class MPTraits, class Set>
ConnectivityMetric<MPTraits, Set>::
ConnectivityMetric(XMLNode& _node, bool _computeAllCCs) :
    CoverageMetric<MPTraits, Set>(_node, _computeAllCCs) {
  this->SetName("ConnectivityMetric" + Set::GetName());
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template<class MPTraits, class Set>
void
ConnectivityMetric<MPTraits, Set>::
Print(ostream& _os) const {
  _os << "Percentage of queries solved" << endl;
}

/*---------------------------- Metric Interface ------------------------------*/

template<class MPTraits, class Set>
double
ConnectivityMetric<MPTraits, Set>::
operator()() {
  CoverageMetric<MPTraits, Set>::operator()(); // Call CoverageMetric first

  static size_t numCalls = 0;
  if(numCalls == 0)
    m_history.open(this->GetBaseFilename() + ".connectivity");

  int numQueries = 0;
  size_t sz = this->m_connections.size();

  for(size_t i = 0; i < sz; ++i)
    sort(this->m_connections[i].begin(), this->m_connections[i].end());
  for(size_t i = 0; i < sz; ++i) {
    for(size_t j = i + 1; j < sz; ++j) {
      vector<VID> intersection;
      set_intersection(
          this->m_connections[i].begin(), this->m_connections[i].end(),
          this->m_connections[j].begin(), this->m_connections[j].end(),
          back_insert_iterator<vector<VID>>(intersection));
      if(!(intersection.empty()))
        ++numQueries;
    }
  }

  const double connectivityAmt = numQueries / ((sz * (sz - 1)) / 2.);
  m_history << ++numCalls << "\t" << connectivityAmt << endl;

  return connectivityAmt;
}

/*----------------------------------------------------------------------------*/

#endif
