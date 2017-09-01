#ifndef COVERAGE_METRIC_H
#define COVERAGE_METRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @TODO
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits, class Set>
class CoverageMetric : public MetricMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    ///@}
    ///@name Construction
    ///@{

    CoverageMetric(const Set& _samples = Set(),
        const vector<string>& _connectorLabels = vector<string>(),
        bool _computeAllCCs = false);

    CoverageMetric(XMLNode& _node, bool _computeAllCCs = false);

    virtual ~CoverageMetric() = default;

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
    vector<string> m_connectorLabels;
    bool m_allData;
    vector<vector<VID> > m_connections;
    ofstream m_history;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template<class MPTraits, class Set>
CoverageMetric<MPTraits, Set>::
CoverageMetric(const Set& _samples, const vector<string>& _connectorLabels,
    bool _computeAllCCs) : m_samples(_samples),
    m_connectorLabels(_connectorLabels), m_allData(_computeAllCCs) {
  this->SetName("CoverageMetric" + Set::GetName());
}


template<class MPTraits, class Set>
CoverageMetric<MPTraits, Set>::
CoverageMetric(XMLNode& _node, bool _computeAllCCs) :
    MetricMethod<MPTraits>(_node), m_samples(_node) {
  this->SetName("CoverageMetric" + Set::GetName());

  m_allData = _node.Read("computeAllCCs", false, _computeAllCCs,
      "Flag when set to true computes coverage to all ccs, "
      "not just the first connectable cc");

  m_connectorLabels.clear();
  for(auto& child : _node)
    if(child.Name() == "Connector")
      m_connectorLabels.push_back(
          child.Read("label", true, "", "connection method label"));

  if(m_connectorLabels.empty())
    throw ParseException(_node.Where(),
        "Please specify at least one node connection method.");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template<class MPTraits, class Set>
void
CoverageMetric<MPTraits, Set>::
Print(ostream& _os) const {
  _os << "Percentage of connection" << endl
      << "\tall_data = " << m_allData << endl
      << "\tnode_connection_labels = ";
  copy(m_connectorLabels.begin(), m_connectorLabels.end(),
      ostream_iterator<string>(_os, " "));
  _os << endl;
}

/*---------------------------- Metric Interface ------------------------------*/

template<class MPTraits, class Set>
double
CoverageMetric<MPTraits, Set>::
operator()() {
  static size_t numCalls = 0;
  if(numCalls == 0)
    m_history.open(this->GetBaseFilename() + ".coverage");

  RoadmapType* rmap = this->GetRoadmap();
  GraphType* rgraph = rmap->GetGraph();

  m_connections = vector<vector<VID> >(m_samples.size());

  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  vector<pair<size_t, VID> > ccs;
  typename vector<pair<size_t, VID> >::iterator ccit;
  get_cc_stats(*rgraph, cmap, ccs);

  vector<VID> sampleList, cc;
  StatClass stats;

  // Temporarily disable roadmap hooks while we make some test vertices and
  // edges.
  rgraph->DisableHooks();
  int index = 0;
  for(auto i = m_samples.begin(); i != m_samples.end(); ++i) {
    VID sampleVID = rgraph->AddVertex(*i);
    sampleList.clear();
    sampleList.push_back(sampleVID);

    for(ccit = ccs.begin(); ccit != ccs.end(); ++ccit) {
      cc.clear();
      cmap.reset();
      get_cc(*rgraph, cmap, ccit->second, cc);

      size_t degreeBefore = rgraph->get_out_degree(sampleVID);

      for(auto sit = m_connectorLabels.begin(); sit != m_connectorLabels.end();
          ++sit)
        this->GetConnector(*sit)->Connect(rmap,
            sampleList.begin(), sampleList.end(),
            cc.begin(), cc.end(), false);

      if((rgraph->get_out_degree(sampleVID)) > degreeBefore) {
        m_connections[index].push_back(ccit->second);
        if(!m_allData)
          break;
      }
    }
    rgraph->DeleteVertex(sampleVID);
    index++;
  }
  rgraph->EnableHooks();
  int numConnections = 0;
  for(size_t i=0; i<m_connections.size(); ++i) {
    if(!(m_connections[i].empty()))
      numConnections++;
  }

  double coverageAmt = ((double)numConnections)/((double)m_connections.size());
  m_history << numCalls++ << "\t" << coverageAmt << endl;

  return coverageAmt;
}

/*----------------------------------------------------------------------------*/

#endif
