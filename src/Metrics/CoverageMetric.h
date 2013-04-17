#ifndef COVERAGEMETRIC_H
#define COVERAGEMETRIC_H

#include "MetricMethod.h"

template<class MPTraits>
class CoverageMetric : public MetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    CoverageMetric(const vector<CfgType>& _samples=vector<CfgType>(), 
        const vector<string>& _connectorLabels = vector<string>(), 
        bool _computeAllCCs = false);
    CoverageMetric(MPProblemType* _problem, XMLNodeReader& _node, bool _computeAllCCs = false);
    virtual ~CoverageMetric();

    virtual void PrintOptions(ostream& _os);

    double operator()();

  protected:
    //input
    vector<CfgType> m_samples;
    vector<string> m_connectorLabels;
    bool m_allData;
    string m_filename, m_outFileName;
    vector<vector<VID> > m_connections;
    ofstream output;
};

template<class MPTraits>
CoverageMetric<MPTraits>::CoverageMetric(const vector<CfgType>& _samples, const vector<string>& _connectorLabels, bool _computeAllCCs)
  : m_samples(_samples), m_connectorLabels(_connectorLabels), m_allData(_computeAllCCs) {
  this->SetName("CoverageMetric");
}

template<class MPTraits>
CoverageMetric<MPTraits>::CoverageMetric(MPProblemType* _problem, XMLNodeReader& _node, bool _computeAllCCs)
  : MetricMethod<MPTraits>(_problem, _node) {
    this->SetName("CoverageMetric");

    m_filename = _node.stringXMLParameter("filename", true, "", "filename containing witness samples");
    m_outFileName = _node.stringXMLParameter("outfilename", true, "", "filename for recording results");
    //read in samples
    m_samples.clear();
    RoadmapType covRdmp;
    covRdmp.Read(m_filename.c_str());
    covRdmp.GetGraph()->GetVerticesData(m_samples);

    output.open((m_outFileName+".coverage").c_str());

    m_allData = _node.boolXMLParameter("computeAllCCs", false, _computeAllCCs, "flag when set to true computes coverage to all ccs, not just the first connectable cc");

    m_connectorLabels.clear();
    for(XMLNodeReader::childiterator i = _node.children_begin(); i != _node.children_end(); ++i) {
      if(i->getName() == "Connector") {
        m_connectorLabels.push_back(i->stringXMLParameter("label", true, "", "connection method label"));
        i->warnUnrequestedAttributes();
      }
      else {
        i->warnUnknownNode();
        exit(-1);
      }
    }
    if(m_connectorLabels.empty()) {
      cerr << "CoverageMetric: you must specify at lease one node connection method.\n";
      exit(-1);
    }
}

template<class MPTraits>
CoverageMetric<MPTraits>::~CoverageMetric() {
}

template<class MPTraits>
void 
CoverageMetric<MPTraits>::PrintOptions(ostream& _os) {
  _os << "Percentage of connection" << endl;
  _os << "\twitness samples from \"" << m_filename << "\"\n";
  _os << "\tall_data = " << m_allData << endl;
  _os << "\tnode_connection_labels = ";
  copy(m_connectorLabels.begin(), m_connectorLabels.end(), ostream_iterator<string>(_os, " "));
  _os << endl;
}

template<class MPTraits>
double 
CoverageMetric<MPTraits>::operator()() {

  static size_t numcalls = 0;

  RoadmapType* rmap = this->GetMPProblem()->GetRoadmap();
  GraphType* rgraph = rmap->GetGraph();

  m_connections = vector<vector<VID> >(m_samples.size());

  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  vector<pair<size_t, VID> > ccs;
  typename vector<pair<size_t, VID> >::iterator ccit;
  get_cc_stats(*rgraph, cmap, ccs);

  vector<VID> sampleList, cc;
  StatClass stats;

  for(size_t i=0; i<m_samples.size(); ++i) {
    VID sampleVID = rgraph->AddVertex(m_samples[i]);
    sampleList.clear();
    sampleList.push_back(sampleVID);

    for(ccit = ccs.begin(); ccit != ccs.end(); ++ccit) {
      cc.clear();
      cmap.reset();
      get_cc(*rgraph, cmap, ccit->second, cc);

      size_t degreeBefore = rgraph->get_out_degree(sampleVID);

      for(vector<string>::iterator sit = m_connectorLabels.begin(); sit != m_connectorLabels.end(); ++sit) {
        this->GetMPProblem()->GetConnector(*sit)->Connect(rmap, stats, cmap, sampleList.begin(), sampleList.end(), cc.begin(), cc.end());
      }

      if((rgraph->get_out_degree(sampleVID)) > degreeBefore) {
        m_connections[i].push_back(ccit->second);
        if(!m_allData)
          break;
      }
    }
    rgraph->delete_vertex(sampleVID);
  }
  int numConnections = 0;
  for(size_t i=0; i<m_connections.size(); ++i) {
    if(!(m_connections[i].empty()))
      numConnections++;
  }

  double coverageAmt = ((double)numConnections)/((double)m_connections.size());
  output << numcalls++ << "\t" << coverageAmt << endl;

  return coverageAmt;
}

#endif

