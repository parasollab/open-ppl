#ifndef COVERAGEMETRIC_H
#define COVERAGEMETRIC_H

#include "MetricsMethod.h"
#include "Connector.h"

template <class CFG, class WEIGHT>
class CoverageMetric : public MetricsMethod {
  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

    CoverageMetric();
    CoverageMetric(vector<CFG>& _samples, vector<string> _nodeConnection, bool _computeAllCCs = false);
    CoverageMetric(XMLNodeReader& _node, MPProblem* _problem, bool _computeAllCCs = false);
    virtual ~CoverageMetric() {}

    virtual void PrintOptions(ostream& _os);

    virtual double operator()();

  protected:
    //input
    vector<CFG> m_samples;
    vector<string> m_nodeConnectionLabels;
    bool m_allData;
    string m_filename, m_outFileName;
    vector<vector<VID> > m_connections;
    ofstream output;
};

template <class CFG, class WEIGHT>
CoverageMetric<CFG, WEIGHT>::CoverageMetric() {
  this->SetName("CoverageMetric");
}

template <class CFG, class WEIGHT>
CoverageMetric<CFG, WEIGHT>::CoverageMetric(vector<CFG>& _samples, vector<string> _nodeConnection, bool _computeAllCCs)
  : m_samples(_samples), m_nodeConnectionLabels(_nodeConnection), m_allData(_computeAllCCs) {
  this->SetName("CoverageMetric");
}

template <class CFG, class WEIGHT>
CoverageMetric<CFG, WEIGHT>::CoverageMetric(XMLNodeReader& _node, MPProblem* _problem, bool _computeAllCCs)
  : MetricsMethod(_node, _problem) {
    this->SetName("CoverageMetric");

    m_filename = _node.stringXMLParameter("filename", true, "", "filename containing witness samples");
    m_outFileName = _node.stringXMLParameter("outfilename", true, "", "filename for recording results");
    //read in samples
    m_samples.clear();
    ifstream is(m_filename.c_str());
    copy(istream_iterator<CFG>(is), istream_iterator<CFG>(), back_insert_iterator<vector<CFG> >(m_samples));
    is.close();

    output.open((m_outFileName+".coverage").c_str());

    m_allData = _node.boolXMLParameter("computeAllCCs", false, _computeAllCCs, "flag when set to true computes coverage to all ccs, not just the first connectable cc");

    m_nodeConnectionLabels.clear();
    for(XMLNodeReader::childiterator I = _node.children_begin(); I != _node.children_end(); ++I) {
      if(I->getName() == "NodeConnectionMethod") {
        m_nodeConnectionLabels.push_back(I->stringXMLParameter("method", true, "", "connection method label"));
        I->warnUnrequestedAttributes();
      }
      else {
        I->warnUnknownNode();
        exit(-1);
      }
    }
    if(m_nodeConnectionLabels.empty()) {
      cerr << "CoverageMetric: you must specify at lease one node connection method.\n";
      exit(-1);
    }

    if(m_debug) PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void CoverageMetric<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "Percentage of connection" << endl;
  _os << "\twitness samples from \"" << m_filename << "\"\n";
  _os << "\tall_data = " << m_allData << endl;
  _os << "\tnode_connection_labels = ";
  copy(m_nodeConnectionLabels.begin(), m_nodeConnectionLabels.end(), ostream_iterator<string>(_os, " "));
  _os << endl;
}

template <class CFG, class WEIGHT>
double CoverageMetric<CFG, WEIGHT>::operator()() {
  static size_t numcalls = 0;

  Roadmap<CFG, WEIGHT>* rmap = GetMPProblem()->GetRoadmap();
  RoadmapGraph<CFG, WEIGHT>* pMap = rmap->m_pRoadmap;
  Connector<CFG, WEIGHT>* cn = GetMPProblem()->GetMPStrategy()->GetConnector();

  m_connections = vector<vector<VID> >(m_samples.size());

  stapl::sequential::vector_property_map< RoadmapGraph<CFG, WEIGHT>, size_t > cmap;
  vector<pair<size_t, VID> > ccs;
  typename vector<pair<size_t, VID> >::iterator CC;
  get_cc_stats(*pMap, cmap, ccs);

  vector<VID> sampleList, cc;
  StatClass Stats;

  for(size_t i=0; i<m_samples.size(); ++i) {
    VID sampleVID = rmap->m_pRoadmap->AddVertex(m_samples[i]);
    sampleList.clear();
    sampleList.push_back(sampleVID);

    for(CC = ccs.begin(); CC != ccs.end(); ++CC) {
      cc.clear();
      cmap.reset();
      get_cc(*pMap, cmap, CC->second, cc);

      size_t degreeBefore = pMap->get_out_degree(sampleVID);

      for(vector<string>::iterator I = m_nodeConnectionLabels.begin(); I != m_nodeConnectionLabels.end(); ++I) {
   	cn->GetMethod(*I)->Connect(rmap, Stats, cmap, sampleList.begin(), sampleList.end(), cc.begin(), cc.end());
      }

      if((pMap->get_out_degree(sampleVID)) > degreeBefore) {
        m_connections[i].push_back(CC->second);
        if(!m_allData)
          break;
      }
    }
    rmap->m_pRoadmap->delete_vertex(sampleVID);
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

