#ifndef _COVERAGE_EVALUATION_H
#define _COVERAGE_EVALUATION_H

/////////////////////////
// evaluate the coverage of a given roadmap
#include "MapEvaluationMethod.h"

template<class CFG, class WEIGHT>
class CoverageMetric
{
  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

    CoverageMetric(vector<CFG> _s = vector<CFG>(), bool _computeAllCCs = false): 
      m_samples(_s), m_allData(_computeAllCCs){}
    virtual ~CoverageMetric(){}

    virtual double operator()(int _regionID, MPProblem* _mp, vector<string>& _nodeConnectionLabels){
      Roadmap<CFG, WEIGHT>* rmap = _mp->GetMPRegion(_regionID)->GetRoadmap();
      RoadmapGraph<CFG,WEIGHT>* pMap = rmap->m_pRoadmap;
      Connector<CFG, WEIGHT>* cm = _mp->GetMPStrategy()->GetConnector();

      //VID backupVID = rmap->m_pRoadmap->getVertIDs();

      m_connections = vector<vector<VID> >(m_samples.size());

      stapl::sequential::vector_property_map< RoadmapGraph<CFG, WEIGHT>,size_t > cmap;
      vector<pair<size_t,VID> > ccs;
      typename vector<pair<size_t,VID> >::iterator CC;
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

          for(vector<string>::iterator I = _nodeConnectionLabels.begin(); I != _nodeConnectionLabels.end(); ++I){
            typename Connector<CFG, WEIGHT>::ConnectionPointer connectionMethod = cm->GetMethod(*I);
            connectionMethod->Connect(rmap, Stats, cmap, sampleList.begin(), sampleList.end(), cc.begin(), cc.end());
          }

          if(pMap->get_out_degree(sampleVID)
              > degreeBefore){
            m_connections[i].push_back(CC->second);
            if(!m_allData)
              break;
          }
        }

        rmap->m_pRoadmap->delete_vertex(sampleVID);
      }

      //rmap->m_pRoadmap->setVertIDs(backupVID); 

      int numConnections = 0;
      for(size_t i=0; i<m_connections.size(); ++i){
        if(!(m_connections[i].empty()))
          numConnections++;
      }
      //cout << "Connection %: "<<((double)numConnections)/((double)m_connections.size())<<endl;
      return ((double)numConnections)/((double)m_connections.size());
    }

  protected:
    //input
    vector<CFG> m_samples;
    bool m_allData;

    //storage
    vector<vector<VID> > m_connections;
};


template <class CFG, class WEIGHT> 
class CoverageEvaluation : public MapEvaluationMethod {
  public:

    CoverageEvaluation(vector<CFG>& _s, double _t, vector<string> _nodeConnection, bool _computeAllCCs = false) 
      : m_samples(_s), m_threshold(_t), m_nodeConnectionLabels(_nodeConnection), m_allData(_computeAllCCs){
        this->SetName("CoverageEvaluator");
      }

    CoverageEvaluation(XMLNodeReader& _node, MPProblem* _problem, bool _defaultAllData = false)
      : MapEvaluationMethod(_node, _problem) {
        this->SetName("CoverageEvaluator");

        m_threshold = _node.numberXMLParameter("threshold", false, 0.8, 0.0, 1.0, "percentage coverage required to pass evaluation");

        m_filename = _node.stringXMLParameter("samples", true, "", "filename containing witness samples");
        //read in samples
        m_samples.clear();
        ifstream is(m_filename.c_str());
        copy(istream_iterator<CFG>(is), istream_iterator<CFG>(), back_insert_iterator<vector<CFG> >(m_samples));
        is.close();

        m_allData = _node.boolXMLParameter("compute_all_ccs", false, _defaultAllData, 
            "flag when set to true computes coverage to all ccs, not just the first connectable cc");

        m_nodeConnectionLabels.clear();
        for(XMLNodeReader::childiterator I = _node.children_begin(); I != _node.children_end(); ++I)
        {
          if(I->getName() == "node_connection_method"){
            m_nodeConnectionLabels.push_back(I->stringXMLParameter("Method", true, "", "connection method label"));
            I->warnUnrequestedAttributes();
          }
          else{
            I->warnUnknownNode();
            exit(-1);
          }
        }
        if(m_nodeConnectionLabels.empty()){
          //instead, would be nice to read labels from the strategy in which it is used... fix later
          cerr << "CoverageEvaluation::ParseXML(): you must specify at least one node connection method for this evaluator.\n";
          exit(-1);
        }
      }

    virtual void PrintOptions(ostream& _os){
      _os << this->GetName() << "::\n";
      _os << "\twitness samples from \"" << m_filename << "\"\n";
      _os << "\tthreshold = " << m_threshold << endl;
      _os << "\tall_data = " << m_allData << endl;
      _os << "\tnode_connection_labels = ";  copy(m_nodeConnectionLabels.begin(),
          m_nodeConnectionLabels.end(), ostream_iterator<string>(_os, " "));  _os << endl;
    }

    virtual bool operator() (){
      return operator()(this->GetMPProblem()->CreateMPRegion());
    }

    virtual bool operator() (int _regionID) {
      CoverageMetric<CFG, WEIGHT> coverage(m_samples, m_allData);
      return (coverage(_regionID, GetMPProblem(), m_nodeConnectionLabels) >= m_threshold);
    }

  protected:
    //input
    vector<CFG> m_samples;
    double m_threshold;
    vector<string> m_nodeConnectionLabels;
    bool m_allData;
    string m_filename;
};

#endif
