#ifndef QUERYEVALUATION_H
#define QUERYEVALUATION_H

#include "MapEvaluationMethod.h"
#include "Query.h"
#include "boost/lambda/lambda.hpp"

template <class CFG, class WEIGHT>
class QueryEvaluation : public MapEvaluationMethod {
  public:

    QueryEvaluation();
    QueryEvaluation(string _filename, Query<CFG, WEIGHT> _query, StatClass _stats, string _lpLabel, 
		    vector<string> _vecStrNodes, Connector<CFG, WEIGHT> _connector, 
		    shared_ptr<DistanceMetricMethod> _dm, bool _intermediateFiles = false);
    QueryEvaluation(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~QueryEvaluation() {}

    virtual void PrintOptions(ostream& _os); 

    virtual bool operator()() {
      return operator()(GetMPProblem()->CreateMPRegion());
    }
    virtual bool operator()(int _regionID); 

  protected:
    string m_queryFilename;
    string m_queryPathname;
    Query<CFG, WEIGHT> m_query;
    StatClass m_stats;
    string m_lpLabel;
    vector<string> m_nodeConnectionLabels;
    Connector<CFG, WEIGHT> m_connector;
    shared_ptr<DistanceMetricMethod> m_dm;
    bool m_intermediateFiles;
};

template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::QueryEvaluation() {
  this->SetName("QueryEvaluation");
}

template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::QueryEvaluation(string _filename, Query<CFG, WEIGHT> _query, StatClass _stats, string _lpLabel,
					      vector<string> _vecStrNodes, Connector<CFG, WEIGHT> _connector,
					      shared_ptr<DistanceMetricMethod> _dm, bool _intermediateFiles)
  : MapEvaluationMethod(), m_queryFilename(_filename), m_queryPathname(_filename), m_query(_query), m_stats(_stats), m_lpLabel(_lpLabel),
    m_nodeConnectionLabels(_vecStrNodes), m_connector(_connector), m_dm(_dm), m_intermediateFiles(_intermediateFiles) {
  this->SetName("QueryEvaluation");
}

template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::QueryEvaluation(XMLNodeReader& _node, MPProblem* _problem)
  : MapEvaluationMethod(_node, _problem) {
  this->SetName("QueryEvaluation");

  m_queryFilename = _node.stringXMLParameter("filename", true, "", "Query Filename");
  m_queryPathname = _node.stringXMLParameter("outpath", false, "", "Query output path filename");
  m_query = Query<CFG, WEIGHT>(m_queryFilename.c_str());

  m_lpLabel = _node.stringXMLParameter("lp_method", true, "", "Local Planner Method");
  m_dm = _problem->GetDistanceMetric()->GetMethod(_node.stringXMLParameter("dm_method", false, "default", "Distance Metric Method"));

  m_intermediateFiles = _node.boolXMLParameter("intermediate_files", false, false, "Determines output of intermediate file mapnodes.path");

  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
    if(citr->getName() == "node_connection_method") {
      m_nodeConnectionLabels.push_back(citr->stringXMLParameter("Method", true, "", "Node Connection Method"));
      citr->warnUnrequestedAttributes();
    }
    else
      citr->warnUnknownNode();
  }

  if(m_nodeConnectionLabels.empty()) {
    cerr << "\n\nError in QueryEvaluation XML constructor:: no node connection methods specified.\n\tUntil NeighborhoodFinder class can support a default/empty string as input, node connection methods must be explicitly specified.\n\tExiting.\n";
    exit(-1);
  }
  if(m_debug) PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void QueryEvaluation<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  using boost::lambda::_1;
   _os << this->GetName() << "::";
   _os << "\n\tquery file = \'" << m_queryFilename << "\'";
   _os << "\n\tpath file = \'" << m_queryPathname << "\'";
   _os << "\n\t distance metric = ";
   m_dm->PrintOptions(_os);
   _os << "\tlocal planner = " << m_lpLabel << endl;
}

template <class CFG, class WEIGHT>
bool QueryEvaluation<CFG, WEIGHT>::operator()(int _regionID) {
  PrintOptions(cout);

  vector<Connector<CfgType, WeightType>::ConnectionPointer > methods;

  if(m_nodeConnectionLabels.empty()) {
    methods.push_back(Connector<CfgType, WeightType>::ConnectionPointer(new NeighborhoodConnection<CfgType, WeightType>("", "", 1, 1, false, true, false)));
  }
  else {
    for(vector<string>::iterator I = m_nodeConnectionLabels.begin(); I != m_nodeConnectionLabels.end(); ++I)
    methods.push_back(GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*I));
  }
  
  Roadmap<CFG, WEIGHT>* rmap = GetMPProblem()->GetMPRegion(_regionID)->GetRoadmap();

  vector<bool> alreadyInRoadmap;
  for(typename vector<CFG>::iterator I = m_query.query.begin(); I != m_query.query.end(); ++I)
    alreadyInRoadmap.push_back(rmap->m_pRoadmap->IsVertex(*I));

  bool queryResult = m_query.PerformQuery(rmap, m_stats, &m_connector, &methods,
                                          GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                          m_lpLabel, m_dm, m_intermediateFiles);
  if(queryResult==true && m_queryPathname.length()>0) {
    cout << "in query Eval";
    ostringstream pathFilename;
    pathFilename << m_queryPathname.c_str() << ".path";
    m_query.WritePath(rmap, (char*)pathFilename.str().c_str());
  }
  for(size_t i=0; i<alreadyInRoadmap.size(); ++i) {
    if(!alreadyInRoadmap[i])
      rmap->m_pRoadmap->delete_vertex(rmap->m_pRoadmap->GetVID(m_query.query[i]));  //deleted added node from rmap
  }

  return queryResult;
}

#endif
