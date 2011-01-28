#ifndef _QUERY_EVALUATION_H
#define _QUERY_EVALUATION_H

#include "Query.h"
#include "boost/lambda/lambda.hpp"

template <class CFG, class WEIGHT>
class QueryEvaluation 
  : public MapEvaluationMethod 
{
 public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  QueryEvaluation(
  string filename,
  Query<CFG, WEIGHT> query,
  Stat_Class stats,
  LocalPlanners<CFG, WEIGHT>* _lp,
  vector<string> vecStrNodes,
  ConnectMap<CFG, WEIGHT> ConnectMap,
  shared_ptr<DistanceMetricMethod> _dm) : 
  MapEvaluationMethod(),
  m_query_filename(filename),
  m_query(query),
  m_stats(stats),
  lp(_lp),
  m_vecStrNodeConnectionLabels(vecStrNodes),
  m_ConnectMap(ConnectMap),
  dm(_dm)
  {}


  QueryEvaluation(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
  : MapEvaluationMethod(in_Node, in_pProblem)
  {
    //lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners(); //later change to have own lp

    m_query_filename = in_Node.stringXMLParameter("filename", true, string(""), string("Query Filename"));
    m_query = Query<CFG, WEIGHT>(m_query_filename.c_str());
    
    string dm_label = in_Node.stringXMLParameter(string("dm_method"), false, string("default"), string("Distance Metric Method"));
    dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label);

    for (XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr)
      if (citr->getName() == "node_connection_method")
      {
        string connect_method = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
        m_vecStrNodeConnectionLabels.push_back(connect_method);
        citr->warnUnrequestedAttributes();
      } else
        citr->warnUnknownNode();
  }
  virtual ~QueryEvaluation() {}

  virtual char* GetName() const {return "query evaluation"; }
  virtual void PrintOptions(ostream& out_os);

  virtual bool operator() ()
  {
    return operator()(GetMPProblem()->CreateMPRegion());
  }
  virtual bool operator() (int in_RegionID);
  
 private:
  string m_query_filename;
  Query<CFG, WEIGHT> m_query;
  Stat_Class m_stats;
  LocalPlanners<CFG, WEIGHT>* lp;
  vector<string> m_vecStrNodeConnectionLabels;
  ConnectMap<CFG, WEIGHT> m_ConnectMap;
  shared_ptr<DistanceMetricMethod >dm ;
};


template <class CFG, class WEIGHT>
void
QueryEvaluation<CFG, WEIGHT>::PrintOptions(ostream& out_os)
{
  using boost::lambda::_1;
  out_os << GetName() << "::";
  out_os << "\n\tquery file = \'" << m_query_filename << "\'";
  out_os << "\n\t"; GetMPProblem()->GetCollisionDetection()->PrintOptions(out_os);
  if(m_vecStrNodeConnectionLabels.empty())
    out_os << "\tnode_connection_methods: ConnectFirst (default)";
  else
  {
    out_os << "\tnode_connection_methods: "; for_each(m_vecStrNodeConnectionLabels.begin(), m_vecStrNodeConnectionLabels.end(), cout << _1 << " ");
  }
  out_os << "\n\t"; GetMPProblem()->GetDistanceMetric()->PrintOptions(out_os);
  out_os << "\tLocalPlanners "; GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->PrintOptions(out_os);
}


template <class CFG, class WEIGHT>
bool
QueryEvaluation<CFG, WEIGHT>::operator() (int in_RegionID)
{
  PrintOptions(cout);

  vector< ConnectMap<CfgType, WeightType>::NodeConnectionPointer > methods;
    
  if(m_vecStrNodeConnectionLabels.empty()) {
    methods.push_back(ConnectMap<CfgType, WeightType>::NodeConnectionPointer(new NeighborhoodConnection<CfgType, WeightType>(1, 1, false, true, false)));
  }
  else
    for(vector<string>::iterator I = m_vecStrNodeConnectionLabels.begin(); I != m_vecStrNodeConnectionLabels.end(); ++I)
      methods.push_back(GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*I));


  Roadmap<CFG, WEIGHT>* rmap = GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap();
  lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners(); //later change to have own lp

  //VID oriVertID = rmap->m_pRoadmap->getVertIDs(); //save vertexID counter
  bool queryResult = m_query.PerformQuery(rmap, m_stats, 
                        &m_ConnectMap, 
                        &methods,
                        GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                        dm);
  
  for(typename vector<CFG>::iterator I = m_query.query.begin(); I != m_query.query.end(); ++I)
    //rmap->m_pRoadmap->DeleteVertex(*I); //deleted added node from rmap
    rmap->m_pRoadmap->delete_vertex(rmap->m_pRoadmap->GetVID(*I)); //deleted added node from rmap
  
  //rmap->m_pRoadmap->setVertIDs(oriVertID); //reset vertex ID counter

  return queryResult;
}

#endif
