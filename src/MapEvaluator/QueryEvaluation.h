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
  //LocalPlanners<CFG, WEIGHT>* _lp,
  string _m_lp_label,
  vector<string> vecStrNodes,
  ConnectMap<CFG, WEIGHT> ConnectMap,
  shared_ptr<DistanceMetricMethod> _dm) : 
  MapEvaluationMethod(),
  m_query_filename(filename),
  m_query_pathname(filename),
  m_query(query),
  m_stats(stats),
		//lp(_lp),
  m_lp_label(_m_lp_label),
  m_vecStrNodeConnectionLabels(vecStrNodes),
  m_ConnectMap(ConnectMap),
  dm(_dm),
  intermediateFiles(false)
  {
    this->SetName("QueryEvaluator");
  }

  QueryEvaluation(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
  : MapEvaluationMethod(in_Node, in_pProblem)
  {
    this->SetName("QueryEvaluator");
    //lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners(); //later change to have own lp

    m_query_filename = in_Node.stringXMLParameter("filename", true, "", "Query Filename");
    m_query_pathname = in_Node.stringXMLParameter("outpath", false, "", "Query output path filename");
    m_query = Query<CFG, WEIGHT>(m_query_filename.c_str());
    
    string dm_label = in_Node.stringXMLParameter("dm_method", false, "default", "Distance Metric Method");
    m_lp_label = in_Node.stringXMLParameter("lp_method", true, "", "Local Planner Method");
    dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label);

    intermediateFiles = in_Node.boolXMLParameter("intermediate_files", false, false, "Determines output of intermediate file mapnodes.path");

    for (XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr)
      if (citr->getName() == "node_connection_method")
      {
        string connect_method = citr->stringXMLParameter("Method", true, "", "Node Connection Method");
        m_vecStrNodeConnectionLabels.push_back(connect_method);
        citr->warnUnrequestedAttributes();
      } else
        citr->warnUnknownNode();
  }
  virtual ~QueryEvaluation() {}

  virtual void PrintOptions(ostream& out_os);

  virtual bool operator() ()
  {
    return operator()(GetMPProblem()->CreateMPRegion());
  }
  virtual bool operator() (int in_RegionID);
  
 private:
  string m_query_filename;
  string m_query_pathname;
  Query<CFG, WEIGHT> m_query;
  Stat_Class m_stats;
  //LocalPlanners<CFG, WEIGHT>* lp;
  string m_lp_label;
  vector<string> m_vecStrNodeConnectionLabels;
  ConnectMap<CFG, WEIGHT> m_ConnectMap;
  shared_ptr<DistanceMetricMethod >dm;
  bool intermediateFiles;
};


template <class CFG, class WEIGHT>
void
QueryEvaluation<CFG, WEIGHT>::PrintOptions(ostream& out_os)
{
  using boost::lambda::_1;
  out_os << this->GetName() << "::";
  out_os << "\n\tquery file = \'" << m_query_filename << "\'";
  out_os << "\n\tpath file = \'" << m_query_pathname << "\'";
  out_os << "\n\tdistance metric = "; 
  dm->PrintOptions(out_os);
  out_os << "\tlocal planner = " << m_lp_label << endl;
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
  //lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners(); //later change to have own lp
  //VID oriVertID = rmap->m_pRoadmap->getVertIDs(); //save vertexID counter
 
  
  vector<bool> already_in_roadmap;
  for(typename vector<CFG>::iterator I = m_query.query.begin(); I != m_query.query.end(); ++I)
    already_in_roadmap.push_back(rmap->m_pRoadmap->IsVertex(*I));

  bool queryResult = m_query.PerformQuery(rmap, m_stats, 
                        &m_ConnectMap, &methods,
											 GetMPProblem()->GetMPStrategy()->GetLocalPlanners(), 
                       m_lp_label,
                       dm, intermediateFiles);
  if(queryResult==true && m_query_pathname.length()>0){
     cout<<"in query Eval";
     char path_filename[100];
     sprintf(path_filename, "%s.path", m_query_pathname.c_str());
     
     m_query.WritePath(rmap,path_filename );
  }
  for(size_t i=0; i<already_in_roadmap.size(); ++i)
    if(!already_in_roadmap[i])
      rmap->m_pRoadmap->delete_vertex(rmap->m_pRoadmap->GetVID(m_query.query[i])); //deleted added node from rmap
  //rmap->m_pRoadmap->setVertIDs(oriVertID); //reset vertex ID counter
  return queryResult;
}

#endif
