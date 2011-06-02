#include "MPStrategy/SmoothQueryStrategy.h"
#include "MPStrategy/MPStrategy.h"

#include "boost/lambda/lambda.hpp"


SmoothQueryStrategy::
SmoothQueryStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
  : QueryStrategy(in_Node, in_pProblem, false) 
{
    LOG_DEBUG_MSG("SmoothQueryStrategy::SmoothQueryStrategy()");
    ParseXML(in_Node); 
    /*
    query.ReadQuery(m_strQueryFileLabel.c_str());
    query.outputPathFile = new char[strlen(m_strPathFileLabel.c_str())+1];
    strcpy(query.outputPathFile, m_strPathFileLabel.c_str());
    */
    LOG_DEBUG_MSG("~SmoothQueryStrategy::SmoothQueryStrategy()");
}

SmoothQueryStrategy::
~SmoothQueryStrategy()
{}

void
SmoothQueryStrategy::
PrintOptions(ostream& out_os)
{
  using boost::lambda::_1;

  QueryStrategy::PrintOptions(out_os);
  
  out_os << "\tsmoothed_path_file = " << m_strSmoothPathFileLabel << endl;
  if(m_vecStrSmoothNodeConnectionLabels.empty())
    out_os << "\tsmooth_node_connection_methods: AllPairs (default)";
  else
  {
    out_os << "\tsmooth_node_connection_methods: ";
    for_each(m_vecStrSmoothNodeConnectionLabels.begin(), m_vecStrSmoothNodeConnectionLabels.end(), cout << _1 << " ");
  }
}

void
SmoothQueryStrategy::
ParseXML(XMLNodeReader& in_Node) 
{
  LOG_DEBUG_MSG("SmoothQueryStrategy::ParseXML()");

  QueryStrategy::ParseXML(in_Node, false);
  
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) 
  {
    if(citr->getName() == "smooth_path_file")
    {
      m_strSmoothPathFileLabel = citr->stringXMLParameter(string("name"), true, string(""), string("Smoothed Path Filename"));
      citr->warnUnrequestedAttributes();
    }    
    else if(citr->getName() == "smooth_node_connection_method") 
    {
      string connect_method = citr->stringXMLParameter(string("Method"), true, string(""), string("Smoothing Node Connection Method"));
      m_vecStrSmoothNodeConnectionLabels.push_back(connect_method);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName()=="dm_method")
    {
      dm_label =citr->stringXMLParameter(string("Method"),true,string(""),string("Distance Metric"));
      citr->warnUnrequestedAttributes();
    }
    else 
    {
      string str = citr->getName();
      if(str != "map_file" && str != "query_file" && str != "path_file" && str != "node_connection_method")
        citr->warnUnknownNode();
    }
  }
  LOG_DEBUG_MSG("~SmoothQueryStrategy::ParseXML()");
}

void
SmoothQueryStrategy::
Run(int in_RegionID) 
{

    GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap()->ReadRoadmapGRAPHONLY(m_strMapFileLabel.c_str());
  LOG_DEBUG_MSG("SmoothQueryStrategy::()");

  cout << "SmoothQueryStrategy::\n";
  PrintOptions(cout);

  OBPRM_srand(getSeed()); 
  
  Roadmap<CfgType,WeightType>* rdmp = GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap();
  Stat_Class* pStatClass = GetMPProblem()->GetMPRegion(in_RegionID)->GetStatClass();
  
  vector< ConnectMap<CfgType, WeightType>::NodeConnectionPointer > methods;
    
  if(m_vecStrNodeConnectionLabels.empty()) {
    methods.push_back(ConnectMap<CfgType, WeightType>::NodeConnectionPointer(new NeighborhoodConnection<CfgType, WeightType>(1, 1, false, true, false)));
  }
  else
    for(vector<string>::iterator I = m_vecStrNodeConnectionLabels.begin(); I != m_vecStrNodeConnectionLabels.end(); ++I)
      methods.push_back(GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*I));


  //perform query
  Clock_Class QueryClock;
  QueryClock.StartClock("Query");

  bool query_result = query.PerformQuery(rdmp, *pStatClass, 
                       &m_ConnectMap, 
                       &methods,
                       GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                       GetMPProblem()->GetDistanceMetric()->GetDMMethod(dm_label));
                                         
  QueryClock.StopPrintClock();
  
  if(query_result)
  {
    query.WritePath(rdmp);
    cout << "\tSUCCESSFUL query\n";

    vector< ConnectMap<CfgType, WeightType>::NodeConnectionPointer > methods;

    if(m_vecStrSmoothNodeConnectionLabels.empty()) {
      // all pairs
      methods.push_back(ConnectMap<CfgType, WeightType>::NodeConnectionPointer(new NeighborhoodConnection<CfgType, WeightType>(0)));
    }
    else
      for(vector<string>::iterator I = m_vecStrSmoothNodeConnectionLabels.begin(); I != m_vecStrSmoothNodeConnectionLabels.end(); ++I)
        methods.push_back(GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*I));

    //redo connection among nodes in path
    Clock_Class SmoothClock;
    SmoothClock.StartClock("Smooth Path");
    vector<VID> path_vids;
    for(vector<CfgType>::iterator I = query.path.begin(); I != query.path.end(); ++I)
      if(rdmp->m_pRoadmap->IsVertex(*I))
        path_vids.push_back(rdmp->m_pRoadmap->GetVID(*I));
        
    vector< ConnectMap<CfgType, WeightType>::NodeConnectionPointer >::iterator itr;
    for (itr = methods.begin(); itr != methods.end(); itr++){
      m_SmoothConnectMap.ConnectNodes(
                           *itr,
                           rdmp, *pStatClass,
                           GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                           false, false,
                           path_vids.begin(), path_vids.end(),
                           path_vids.begin(), path_vids.end());
    }
    SmoothClock.StopPrintClock();

    //reperform query
    Clock_Class SmoothQueryClock;
    SmoothQueryClock.StartClock("Query Smoothed Path");
    query.path.clear();
    bool smooth_query_result = query.PerformQuery(rdmp, *pStatClass,
                                                  &m_ConnectMap,
                                                  &methods,
                                                  GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                                  GetMPProblem()->GetDistanceMetric()->GetDMMethod(dm_label));
    SmoothQueryClock.StopPrintClock();

    //output smoothed path
    if(smooth_query_result)
    {
      char* str = new char[strlen(m_strSmoothPathFileLabel.c_str()+1)];
      strcpy(str, m_strSmoothPathFileLabel.c_str());
      query.WritePath(rdmp, str);
      delete str;
    }
  }
  else
    cout << "\tUNSUCCESSFUL query\n";


  LOG_DEBUG_MSG("~SmoothQueryStrategy::()");
}

