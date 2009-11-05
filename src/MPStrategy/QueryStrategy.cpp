#include "MPStrategy/QueryStrategy.h"
#include "MPStrategy/MPStrategy.h"

QueryStrategy::
QueryStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml)
 : MPStrategyMethod(in_Node,in_pProblem) 
{
  LOG_DEBUG_MSG("QueryStrategy::QueryStrategy()");
  if(parse_xml)
    ParseXML(in_Node); 
  LOG_DEBUG_MSG("~QueryStrategy::QueryStrategy()");
}

QueryStrategy::
~QueryStrategy() 
{}

void 
QueryStrategy::
PrintOptions(ostream& out_os) 
{
  using boost::lambda::_1;
  out_os << "QueryStrategy::";
  out_os << "\n\tmap file = \"" << m_strMapFileLabel << "\"";
  out_os << "\n\tquery file = \"" << m_strQueryFileLabel << "\"";
  out_os << "\n\tpath file = \"" << m_strPathFileLabel << "\"";
  out_os << "\n\t"; GetMPProblem()->GetCollisionDetection()->PrintOptions(out_os);
  if(m_vecStrNodeConnectionLabels.empty())
    out_os << "\tnode_connection_methods: ConnectFirst (default)";
  else
  {  
    out_os << "\tnode_connection_methods: ";
    for_each(m_vecStrNodeConnectionLabels.begin(), m_vecStrNodeConnectionLabels.end(), cout << _1 << " ");
  }
  out_os << "\n\t"; GetMPProblem()->GetDistanceMetric()->PrintOptions(out_os);
  out_os << "\tLocalPlanners "; GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->PrintOptions(out_os);
}
 
void 
QueryStrategy::
ParseXML(XMLNodeReader& in_Node, bool warn_unknown) 
{
  LOG_DEBUG_MSG("QueryStrategy::ParseXML()");
  
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) 
  {
    if(citr->getName() == "node_connection_method") 
    {
      string connect_method = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
      m_vecStrNodeConnectionLabels.push_back(connect_method);
      citr->warnUnrequestedAttributes();
    } 
    /*
    else if(citr->getName() == "lp_method") 
    {
      m_strLocalPlannerLabel = citr->stringXMLParameter(string("Method"), true, string(""), string("Local Planner Method"));
      citr->warnUnrequestedAttributes();
    }
    */
    else if(citr->getName() == "map_file")
    {
      m_strMapFileLabel = citr->stringXMLParameter(string("name"), true, string(""), string("Map Filename"));
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "query_file")
    {
      m_strQueryFileLabel = citr->stringXMLParameter(string("name"), true, string(""), string("Query Filename"));
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "path_file")
    {
      m_strPathFileLabel = citr->stringXMLParameter(string("name"), true, string(""), string("Path Filename"));
      citr->warnUnrequestedAttributes();
    }
    else 
    {
      if(warn_unknown)
        citr->warnUnknownNode();
    }
  }
  
  query.ReadQuery(m_strQueryFileLabel.c_str());
  query.outputPathFile = new char[strlen(m_strPathFileLabel.c_str())+1];
  strcpy(query.outputPathFile, m_strPathFileLabel.c_str());
  
  LOG_DEBUG_MSG("~QueryStrategy::ParseXML()");
}
   
void 
QueryStrategy::
operator()(int in_RegionID) 
{
  LOG_DEBUG_MSG("QueryStrategy::()");

  PrintOptions(cout);

  OBPRM_srand(getSeed()); 
  
  Roadmap<CfgType,WeightType>* rdmp = GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap();
  Stat_Class* pStatClass = GetMPProblem()->GetMPRegion(in_RegionID)->GetStatClass();

  vector< ConnectMap<CfgType, WeightType>::NodeConnectionPointer > methods;
    
  if(m_vecStrNodeConnectionLabels.empty()) {
    methods.push_back(ConnectMap<CfgType, WeightType>::NodeConnectionPointer(new ConnectFirst<CfgType, WeightType>()));
  }
  else
    for(vector<string>::iterator I = m_vecStrNodeConnectionLabels.begin(); I != m_vecStrNodeConnectionLabels.end(); ++I)
      methods.push_back(GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*I));

  //perform query
  Clock_Class QueryClock;
  QueryClock.StartClock("Query");
  if(query.PerformQuery(rdmp, *pStatClass, 
                        &m_ConnectMap, 
                        &methods,
                        GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                        GetMPProblem()->GetDistanceMetric()))
  {
    query.WritePath(rdmp);
    cout << endl << "SUCCESSFUL query";
  }
  else
  {
    cout << endl << "UNSUCCESSFUL query";
  }
  QueryClock.StopClock();

#if QUIET
#else
  cout << ":" << QueryClock.GetClock_SEC() << " sec (ie, " << QueryClock.GetClock_USEC() << " usec)";
#endif
  cout << endl;

  LOG_DEBUG_MSG("~QueryStrategy::()");
}

