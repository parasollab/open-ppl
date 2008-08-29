#ifndef QueryStrategy_h
#define QueryStrategy_h

#include "MPStrategy/MPStrategyMethod.h"
#include "MPStrategy/Query.h"
//#include "LocalPlanners.h"

class QueryStrategy : public MPStrategyMethod 
{
 public:   
  QueryStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) : MPStrategyMethod(in_Node,in_pProblem) 
  {
    LOG_DEBUG_MSG("QueryStrategy::QueryStrategy()");
    ParseXML(in_Node); 
    query.ReadQuery(m_strQueryFileLabel.c_str());
    query.outputPathFile = new char[strlen(m_strPathFileLabel.c_str())+1];
    strcpy(query.outputPathFile, m_strPathFileLabel.c_str());
    LOG_DEBUG_MSG("~QueryStrategy::QueryStrategy()");
  };
  virtual ~QueryStrategy() {}

  virtual void PrintOptions(ostream& out_os) {}
  
  virtual void ParseXML(XMLNodeReader& in_Node) 
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
      else if(citr->getName() == "lp_method") 
      {
        m_strLocalPlannerLabel = citr->stringXMLParameter(string("Method"), true, string(""), string("Local Planner Method"));
        citr->warnUnrequestedAttributes();
      }
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
        citr->warnUnknownNode();
      }
    }
    LOG_DEBUG_MSG("~QueryStrategy::ParseXML()");
  }
   
  virtual void operator()(int in_RegionID) 
  {
    LOG_DEBUG_MSG("QueryStrategy::()");

    OBPRM_srand(getSeed()); 
    Roadmap<CfgType,WeightType>* rdmp = GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap();
    Stat_Class* pStatClass = GetMPProblem()->GetMPRegion(in_RegionID)->GetStatClass();
    CollisionDetection* cd = GetMPProblem()->GetCollisionDetection();
    ConnectMap<CfgType, WeightType>* cm = GetMPProblem()->GetMPStrategy()->GetConnectMap();
    LocalPlanners<CfgType,WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
    DistanceMetric* dm = GetMPProblem()->GetDistanceMetric();

    Clock_Class QueryClock;
    QueryClock.StartClock("Query");
    if(query.PerformQuery(rdmp, *pStatClass, cd, cm, lp, dm))
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
  
  virtual void operator()() 
  {
    int regionID = GetMPProblem()->CreateMPRegion(); 
    GetMPProblem()->GetMPRegion(regionID)->GetRoadmap()->ReadRoadmapGRAPHONLY(m_strMapFileLabel.c_str());
    (*this)(regionID); 
  }

private:
  vector<string> m_vecStrNodeConnectionLabels;
  string m_strLocalPlannerLabel;
  string m_strMapFileLabel;
  string m_strQueryFileLabel;
  string m_strPathFileLabel;
  Query<CfgType, WeightType> query;
};

#endif

