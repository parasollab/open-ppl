#ifndef PRMStrategy_h
#define PRMStrategy_h




#include "SwitchDefines.h"
#include<sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"
#include "RoadmapGraph.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ValidityChecker.hpp"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
//#include "GenerateMapNodes.h"
#include "Sampler.h"

//#include "GeneratePartitions.h"

//#include "ExplicitInstantiation.h"

/* util.h defines PMPL_EXIT used in initializing the environment*/
#include "util.h"
#include "MPProblem.h"
#include "MPCharacterizer.h"

#include "MapEvaluator.h"

#include "MPStrategy/MPStrategyMethod.h"

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType, WeightType> GRAPH;
typedef GRAPH::vertex_descriptor VID; 


class PRMRoadmap : public MPStrategyMethod {
 public:
  
  
  PRMRoadmap(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPStrategyMethod(in_Node,in_pProblem) {
    LOG_DEBUG_MSG("PRMRoadmap::PRMRoadmap()");
    ParseXML(in_Node);    
    LOG_DEBUG_MSG("~PRMRoadmap::PRMRoadmap()");
  };
    virtual ~PRMRoadmap() {}
    
    virtual void PrintOptions(ostream& out_os) { };
    
    virtual void ParseXML(XMLNodeReader& in_Node) {
      LOG_DEBUG_MSG("PRMRoadmap::ParseXML()");
      XMLNodeReader::childiterator citr;
      for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
	if(citr->getName() == "node_generation_method") {
	  string node_gen_method = citr->stringXMLParameter(string("Method"), true,
							    string(""), string("Node Generation Method"));
	  m_vecStrNodeGenLabels.push_back(node_gen_method);
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName() == "node_connection_method") {
	  string connect_method = citr->stringXMLParameter(string("Method"), true,
							   string(""), string("Node Connection Method"));
	  m_vecStrNodeConnectionLabels.push_back(connect_method);
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName() == "component_connection_method") {
	  string connect_method = citr->stringXMLParameter(string("Method"), true,
							   string(""), string("CC Connection Method"));
	  m_vecStrComponentConnectionLabels.push_back(connect_method);
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName() == "lp_method") {
	  
	  m_strLocalPlannerLabel = citr->stringXMLParameter(string("Method"), true,
							    string(""), string("Local Planner Method"));
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName() == "NodeCharacterizer") {
	  string node_char = citr->stringXMLParameter(string("Method"), true,
						      string(""), string("Node Characterization Method"));
	  m_vecNodeCharacterizerLabels.push_back(node_char);
	  citr->warnUnrequestedAttributes();
	} else {
	  citr->warnUnknownNode();
	}
      }
      
      
      
      LOG_DEBUG_MSG("~PRMRoadmap::ParseXML()");
    };
    
    virtual void operator()(int in_RegionID) {
      LOG_DEBUG_MSG("PRMRoadmap::()");
      OBPRM_srand(getSeed()); 
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      Stat_Class * pStatClass = region->GetStatClass();
      
      
      Clock_Class Allstuff;
      
      
      
      Allstuff.StartClock("Everything");
      Clock_Class        NodeGenClock;
      Clock_Class        ConnectionClock;
      //---------------------------
      // Generate roadmap nodes
      //---------------------------
      for(int it =1; it<= m_iterations; ++it)
	{
	  
	  vector<CfgType> nodes;
	  vector<VID> new_free_vids;
	  
	  new_free_vids.erase(new_free_vids.begin(),new_free_vids.end());
	  nodes.erase(nodes.begin(),nodes.end());
	  NodeGenClock.StartClock("Node Generation");
	  typedef vector<string>::iterator I;
	  for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr)
	    {
	      
	      vector< CfgType > vectorCfgs, in_nodes(1024);
	      
	      vectorCfgs.reserve(num_nodes);
	      Sampler<CfgType>::SamplerPointer  pNodeGen;
	      pNodeGen = GetMPProblem()->GetMPStrategy()->
		GetSampler()->GetSamplingMethod(*itr);
	      pNodeGen->GetSampler()->Sample(pNodeGen,GetMPProblem()->GetEnvironment(),*pStatClass,  num_nodes, 100, back_inserter(vectorCfgs));
	      
	      
	      cout << "\nFinished ... I did this many : " << vectorCfgs.size() << endl;
	      new_free_vids = region->AddToRoadmap(vectorCfgs);
	    }
	  
	  NodeGenClock.StopClock();
	  
	  //Characterize Nodes
	  
	  MPCharacterizer<CfgType, WeightType>* characterize =
            GetMPProblem()->GetMPStrategy()->GetCharacterizer();
	  typedef vector<string>::iterator J;
	  for(J itr = m_vecNodeCharacterizerLabels.begin(); 
	      itr != m_vecNodeCharacterizerLabels.end(); ++itr)
	    {
	      NodeCharacterizerMethod<CfgType,WeightType>* pNodeChar;
	      pNodeChar = characterize->GetNodeCharacterizerMethod(*itr);
	      pNodeChar->Characterize(region);
	    }
	  
	  
	  //---------------------------
	  // Connect roadmap nodes
	  //---------------------------
	  ConnectionClock.StartClock("Node Connection");
	  vector<VID> verticesVID;
	  region->GetRoadmap()->m_pRoadmap->GetVerticesVID(verticesVID);
	  LOG_DEBUG_MSG("PRMRoadmap:: all nodes: " << verticesVID.size() << "; new nodes: " << new_free_vids.size());
	  ConnectMap<CfgType, WeightType>* connectmap = GetMPProblem()->GetMPStrategy()->GetConnectMap();
	  typedef vector<string>::iterator J;
	  for(J itr = m_vecStrNodeConnectionLabels.begin(); 
	      itr != m_vecStrNodeConnectionLabels.end(); ++itr)
	    {
	      LOG_DEBUG_MSG("PRMRoadmap:: " << *itr);
	      ConnectMap<CfgType, WeightType>::NodeConnectionPointer pConnection;
	      pConnection = connectmap->GetNodeMethod(*itr);
	      GetMPProblem()->GetMPStrategy()->GetConnectMap()->ConnectNodes(
									     shared_ptr<NodeConnectionMethod<CfgType,WeightType> > (pConnection),
									     region->GetRoadmap(), *pStatClass, 
									     GetMPProblem()->GetDistanceMetric(),
									     GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
									     GetMPProblem()->GetMPStrategy()->addPartialEdge, 
									     GetMPProblem()->GetMPStrategy()->addAllEdges,
									     verticesVID.begin(), verticesVID.end());
	    }
	  
	  
	  typedef vector<string>::iterator K;
	  for(K itr = m_vecStrComponentConnectionLabels.begin(); 
	      itr != m_vecStrComponentConnectionLabels.end(); ++itr)
	    {
	      LOG_DEBUG_MSG("PRMRoadmap:: " << *itr);
	      ConnectMap<CfgType,WeightType>::ComponentConnectionPointer pConnection;
	      pConnection = connectmap->GetComponentMethod(*itr);
	      connectmap->ConnectComponents(pConnection,region->GetRoadmap(), *pStatClass, 
					    GetMPProblem()->GetDistanceMetric(),
					    GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
					    GetMPProblem()->GetMPStrategy()->addPartialEdge, 
					    GetMPProblem()->GetMPStrategy()->addAllEdges);
	      
	    }
	  
	  
	  
	  ConnectionClock.StopClock();
	  std::stringstream ss;
	  ss << it;
	  std::string str_index;
	  ss >> str_index;
	  
	  
	  std::stringstream ssIterations;
	  ssIterations << m_iterations;
	  std::string str_Iterations;
	  ssIterations >> str_Iterations;
	  
	  std::stringstream ssRandomSeed;
	  ssRandomSeed << getSeed();
	  std::string str_RandomSeed;
	  ssRandomSeed >> str_RandomSeed; 
	  
	  string output_base_filename = getBaseFilename() +"." + str_RandomSeed +"." 
	    + str_index + "of" + str_Iterations;
	  string outputFilename = output_base_filename+ ".map";
	  string outStatname = output_base_filename+ ".stat";
	  ofstream  myofstream(outputFilename.c_str());
	  std::ofstream  stat_ofstream(outStatname.c_str());
	  
	  if (!myofstream) {
	    LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
	    exit(-1);
	  }
	  region->WriteRoadmapForVizmo(myofstream);
	  myofstream.close();
	  
	  NodeGenClock.PrintClock();
	  ConnectionClock.PrintClock();
	  Allstuff.StopPrintClock();
	  
	  std::streambuf* sbuf = std::cout.rdbuf(); // to be restored later
	  std::cout.rdbuf(stat_ofstream.rdbuf());   // redirect destination of std::cout
	  
	  pStatClass->PrintAllStats(region->GetRoadmap());
	  NodeGenClock.PrintClock();
	  ConnectionClock.PrintClock();
	  Allstuff.StopPrintClock();
	  
	  
	  pStatClass->ComputeIntraCCFeatures(region->GetRoadmap(),
					     GetMPProblem()->GetDistanceMetric());
	  
	  pStatClass->ComputeInterCCFeatures(region->GetRoadmap(),
					     GetMPProblem()->GetDistanceMetric());
	  pStatClass->PrintFeatures();
	  
	  std::cout.rdbuf(sbuf);  // restore original stream buffer 
	  stat_ofstream.close();
	  
	  cout << "Finished iteration " << str_index << " of " << str_Iterations << endl;
	}
      cout << "!!ALL FINISHED!!"<< endl;
      LOG_DEBUG_MSG("~PRMRoadmap::()");
    }
    
    virtual void operator()() {
      int newRegionId = GetMPProblem()->CreateMPRegion();
      (*this)(newRegionId);      
    };
    
 private:
    vector<string> m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    vector<string> m_vecStrComponentConnectionLabels;
    vector<string> m_vecNodeCharacterizerLabels;
    string m_strLocalPlannerLabel;
    
};


class PRMOriginalRoadmap : public MPStrategyMethod {
 public:
  
  
  PRMOriginalRoadmap(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPStrategyMethod(in_Node,in_pProblem) {
    LOG_DEBUG_MSG("PRMOriginalRoadmap::PRMOriginalRoadmap()");
    ParseXML(in_Node);    
    LOG_DEBUG_MSG("~PRMOriginalRoadmap::PRMOriginalRoadmap()");
  };
    virtual ~PRMOriginalRoadmap() {}
    
    virtual void PrintOptions(ostream& out_os) { };
    
    virtual void ParseXML(XMLNodeReader& in_Node) {
      LOG_DEBUG_MSG("PRMOriginalRoadmap::ParseXML()");
      XMLNodeReader::childiterator citr;
      for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
	if(citr->getName() == "node_generation_method") {
	  string node_gen_method = citr->stringXMLParameter(string("Method"),true,
							    string(""),string("Node Generation Method"));
	  m_vecStrNodeGenLabels.push_back(node_gen_method);
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName() == "node_connection_method") {
	  string connect_method = citr->stringXMLParameter(string("Method"),true,
							   string(""),string("Node Connection Method"));
	  m_vecStrNodeConnectionLabels.push_back(connect_method);
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName() == "component_connection_method") {
	  string connect_method = citr->stringXMLParameter(string("Method"),true,
							   string(""),string("CC Connection Method"));
	  m_vecStrComponentConnectionLabels.push_back(connect_method);
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName() == "lp_method") {
	  m_strLocalPlannerLabel = citr->stringXMLParameter(string("Method"),true,
							    string(""),string("Local Planner Method"));
	  citr->warnUnrequestedAttributes();
	} else {
	  citr->warnUnknownNode();
	}
      }
      
      LOG_DEBUG_MSG("~PRMOriginalRoadmap::ParseXML()");
    };
    
    virtual void operator()(int in_RegionID) {
      LOG_DEBUG_MSG("PRMOriginalRoadmap::()");
      
      OBPRM_srand(getSeed()); 
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      Stat_Class * pStatClass = region->GetStatClass();
      
      pStatClass->ClearStats();
      
      Clock_Class Allstuff;
      
      
      
      Allstuff.StartClock("Everything");
      Clock_Class        NodeGenClock;
      Clock_Class        ConnectionClock;
      
      //---------------------------
      // Generate roadmap nodes
      //---------------------------
      NodeGenClock.StartClock("Node Generation");
      typedef vector<string>::iterator I;
      for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr)
	{
	  vector< CfgType > vectorCfgs, in_nodes;
	  Sampler<CfgType>::SamplerPointer  pNodeGen;
	  pNodeGen = GetMPProblem()->GetMPStrategy()->
	    GetSampler()->GetSamplingMethod(*itr);
	  pNodeGen->GetSampler()->Sample(pNodeGen,GetMPProblem()->GetEnvironment(),*pStatClass,in_nodes.begin(),in_nodes.end(),1,back_inserter(vectorCfgs));
	  cout << "Finished ... I did this many : " << vectorCfgs.size() << endl;
	  region->AddToRoadmap(vectorCfgs);
	}
      
      NodeGenClock.StopClock();
      
      //---------------------------
      // Connect roadmap nodes
      //---------------------------
      ConnectionClock.StartClock("Node Connection");
      vector<VID> allVIDS;
      region->GetRoadmap()->m_pRoadmap->GetVerticesVID(allVIDS);
      ConnectMap<CfgType, WeightType>* connectmap = GetMPProblem()->GetMPStrategy()->GetConnectMap();
      typedef vector<string>::iterator J;
      for(J itr = m_vecStrNodeConnectionLabels.begin(); 
	  itr != m_vecStrNodeConnectionLabels.end(); ++itr)
	{
	  LOG_DEBUG_MSG("PRMOriginalRoadmap:: " << *itr);
	  
	  ConnectMap<CfgType,WeightType>::NodeConnectionPointer pConnection;
	  pConnection = connectmap->GetNodeMethod(*itr);
	  cout << "Calling connection method:: " << pConnection->GetLabel() << endl;
	  connectmap->ConnectNodes(pConnection, region->GetRoadmap(), *pStatClass, 
				   GetMPProblem()->GetDistanceMetric(), 
				   GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
				   GetMPProblem()->GetMPStrategy()->addPartialEdge, 
				   GetMPProblem()->GetMPStrategy()->addAllEdges,
				   allVIDS.begin(), allVIDS.end(), allVIDS.begin(), allVIDS.end());
	}
      
      typedef vector<string>::iterator K;
      for(K itr = m_vecStrComponentConnectionLabels.begin(); 
	  itr != m_vecStrComponentConnectionLabels.end(); ++itr)
	{
	  LOG_DEBUG_MSG("PRMOriginalRoadmap:: " << *itr);
	  ConnectMap<CfgType,WeightType>::ComponentConnectionPointer pConnection;
	  pConnection = connectmap->GetComponentMethod(*itr);
	  
	  connectmap->ConnectComponents(pConnection, region->GetRoadmap(), *pStatClass, 
					GetMPProblem()->GetDistanceMetric(), 
					GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
					GetMPProblem()->GetMPStrategy()->addPartialEdge, 
					GetMPProblem()->GetMPStrategy()->addAllEdges);
	  
	}
      
      
      
      ConnectionClock.StopClock();
      
      
      string outputFilename = getBaseFilename() + ".original.map";  
      ofstream  myofstream(outputFilename.c_str());
      
      if (!myofstream) {
        LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
        exit(-1);
      }
      region->WriteRoadmapForVizmo(myofstream);
      myofstream.close();
      
      
      pStatClass->PrintAllStats(region->GetRoadmap());
      NodeGenClock.PrintClock();
      ConnectionClock.PrintClock();
      Allstuff.StopPrintClock();
      
      cout << "Finished map " << endl;
      LOG_DEBUG_MSG("~PRMOriginalRoadmap::()");
    }
    
    virtual void operator()() {
      int newRegionId = GetMPProblem()->CreateMPRegion();
      (*this)(newRegionId);      
    };
    
 private:
    vector<string> m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    vector<string> m_vecStrComponentConnectionLabels;
    vector<string> m_vecNodeCharacterizerLabels;
    string m_strLocalPlannerLabel;
    
};

#endif

