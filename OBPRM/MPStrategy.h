#ifndef MPStrategy_h
#define MPStrategy_h




#include "SwitchDefines.h"
#include<sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"
#include "Input.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"

#include "GeneratePartitions.h"


//#include "ExplicitInstantiation.h"

/* util.h defines EXIT used in initializing the environment*/
#include "util.h"
#include "MPProblem.h"
#include "MPCharacterizer.h"

#include "MapEvaluator.h"

///Will be used to derive IMP,PRM,RRT,metaplanner, etc.

class MPStrategyMethod : public MPBaseObject 
{
  public:
    MPStrategyMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
      MPBaseObject(in_pNode,in_pProblem) { 
      m_baseSeed=0;
      int seed;
      if(TIXML_SUCCESS  == in_pNode->ToElement()->QueryIntAttribute("seed",&seed)) {
        m_baseSeed = (long) seed;
      } else {
        LOG_DEBUG_MSG("MPStrategyMethod::No Seed Found");
        struct timeval tv;
        gettimeofday(&tv,NULL);
        m_baseSeed = ((unsigned int) tv.tv_usec);
	cout << "RANDOM SEED = " << m_baseSeed << endl;
      }
  
      int iterations;
      if(TIXML_SUCCESS  == in_pNode->ToElement()->QueryIntAttribute("iterations",&iterations)) {
        m_iterations = iterations;
      } else {
        LOG_DEBUG_MSG("MPStrategyMethod::Iterations Found");
      }
      const char* filename;
      filename= in_pNode->ToElement()->Attribute("filename");
      if(filename) {
        m_base_filename = string(filename);
      } else {
        LOG_DEBUG_MSG("MPStrategyMethod::No filename Found");
      }
      LOG_DEBUG_MSG("MPStrategyMethod::Seed is " << m_baseSeed);

      m_reset_stats = false;
      int reset_stats;
      if(TIXML_SUCCESS  == in_pNode->ToElement()->QueryIntAttribute("reset_stats",&reset_stats)) {
	if (reset_stats)
	  m_reset_stats = true;
      }

    };
  virtual void ParseXML(TiXmlNode* in_pNode)=0;
  virtual void operator()()=0;
  virtual void operator()(int in_RegionID)=0;
  virtual void PrintOptions(ostream& out_os)=0;
  long getSeed(){return m_baseSeed;};
  string getBaseFilename(){return m_base_filename;};
  void setSeed(long in_seed){m_baseSeed = in_seed;};
 private:
  
  long m_baseSeed;
  string m_base_filename;

 protected:
  int m_iterations;
  bool m_reset_stats;
};



class MPStrategy : public MPBaseObject
{
public: 
  MPStrategy(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  
  void ParseStrategyMethod(TiXmlNode* in_pNode);
  
  LocalPlanners<CfgType, WeightType>* GetLocalPlanners() {return m_pLocalPlanners;};
  GenerateMapNodes<CfgType>* GetGenerateMapNodes() {return m_pNodeGeneration;};
  ConnectMap<CfgType, WeightType>* GetConnectMap(){return m_pConnection;};
  MPCharacterizer<CfgType, WeightType>* GetCharacterizer(){return m_pCharacterizer;};
  MapEvaluator< CfgType, WeightType > * GetMapEvaluator() { return m_Evaluator;}; 

  bool addPartialEdge, addAllEdges; //move to connect map class
  void PrintOptions(ostream& out_os);
  void Solve(); 
  MPStrategyMethod* GetMPStrategyMethod(string& );////////////////////////
  ///@ToDo Move addPartialEdge, addAllEdges to ConnectMap
 private:
  GenerateMapNodes<CfgType>* m_pNodeGeneration;
  ConnectMap<CfgType, WeightType>* m_pConnection;
  LocalPlanners<CfgType, WeightType>* m_pLocalPlanners;
  //Characterization and Filtering
  MPCharacterizer<CfgType, WeightType>* m_pCharacterizer;
  
  //Map_Evaluation
  MapEvaluator<CfgType, WeightType>* m_Evaluator;

  vector< MPStrategyMethod* > all_MPStrategyMethod;
  string m_strController_MPStrategyMethod;
};


class PRMRoadmap : public MPStrategyMethod {
  public:
    
    
  PRMRoadmap(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("PRMRoadmap::PRMRoadmap()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~PRMRoadmap::PRMRoadmap()");
    };
    
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(TiXmlNode* in_pNode) {
    LOG_DEBUG_MSG("PRMRoadmap::ParseXML()");
    //OBPRM_srand(getSeed()); 
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "node_generation_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          string node_generation_method(in_char);
          m_vecStrNodeGenLabels.push_back(node_generation_method);
        }
      } else if(string(pChild->Value()) == "node_connection_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        LOG_DEBUG_MSG("PRMRoadmap::ParseXML() -- node_connection_method");
        if(in_char) {
          string connect_method(in_char);
          m_vecStrNodeConnectionLabels.push_back(connect_method);
        }
      } else if(string(pChild->Value()) == "component_connection_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        LOG_DEBUG_MSG("PRMRoadmap::ParseXML() -- component_connection_method");
        if(in_char) {
          string connect_method(in_char);
          m_vecStrComponentConnectionLabels.push_back(connect_method);
        }
      } else if(string(pChild->Value()) == "lp_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          m_strLocalPlannerLabel = string(in_char);
        }
      } else if(string(pChild->Value()) == "NodeCharacterizer") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          string node_char(in_char);
          m_vecNodeCharacterizerLabels.push_back(node_char);
        }
      } else {
        LOG_WARNING_MSG("PRMRoadmap::  I don't know: "<< endl << *pChild);
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
    //string base_filename = "itr_test_";
    


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
      vector< CfgType > vectorCfgs;
      NodeGenerationMethod<CfgType> * pNodeGen;
      pNodeGen = GetMPProblem()->GetMPStrategy()->
          GetGenerateMapNodes()->GetMethod(*itr);
      pNodeGen->GenerateNodes(region, vectorCfgs); ///\todo this needs fixing bad.
      cout << "Finished ... I did this many : " << vectorCfgs.size() << endl;
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
      /*
    //Remove nodes that we don't want;
    RoadmapGraph<CfgType,WeightType>* pMap = region->GetRoadmap()->m_pRoadmap;
    vector<VID> map_vids;
    pMap->GetVerticesVID(map_vids);
    typedef vector<VID>::iterator vecVID;
    for(vecVID itr = map_vids.begin(); itr!= map_vids.end(); ++itr)
    {
      if(!(pMap->GetData(*itr).IsLabel("BridgeLike")))
        pMap->DeleteVertex(*itr);
    }
   */
    
  //---------------------------
  // Connect roadmap nodes
  //---------------------------
    ConnectionClock.StartClock("Node Connection");
    
    // get VID's from nodes in the roadmap  
    vector<VID> verticesVID;
    region->GetRoadmap()->m_pRoadmap->GetVerticesVID(verticesVID);
    LOG_DEBUG_MSG("PRMRoadmap:: all nodes: " << verticesVID.size() << "; new nodes: " << new_free_vids.size());
    ConnectMap<CfgType, WeightType>* connectmap = GetMPProblem()->GetMPStrategy()->GetConnectMap();
    typedef vector<string>::iterator J;
    for(J itr = m_vecStrNodeConnectionLabels.begin(); 
        itr != m_vecStrNodeConnectionLabels.end(); ++itr)
    {
      LOG_DEBUG_MSG("PRMRoadmap:: " << *itr);
      NodeConnectionMethod<CfgType,WeightType>* pConnection;
      pConnection = connectmap->GetNodeMethod(*itr);
      //connect new free vids to nodes that were already in the roadmap at itr-1
      pConnection->Connect(region->GetRoadmap(), *pStatClass, 
                           GetMPProblem()->GetCollisionDetection(),
                           GetMPProblem()->GetDistanceMetric(), 
                           GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                           GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                           GetMPProblem()->GetMPStrategy()->addAllEdges,
			   new_free_vids, verticesVID);
      // @todo need to use ifSameCC
      // @todo improve performance: copy new nodes to a temporary roadmap where connection is done.
      // @todo Then,  copy nodes and edges back from the temporary roadmap into the "permanent" roadmap
      // @todo connections in temporary roadmap can be done with IfSameCC option enabled 
      // @todo ISameCC is as efficient as it could be because the temporary roadmap only contains a fraction
      // @todo of the nodes of the "permanent" roadmap
/*        pConnection->Connect(region->GetRoadmap(), *pStatClass,  */
/*  			   GetMPProblem()->GetCollisionDetection(), */
/*  			   GetMPProblem()->GetDistanceMetric(),  */
/*  			   GetMPProblem()->GetMPStrategy()->GetLocalPlanners(), */
/*                             GetMPProblem()->GetMPStrategy()->addPartialEdge,  */
/*  			   GetMPProblem()->GetMPStrategy()->addAllEdges, */
/*  			   new_free_vids, new_free_vids); */
    }
      
    typedef vector<string>::iterator K;
    for(K itr = m_vecStrComponentConnectionLabels.begin(); 
        itr != m_vecStrComponentConnectionLabels.end(); ++itr)
    {
      LOG_DEBUG_MSG("PRMRoadmap:: " << *itr);
      ComponentConnectionMethod<CfgType,WeightType>* pConnection;
      pConnection = connectmap->GetComponentMethod(*itr);
      
      pConnection->Connect(region->GetRoadmap(), *pStatClass, 
                           GetMPProblem()->GetCollisionDetection(),
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
 
      string output_base_filename =  getBaseFilename() +"." + str_RandomSeed +"." 
                                              + str_index + "of" + str_Iterations;
      string outputFilename = output_base_filename+ ".map";
      string outStatname = output_base_filename+ ".stat";
      ofstream  myofstream(outputFilename.c_str());
      std::ofstream  stat_ofstream(outStatname.c_str());
      //std::filebuf stat_ofstream;       // no constructor takes a file name
      //stat_ofstream.open(outStatname.c_str());   // ... it has to be opened manually 
  
      if (!myofstream) {
        LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
        exit(-1);
      }
      region->WriteRoadmapForVizmo(myofstream);
      myofstream.close();
    

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
    // system call to gzip
      string system_out_call(string("gzip -9 ") + outputFilename);
      
      system(system_out_call.c_str());
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
    
    
  PRMOriginalRoadmap(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("PRMOriginalRoadmap::PRMOriginalRoadmap()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~PRMOriginalRoadmap::PRMOriginalRoadmap()");
    };
    
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(TiXmlNode* in_pNode) {
    LOG_DEBUG_MSG("PRMOriginalRoadmap::ParseXML()");
    //OBPRM_srand(getSeed()); 
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "node_generation_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          string node_generation_method(in_char);
          m_vecStrNodeGenLabels.push_back(node_generation_method);
        }
      } else if(string(pChild->Value()) == "node_connection_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        LOG_DEBUG_MSG("PRMOriginalRoadmap::ParseXML() -- node_connection_method");
        if(in_char) {
          string connect_method(in_char);
          m_vecStrNodeConnectionLabels.push_back(connect_method);
        }
      } else if(string(pChild->Value()) == "component_connection_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        LOG_DEBUG_MSG("PRMOriginalRoadmap::ParseXML() -- component_connection_method");
        if(in_char) {
          string connect_method(in_char);
          m_vecStrComponentConnectionLabels.push_back(connect_method);
        }
      } else if(string(pChild->Value()) == "lp_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          m_strLocalPlannerLabel = string(in_char);
        }
      } else {
        LOG_WARNING_MSG("PRMOriginalRoadmap::  I don't know: "<< endl << *pChild);
      }
    }
      
   
    
    LOG_DEBUG_MSG("~PRMOriginalRoadmap::ParseXML()");
  };
   
  virtual void operator()(int in_RegionID) {
    LOG_DEBUG_MSG("PRMOriginalRoadmap::()");

    OBPRM_srand(getSeed()); 
    MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
    Stat_Class * pStatClass = region->GetStatClass();
    
    if (m_reset_stats)
      pStatClass->ClearStats();
  
    Clock_Class Allstuff;
    //string base_filename = "itr_test_";
    


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
      vector< CfgType > vectorCfgs;
      NodeGenerationMethod<CfgType> * pNodeGen;
      pNodeGen = GetMPProblem()->GetMPStrategy()->
          GetGenerateMapNodes()->GetMethod(*itr);
      pNodeGen->GenerateNodes(region, vectorCfgs); ///\todo this needs fixing bad.
      cout << "Finished ... I did this many : " << vectorCfgs.size() << endl;
      region->AddToRoadmap(vectorCfgs);
    }
      
    NodeGenClock.StopClock();
    
  //---------------------------
  // Connect roadmap nodes
  //---------------------------
    ConnectionClock.StartClock("Node Connection");
      
    ConnectMap<CfgType, WeightType>* connectmap = GetMPProblem()->GetMPStrategy()->GetConnectMap();
    typedef vector<string>::iterator J;
    for(J itr = m_vecStrNodeConnectionLabels.begin(); 
        itr != m_vecStrNodeConnectionLabels.end(); ++itr)
    {
      LOG_DEBUG_MSG("PRMOriginalRoadmap:: " << *itr);
      NodeConnectionMethod<CfgType,WeightType>* pConnection;
      pConnection = connectmap->GetNodeMethod(*itr);
      pConnection->Connect(region->GetRoadmap(), *pStatClass, 
                           GetMPProblem()->GetCollisionDetection(),
                           GetMPProblem()->GetDistanceMetric(), 
                           GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                           GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                           GetMPProblem()->GetMPStrategy()->addAllEdges);
    }
      
    typedef vector<string>::iterator K;
    for(K itr = m_vecStrComponentConnectionLabels.begin(); 
        itr != m_vecStrComponentConnectionLabels.end(); ++itr)
    {
      LOG_DEBUG_MSG("PRMOriginalRoadmap:: " << *itr);
      ComponentConnectionMethod<CfgType,WeightType>* pConnection;
      pConnection = connectmap->GetComponentMethod(*itr);
      
      pConnection->Connect(region->GetRoadmap(), *pStatClass, 
                           GetMPProblem()->GetCollisionDetection(),
                           GetMPProblem()->GetDistanceMetric(), 
                           GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                           GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                           GetMPProblem()->GetMPStrategy()->addAllEdges);
      
    }
      
      
      
    ConnectionClock.StopClock();
  
    string outputFilename = getBaseFilename() + "original.map";
    ofstream  myofstream(outputFilename.c_str());
    
    if (!myofstream) {
      LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
      exit(-1);
    }
    region->WriteRoadmapForVizmo(myofstream);
    myofstream.close();
    
      
    pStatClass->PrintAllStats(region->GetRoadmap());
    


    cout << "I took this long" << endl;

    Allstuff.StopPrintClock();
  
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

class RoadmapInput : public MPStrategyMethod {
  public:
    
    
  RoadmapInput(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("RoadmapInput::RoadmapInput()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~RoadmapInput::RoadmapInput()");
    };
    
    virtual void PrintOptions(ostream& out_os) { };
  
    virtual void ParseXML(TiXmlNode* in_pNode) {
      LOG_DEBUG_MSG("RoadmapInput::ParseXML()");
      
          const char* in_char = in_pNode->ToElement()->Attribute("input_map");
          if(in_char) {
            m_strInputFileName = string(in_char);
          }
         else {
           LOG_ERROR_MSG("RoadmapInput::  I don't know: "<< endl << *in_pNode);exit(-1);
        }
    
        LOG_DEBUG_MSG("~RoadmapInput::ParseXML()");
    };
   
    virtual void operator()(int in_RegionID) {
      LOG_DEBUG_MSG("PRMInput::() -- Reading in file: " << m_strInputFileName);
      OBPRM_srand(getSeed()); 
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      
      region->GetRoadmap()->ReadRoadmapGRAPHONLY(m_strInputFileName.c_str());
      
      LOG_DEBUG_MSG("~PRMInput::()");
    }
  
    virtual void operator()() {
      int newRegionId = GetMPProblem()->CreateMPRegion();
      (*this)(newRegionId);      
    };

  private:
    string m_strInputFileName;
   
};

class RoadmapClear : public MPStrategyMethod {
 public:
    
  RoadmapClear(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("RoadmapClear::RoadmapClear()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~RoadmapClear::RoadmapClear()");
    };
    
    virtual void PrintOptions(ostream& out_os) { };
  
    virtual void ParseXML(TiXmlNode* in_pNode) {
      LOG_DEBUG_MSG("RoadmapClear::ParseXML()");
    
      LOG_DEBUG_MSG("~RoadmapClear::ParseXML()");
    };
   
    virtual void operator()(int in_RegionID) {
      LOG_DEBUG_MSG("RoadmapClear::() ");
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      OBPRM_srand(getSeed()); 
      

      region->GetRoadmap()->m_pRoadmap->EraseGraph();
      region->GetStatClass()->ClearStats();
      
      LOG_DEBUG_MSG("~RoadmapClear::()");
    }
  
    virtual void operator()() {
      int newRegionId = GetMPProblem()->CreateMPRegion();
      (*this)(newRegionId);      
    };

  private:
   
};


class MPComparer : public MPStrategyMethod {
  
public: 
  MPComparer(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("MPComparer::MPComparer()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~MPComparer::MPComparer()");
  };
  
  
  
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(TiXmlNode* in_pNode) {
    LOG_DEBUG_MSG("MPComparer::ParseXML()");
    
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "input") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          string strategy(in_char);
          m_input_methods.push_back(strategy);
        }
      } else if (string(pChild->Value()) == "comparer_method") {
	const char* in_char = pChild->ToElement()->Attribute("Method");
	LOG_DEBUG_MSG("MPComparer::ParseXML() -- comparer_method");
	if (in_char) {
	  string evaluator_method(in_char);
	  m_comparer_methods.push_back(evaluator_method);
	}
      } else {
        LOG_WARNING_MSG("MPComparer::  I don't know: "<< endl << *pChild);
      }
    }
      
    LOG_DEBUG_MSG("~MPComparer::ParseXML()");
  }

  virtual void operator()(int in_RegionID) { 

  }
   
  // @todo make the parameter be a vector<int> in_region_ids and loop through it to map regions
  virtual void operator()(int in_RegionID_1, int in_RegionID_2) { 
    OBPRM_srand(getSeed()); 
    // mapping region 1
    LOG_DEBUG_MSG("MPComparer::() -- executing "<< m_input_methods[0]);
    MPStrategyMethod* input1 = GetMPProblem()->GetMPStrategy()->
        GetMPStrategyMethod(m_input_methods[0]);
    (*input1)(in_RegionID_1);

    // mapping region 2
    LOG_DEBUG_MSG("MPComparer::() -- executing "<< m_input_methods[1]);
    MPStrategyMethod* input2 = GetMPProblem()->GetMPStrategy()->
        GetMPStrategyMethod(m_input_methods[1]);
    (*input2)(in_RegionID_2); 

    // comparing region 1 to region 2 with each comparer
    typedef vector<string>::iterator Itrtr;
    for (Itrtr itrtr = m_comparer_methods.begin(); itrtr < m_comparer_methods.end(); itrtr++) {
      MPRegionComparerMethod< CfgType, WeightType > * region_comparer;
      region_comparer = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetComparerMethod(*itrtr);
      region_comparer->Compare(in_RegionID_1, in_RegionID_2);
    }

  }

  virtual void operator()() {
    LOG_DEBUG_MSG("MPComparer::()");
    
    int Input1RegionId = GetMPProblem()->CreateMPRegion();
    int Input2RegionId = GetMPProblem()->CreateMPRegion();
    
    (*this)(Input1RegionId, Input2RegionId);

    LOG_DEBUG_MSG("MPComparer::()");

  }
  
  private:
  vector<string> m_input_methods;
  vector<string> m_comparer_methods;
};



class MPMultiStrategy : public MPStrategyMethod {
  
public: 
  MPMultiStrategy(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("MPMultiStrategy::MPMultiStrategy()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~MPMultiStrategy::MPMultiStrategy()");
  };
  
  
  
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(TiXmlNode* in_pNode) {
    LOG_DEBUG_MSG("MPMultiStrategy::ParseXML()");
    
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if (string(pChild->Value()) == "strategy") {
	const char* in_char = pChild->ToElement()->Attribute("Method");
	LOG_DEBUG_MSG("MPMultiStrategy::ParseXML() -- strategy_method");
	if (in_char) {
	  string strategy(in_char);
	  m_strategy_methods.push_back(strategy);
	}
      } else {
        LOG_WARNING_MSG("MPMultiStrategy::  I don't know: "<< endl << *pChild);
      }
    }
      
    LOG_DEBUG_MSG("~MPMultiStrategy::ParseXML()");
  }

  virtual void operator()(int in_RegionID) { 

    // initializing region from input
    typedef vector< string >::iterator VITRTR;
    for (VITRTR s_itrtr = m_strategy_methods.begin(); s_itrtr < m_strategy_methods.end(); s_itrtr++) { 
      LOG_DEBUG_MSG("MPMultiStrategy::() -- executing "<< (*s_itrtr));
      MPStrategyMethod* strategy = GetMPProblem()->GetMPStrategy()->
        GetMPStrategyMethod(*s_itrtr);
      (*strategy)(in_RegionID);
    }
  }
   

  virtual void operator()() {
    LOG_DEBUG_MSG("MPMultiStrategy::()");
    
    int RegionId = GetMPProblem()->CreateMPRegion();    
    (*this)(RegionId);

    LOG_DEBUG_MSG("MPMultiStrategy::()");

  }
  
  private:
  vector< string > m_strategy_methods;
};





#endif
