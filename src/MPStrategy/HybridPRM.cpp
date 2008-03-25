#include "SwitchDefines.h"
#include <sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"

#include "GeneratePartitions.h"

/* util.h defines EXIT used in initializing the environment*/
#include "util.h"
#include "MPProblem.h"
#include "MPCharacterizer.h"

#include "MapEvaluator.h"
#include "MPStrategy/MPStrategy.h"

#include "MPStrategy/MPStrategyMethod.h"

#include "MPStrategy/HybridPRM.h"




void 
HybridPRM::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("HybridPRM::ParseXML()");
  //OBPRM_srand(getSeed());
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "node_generation_method") {
      string node_gen_method = citr->stringXMLParameter("Method",true,"","Method");
      m_vecStrNodeGenLabels.push_back(node_gen_method);
      
      int initial_cost = citr->numberXMLParameter("initial_cost",true,
                                  0,0,MAX_INT,"initial_cost");
      if (initial_cost > 0)
        m_mapStrNodeGenCost[node_gen_method] = initial_cost;
      else
        m_mapStrNodeGenCost[node_gen_method] = 1;
    } else if(citr->getName() == "node_connection_method") {
      string connect_method = citr->stringXMLParameter("Method",true,"","Method");
      m_vecStrNodeConnectionLabels.push_back(connect_method);
    } else if(citr->getName() == "NodeCharacterizer") {
      string node_char = citr->stringXMLParameter("Method",true,"","Method");
      m_vecNodeCharacterizerLabels.push_back(node_char);
    } else if(citr->getName() == "component_connection_method") {
      string connect_method = citr->stringXMLParameter("Method",true,"","Method");
      m_vecStrComponentConnectionLabels.push_back(connect_method);
    } else if(citr->getName() == "WitnessQuery") {
      m_strWitnessFilename = citr->stringXMLParameter("Filename",true,"","Filename");
    } else {
      citr->warnUnknownNode();
    }
  }

  
  m_percentage_random = in_Node.numberXMLParameter("percent_random",true,
                                double(0.5),double(0),double(1),"percent_random");
  
  m_bin_size = in_Node.numberXMLParameter("bin_size",true,
                                5,1,MAX_INT,"bin_size");

  m_window_percent = in_Node.numberXMLParameter("window_percent",true,
                                double(0.5),double(0),double(1),"window_percent");

  m_count_cost = in_Node.numberXMLParameter("Count_Cost",true,
                                5,0,MAX_INT,"Count_Cost");

  int fixed_cost = in_Node.numberXMLParameter("fixed_cost",true,
                                0,0,1,"fixed_cost");
  
  if (fixed_cost == 1);
  m_fixed_cost = 1;

  int resetting_learning;
  m_resetting_learning = in_Node.numberXMLParameter("resetting_learning",true,
                                0,0,1,"resetting_learning");


  m_sampler_selection_distribution = in_Node.stringXMLParameter("sampler_selection_distribution",
                                                  false,"","sampler_selection_distribution");

  if (m_sampler_selection_distribution == "nowindow_adaptive")
    m_window_percent = 1.0; // 100% of the time learning

  //--------------------------
  //SET UP BASEFILENAME OUTPUT
  //--------------------------
  std::stringstream ssRandomSeed;
  ssRandomSeed << getSeed();
  std::string str_RandomSeed;
  ssRandomSeed >> str_RandomSeed; 
  m_strBaseFilename =  getBaseFilename() +"." + str_RandomSeed;

  //--------------------------
  //Reading in witness queries
  //--------------------------
  CfgType tempCfg;
  ifstream  myifstream(m_strWitnessFilename.c_str());
  if (!myifstream) {
    cout << endl << "In PRMIncrementalStrategy: can't open witness file: " << m_strWitnessFilename;
    exit(-1);
  }
  while (1) {
    tempCfg.Read(myifstream);
    if(!myifstream) break;
    m_vecWitnessNodes.push_back(tempCfg);
  }
  myifstream.close();

  LOG_DEBUG_MSG("~HybridPRM::ParseXML()");
}


//For i =  0 to num nodes
//  Call Function to get next NodeGenMethod
//  Connect and Classify Node
//  Update weights based on node


void 
HybridPRM::
operator()(int in_RegionID) {
  LOG_DEBUG_MSG("HybridPRM::()");
  OBPRM_srand(getSeed()); 
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
  Clock_Class Allstuff, ConnectionClock;
  Stat_Class * pStatClass = region->GetStatClass();
  m_totalSamples = 0;
  initializeWeightProb();
  //------------------------
  //SET UP FILENAME OUTPUT
  //----------------------- 
  string outputFilename = m_strBaseFilename+ ".map";
  string outStatname = m_strBaseFilename+ ".stat";
  string outCharname = m_strBaseFilename+ ".char";
  string outTotal = m_strBaseFilename+ ".total";
  ofstream  myofstream(outputFilename.c_str());
  std::ofstream  stat_ofstream(outStatname.c_str());
  std::ofstream  char_ofstream(outCharname.c_str());
  std::ofstream  total_ofstream(outTotal.c_str());
  double NodeGenTotalTime(0);
  //-----------------------
  //Set up witness nodes
  //-----------------------
  string envFileName = GetMPProblem()->GetEnvFileName();
  string firstNodeGen = *m_vecStrNodeGenLabels.begin();
  string firstConnection = *m_vecStrNodeConnectionLabels.begin();
  char_ofstream << "#env_file_name:seed:num_node_gen:node_gen_methods" << endl; // this could be better
  char_ofstream << envFileName << ":" << getSeed() << ":" << "HybridPRM" << ":" << firstConnection << ":" <<  endl;
  char_ofstream << "#numnodes";
  for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
    char_ofstream << ":" << m_vecStrNodeGenLabels[i];
  }
  char_ofstream << ":ThisNodeType:NodeGenTime:witnessCoverage:witneessQuery:largest_cc_dia:sum_cc_dia:dia_time:distance_time:CD-Calls";// << endl;
   for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
    char_ofstream << ":" << m_vecStrNodeGenLabels[i] << "-numNodes";
  }
   for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
    char_ofstream << ":" << m_vecStrNodeGenLabels[i] << "-numOversamples";
  }
  char_ofstream << endl;

  double total_dia_time(0),largest_cc_dia(0),sum_cc_dia(0);

  pair < unsigned int, unsigned int > witness_qry;
  double out_qry, witness_coverage;
  unsigned int witness_queries = m_vecWitnessNodes.size()*(m_vecWitnessNodes.size()-1)/2;
  Clock_Class query_time;

  //---------------------------
  // Generate roadmap nodes
  //---------------------------
  Allstuff.StartClock("Everything");
 
  
  cout << "I will do this many samples: " << m_iterations << endl;
  initializeWeightProb();
  CopyPlearnPuse();

  for (int i=0; i<m_iterations && m_totalSamples < m_iterations; ++i) {
    //Call Function to get next NodeGenMethod
    //Connect and Classify Node
    //Update weights based on node
    outputWeightMatrix(cout);
    NodeGenerationMethod<CfgType> * pNodeGen;
    //Generate nodes given 1 node gen method



    do {
      bool learning=false;
      if((m_totalSamples % m_bin_size) < (m_bin_size * m_window_percent)) 
	learning = true;
      string next_node_gen = GetNextNodeGenStr(learning);

      Clock_Class NodeGenClock;
      NodeGenClock.StartClock("Node Generation");
      vector< CfgType > vectorCfgs;
      cout << "About to get next pointer method = " <<next_node_gen<< endl;
      pNodeGen = GetMPProblem()->GetMPStrategy()->GetGenerateMapNodes()->GetMethod(next_node_gen);
      unsigned long int num_cd_before = pStatClass->GetIsCollTotal();
      pNodeGen->GenerateNodes(region, vectorCfgs);
      

      for(int j=0; j<vectorCfgs.size(); ++j) {   //Loop through all nodes created by node gen
	int lp_attempt_before(0),lp_attempt_after(0),lp_connect_before(0),lp_connect_after(0);
	if(vectorCfgs[j].IsLabel("VALID")) {
	  if(vectorCfgs[j].GetLabel("VALID")) {  //Add to Free roadmap
	    ++m_totalSamples;               //Increment total node counter for stop criteria
	    int nNumPrevCCs = GetCCcount(*(region->roadmap.m_pRoadmap));
	    vector< CfgType > newCfg;
	    newCfg.push_back(vectorCfgs[j]);
	    int newVID = region->roadmap.m_pRoadmap->AddVertex(newCfg);
	    
	    //Connect New node to roadmap.
	    ConnectMap<CfgType, WeightType>* connectmap = GetMPProblem()->GetMPStrategy()->GetConnectMap();
	    typedef vector<string>::iterator J;
	    int lp_attempt_before = pStatClass->LPAttempts[0];
	    int lp_connect_before = pStatClass->LPConnections[0];
	    for(J itr = m_vecStrNodeConnectionLabels.begin(); itr != m_vecStrNodeConnectionLabels.end(); ++itr) {
	      NodeConnectionMethod<CfgType,WeightType>* pConnection;
	      pConnection = connectmap->GetNodeMethod(*itr);
	      //connect new free vid to nodes that were already in the roadmap at itr-1
	      vector<VID> new_free_vid;
	      vector<VID> map_vids;
	      new_free_vid.push_back(newVID);
	      region->roadmap.m_pRoadmap->GetVerticesVID(map_vids);
	      pConnection->Connect(region->GetRoadmap(), *pStatClass, 
				   GetMPProblem()->GetCollisionDetection(),
				   GetMPProblem()->GetDistanceMetric(), 
				   GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
				   GetMPProblem()->GetMPStrategy()->addPartialEdge, 
				   GetMPProblem()->GetMPStrategy()->addAllEdges,
				   new_free_vid, map_vids);
	    }
	    int lp_attempt_after = pStatClass->LPAttempts[0];
	    int lp_connect_after = pStatClass->LPConnections[0];
	    int nNumCurrCCs = GetCCcount(*(region->roadmap.m_pRoadmap));
	    unsigned long int num_cd_after = pStatClass->GetIsCollTotal();
	    unsigned long int cost = num_cd_after - num_cd_before;
	    cout << "Latest Cost = " << cost << endl;
	    cout << "out_qry = " << out_qry << endl;
	    double reward(0.0);
	    if(nNumCurrCCs > nNumPrevCCs) { //CC Create
	      reward = 1.0;
	    } else if(nNumCurrCCs < nNumPrevCCs) { //CC Merge
	      reward = 1.0;
	    } else { //"Other" -- not rewarded
	      //Calculate the reward for an expand node
	      int lp_attempt = lp_attempt_after - lp_attempt_before;
	      int lp_connect = lp_connect_after - lp_connect_before;
	      if(lp_attempt != 0 )
		reward = double( double(1) - double(double(lp_connect) / double(lp_attempt)));
	      else 
		reward = double(0.0);
	    }
	    if (reward < 0.7) // considered oversample
	      m_mapStrNodeGenNumOversamples[next_node_gen]++;

	    if((m_totalSamples % m_bin_size) < (m_bin_size * m_window_percent)) { //if learning
	      if (reward > 0.0 && reward < 1.0)
		cout << endl << endl << "@#$@#$@#$#$#@$ I am rewarding an expand node a reward of:" << reward << endl << endl;
	      RewardAndRecalcWeight(next_node_gen,reward,cost);
	    } else {
	      m_mapStrNodeGenNumSamp[next_node_gen]++; 
	    } // remove this line soon
 //else { initializeWeightProb(); m_mapStrNodeGenNumSamp[next_node_gen]++; } // remove this line soon
	    if (m_totalSamples%m_bin_size == 0) { // at the end of the bin
	      if (m_sampler_selection_distribution == "nowindow_adaptive")
		CopyPlearnPuse();
	      if (m_resetting_learning == 1)
		initializeWeightProb();
	    }

	  }
	}
	
      }

      NodeGenClock.StopClock();
      NodeGenTotalTime += NodeGenClock.GetClock_SEC(); 
      //End Node generation and connection

      //Start method evaluation 

      query_time.StartClock("query_time");
      //if((m_totalSamples % 50 == 0) || (m_totalSamples == 1)) {
      if(out_qry != 100) {
	witness_qry = ConnectionsWitnessToRoadmap(m_vecWitnessNodes,&(region->roadmap),m_queryStat);
	witness_coverage = 100*witness_qry.first/m_vecWitnessNodes.size();
	out_qry = 100*witness_qry.second/witness_queries;
      }
      query_time.StopClock();

      Clock_Class dia_time;
      dia_time.StartClock("dia_time");
      if((m_totalSamples % 50 == 0) || (m_totalSamples == 1)) {
	//Calculate CC diameter information.
	largest_cc_dia = sum_cc_dia = double(0.0);
	double _cc_dia = double(0.0);
	vector < pair< int, VID > > cc;
	GetCCStats(*(region->roadmap.m_pRoadmap), cc);
	for(int i=0; i<cc.size(); ++i) {
	  _cc_dia = cc_diamater(region->roadmap.m_pRoadmap, cc[i].second);
	  sum_cc_dia += _cc_dia;
	  if(_cc_dia > largest_cc_dia) {largest_cc_dia = _cc_dia;}
	}
      }
      dia_time.StopClock();
      total_dia_time += dia_time.GetClock_SEC();
      
      


      //Start Printing node stats
      char_ofstream << m_totalSamples;
      for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
	char_ofstream << ":" << m_mapStrNodeGenProbability[m_vecStrNodeGenLabels[i]];
      }
      char_ofstream << ":" << next_node_gen << ":" << NodeGenTotalTime << ":" << witness_coverage << ":" << out_qry;
      char_ofstream << ":" << largest_cc_dia << ":" << sum_cc_dia << ":" << total_dia_time << ":" << GetMPProblem()->GetDistanceMetric()->m_distance_time << ":" << pStatClass->GetIsCollTotal();// << endl;;
      
      for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
	char_ofstream << ":" << m_mapStrNodeGenNumSamp[m_vecStrNodeGenLabels[i]];
      }
      for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
	char_ofstream << ":" << m_mapStrNodeGenNumOversamples[m_vecStrNodeGenLabels[i]];
      }
      
      
      char_ofstream << endl;
  
    } while( (m_totalSamples % m_bin_size) > (m_bin_size * m_window_percent));

  } 

 

  total_ofstream << "env: " << envFileName << endl;
  total_ofstream << "nodegen: " << firstNodeGen << endl;
  total_ofstream << "connection: " << firstConnection << endl;
  total_ofstream << "nodes: " << region->roadmap.m_pRoadmap->GetVertexCount() << endl;
  total_ofstream << "ccs: " << GetCCcount(*(region->roadmap.m_pRoadmap)) << endl;
  total_ofstream << "iscoll: " <<  pStatClass->GetIsCollTotal() << endl;
  total_ofstream << "time: " << NodeGenTotalTime << endl;
  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  ConnectionClock.StopClock();

  if (!myofstream) {
     LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
     exit(-1);
   }
  region->WriteRoadmapForVizmo(myofstream);
  myofstream.close();
  

  std::streambuf* sbuf = std::cout.rdbuf(); // to be restored later
  std::cout.rdbuf(stat_ofstream.rdbuf());   // redirect destination of std::cout
  cout << "NodeGen+Connection Stats" << endl;
  pStatClass->PrintAllStats(region->GetRoadmap());
  //NodeGenClock.PrintClock();
  cout << "Node Gen = " << NodeGenTotalTime << endl;
  ConnectionClock.PrintClock();
  Allstuff.StopPrintClock();
  pStatClass->PrintFeatures();

  cout << "Overhead Stats" << endl;
  m_nodeOverheadStat.PrintAllStats(region->GetRoadmap());
  cout << "Query Stats" << endl;
  m_queryStat.PrintAllStats(region->GetRoadmap());
     
  std::cout.rdbuf(sbuf);  // restore original stream buffer 
  stat_ofstream.close();
  char_ofstream.close();
   // system call to gzip
   //  string system_out_call(string("gzip -9 ") + outputFilename);
     
  // system(system_out_call.c_str());
  cout << "!!ALL FINISHED!!"<< endl;
  LOG_DEBUG_MSG("~HybridPRM::()");
}


bool 
HybridPRM::
CanConnectComponents(vector < CfgType > & cc_a, vector < CfgType > & cc_b, Stat_Class& stat) {
  // variables needed for the local planner call in loop
  LocalPlanners < CfgType, WeightType > * lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  LPOutput< CfgType, WeightType > lp_output; 
  Environment * env = GetMPProblem()->GetEnvironment();
  CollisionDetection * cd = GetMPProblem()->GetCollisionDetection();
  DistanceMetric * dm = GetMPProblem()->GetDistanceMetric();
  double pos_res = GetMPProblem()->GetEnvironment()->GetPositionRes();
  double ori_res = GetMPProblem()->GetEnvironment()->GetOrientationRes();
  //Stat_Class Stats;

  typedef  vector< CfgType >::iterator CFG_ITRTR;
  for(CFG_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) {
    sort(cc_b.begin(), cc_b.end(), CFG_CFG_DIST_COMPARE<CfgType>(*i_cc_a,dm,env));
    for (CFG_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) {
      if (lp->IsConnected(env, stat, cd, dm, (*i_cc_a), (*i_cc_b), 
        &lp_output, pos_res, ori_res, true)) {
          return true; // st  op as soon as one cc in a can connect to a node in b
      }
    }
  }
  return false;
}






double 
HybridPRM::
cc_diamater(RoadmapGraph<CfgType,WeightType>* pGraph, VID _cc) {
 // vector<VID> cc_vids;
 // GetCC ( *pGraph, _cc, cc_vids);
  double return_val = 0.0;
  /*
  for(int i=0; i<cc_vids.size(); ++i) {
  //  RoadmapGraph<CfgType,WeightType> *result; 
  //  result = new RoadmapGraph<CfgType,WeightType>;
    //double length = DijkstraSSSP(*(pGraph),*result,cc_vids[i]);
    double length = DijkstraSSSP(*(pGraph),cc_vids[i]);
    //result->EraseGraph();
    //delete result;
    if(length > return_val) { return_val = length;}
  }
  return return_val;
  */

  VID farVID, tmpVID;
  double length = ComponentDiameter(*(pGraph),_cc, &farVID);
  return_val = ComponentDiameter(*(pGraph),farVID, &tmpVID);
  return return_val;
}


pair < unsigned int, unsigned int >
HybridPRM::
ConnectionsWitnessToRoadmap(vector < CfgType > & witness_cfgs, Roadmap< CfgType, WeightType > *rdmp, Stat_Class& stat) {
  int small_cc_size =0;
  vector < pair< int, VID > > cc;
  GetCCStats(*(rdmp->m_pRoadmap), cc);
  // small_cc_size = int(double(cc[0].first) * double(0.01));
  vector < vector< unsigned int > > connected_to_cc;
  vector < unsigned int > tmp_vector;
  for(unsigned int i=0; i< cc.size(); i++)
    connected_to_cc.push_back(tmp_vector);
  
  unsigned int possible_connections = 0;
  vector< CfgType > witness_test_cfg;
  typedef vector< CfgType >::iterator CFG_ITRTR;
  for (unsigned int i=0; i < witness_cfgs.size(); i++) {
    witness_test_cfg.clear();
    witness_test_cfg.push_back(witness_cfgs[i]);

    typedef vector < pair< int, VID > >::iterator CC_ITRTR;
    unsigned int j=0;
    bool i_witness_can_connect = false;
    for (unsigned int j=0; j < cc.size(); j++) {
      vector < CfgType > cc_cfgs;
      //GetCC(*(rdmp->m_pRoadmap), *(new CfgType(rdmp->m_pRoadmap->GetData(cc[j].second))), cc_cfgs);
      CfgType * tmp_pointer = new CfgType(rdmp->m_pRoadmap->GetData(cc[j].second));
      GetCC(*(rdmp->m_pRoadmap), *(tmp_pointer), cc_cfgs);
      delete tmp_pointer;
      if (cc_cfgs.size() >= small_cc_size) {
        if ( CanConnectComponents(witness_test_cfg, cc_cfgs, stat) ) {
          i_witness_can_connect = true;
          connected_to_cc[j].push_back(i);
        }
      }
    }
    if (i_witness_can_connect)
      possible_connections++; 
  }

  unsigned int possible_queries = 0;
  typedef vector< unsigned int >::iterator INT_ITRTR;
  typedef vector< vector < unsigned int > >::iterator DINT_ITRTR;
  for (DINT_ITRTR i_ccs=connected_to_cc.begin(); i_ccs < connected_to_cc.end(); i_ccs++) {
    bool i_in_j = false;
    for(DINT_ITRTR i_ccs_other= i_ccs+1; i_ccs_other < connected_to_cc.end(); i_ccs_other++) {
      for (INT_ITRTR i_el_i = i_ccs->begin(); i_el_i < i_ccs->end(); i_el_i++) {
        //test whether *i_ccs is in *(i_ccs+1)  
        for (INT_ITRTR i_el_j = (i_ccs_other)->begin(); i_el_j < (i_ccs_other)->end(); i_el_j++) {
          if ( (*i_el_i) == (*i_el_j) ) {
            i_ccs_other->insert(i_ccs_other->end(),i_ccs->begin(),i_ccs->end());
            i_in_j = true;
            break;
          }
        }
        if (i_in_j)
          i_ccs->clear();
        }
      }
    }

  for (DINT_ITRTR i_con=connected_to_cc.begin(); i_con < connected_to_cc.end(); i_con++) {
    // count whether *i_witness_cfg can connect to this cc in rdmp  
    // count the number of queries that could be done through this cc
    sort(i_con->begin(),i_con->end());
    INT_ITRTR newEnd;
    newEnd = unique(i_con->begin(),i_con->end());
    int i_con_size = newEnd - i_con->begin();
    possible_queries += (i_con_size)*(i_con_size-1); //remember to divide by 2 at the end
  }

  return pair < unsigned int, unsigned int>(possible_connections, possible_queries/2);
}

