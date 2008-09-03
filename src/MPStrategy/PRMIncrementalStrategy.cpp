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

#include "MPStrategy/PRMIncrementalStrategy.h"




void 
PRMIncrementalStrategy::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("PRMIncrementalStrategy::ParseXML()");
  //OBPRM_srand(getSeed()); 
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
     if(citr->getName() == "node_generation_method") {
      string node_gen_method = citr->stringXMLParameter("Method",true,"","Method");
      m_vecStrNodeGenLabels.push_back(node_gen_method);
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

  LOG_DEBUG_MSG("~PRMIncrementalStrategy::ParseXML()");
}


void 
PRMIncrementalStrategy::
operator()(int in_RegionID) {
  LOG_DEBUG_MSG("PRMIncrementalStrategy::()");
  OBPRM_srand(getSeed()); 
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
  Clock_Class Allstuff, NodeGenClock, ConnectionClock;
  Stat_Class * pStatClass = region->GetStatClass();
  m_totalSamples = 0;
  m_query_solved = false;
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
  //-----------------------
  //Set up witness nodes
  //-----------------------
  unsigned int witness_queries = m_vecWitnessNodes.size()*(m_vecWitnessNodes.size()-1)/2;
  cout << "witness_queries = " << witness_queries << endl;

  int neighbor_create, neighbor_merge, neighbor_expand, neighbor_over;
  double cc_create_clearance = 0;
  pair < unsigned int, unsigned int > witness_qry;
  double witness_converage;
  double out_qry, new_cc_dia, old_cc_dia, ratio_cc_dia;
  int stat_cccreate, stat_ccmerge, stat_ccexpand, stat_ccoversample;
  int stat_oversample_relaxed = 0;
  double max_cc_dia_found = 0.0;
  stat_cccreate = stat_ccmerge = stat_ccexpand =stat_ccoversample= 0;
  int merged_cc_size;
  int old_biggest_cc_size;
  double ratio_cc_size;
  double largest_cc_dia, sum_cc_dia;
  largest_cc_dia = sum_cc_dia = double(0.0);
  double total_query_time, total_dia_time; total_query_time = total_dia_time = double(0.0);
  double running_time = double(0.0);
  

  string envFileName = GetMPProblem()->GetEnvironment()->GetEnvFileName();
  string firstNodeGen = *m_vecStrNodeGenLabels.begin();
  string firstConnection = *m_vecStrNodeConnectionLabels.begin();
  char_ofstream << "#env_file_name:node_gen:con_method:seed" << endl;
  char_ofstream << envFileName << ":" << firstNodeGen << ":" << firstConnection << ":" << getSeed() << endl;
  char_ofstream << "#numnodes" << ":" << "cc_create" 
                << ":" << "cc_merge" << ":" 
                << "CD-Running-Total:Query-Overhead-Total:witness_coverage:witness_queries:num_css" 
                << ":map_running_time:total_query_time:total_dia_time"
                << ":largest_cc_dia:sum_cc_dia:new_cc_dia:old_cc_dia:ratio_cc_dia" << endl;

  //---------------------------
  // Generate roadmap nodes
  //---------------------------
  Allstuff.StartClock("Everything");
  NodeGenClock.StartClock("Node Generation");
  typedef vector<string>::iterator I;
//  for(int iterations=0; iterations < m_iterations; ++iterations)
  while(!IsFinished())
  for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr)
  { //For each node generation method mentioned.
    vector< CfgType > vectorCfgs;
    NodeGenerationMethod<CfgType> * pNodeGen;
    //Generate nodes given 1 node gen method
    pNodeGen = GetMPProblem()->GetMPStrategy()->GetGenerateMapNodes()->GetMethod(*itr);
    pNodeGen->GenerateNodes(region, vectorCfgs);
    //cout << "NodeGen: " << *itr << "created " << vectorCfgs.size() << "(non)valid nodes" << endl;
    //Add Nodes to roadmap one by one.
    for(vector<CfgType>::iterator itr_cfg = vectorCfgs.begin();
        itr_cfg != vectorCfgs.end(); ++itr_cfg) 
    {
      if((*itr_cfg).IsLabel("VALID")) {  
        if((*itr_cfg).GetLabel("VALID")) {//Add to Free roadmap
          Clock_Class one_node;
          one_node.StartClock("one_node");
          ++m_totalSamples;               //Increment total node counter for stop criteria
          cc_create_clearance = 0.0;
          neighbor_create = neighbor_merge = neighbor_expand = neighbor_over = 0;
          merged_cc_size = old_biggest_cc_size = 0;
          ratio_cc_size = 0.0;
          new_cc_dia =  old_cc_dia = ratio_cc_dia = 0.0;
          int nNumPrevCCs = GetCCcount(*(region->roadmap.m_pRoadmap));
          vector < pair< int, VID > > cc;
          GetCCStats(*(region->roadmap.m_pRoadmap), cc);
          if(cc.size() > 0) {
            old_biggest_cc_size = cc[0].first;
          } else { old_biggest_cc_size = 0; }
          vector< CfgType > newCfg;
          newCfg.push_back(*itr_cfg);
          int newVID = region->roadmap.m_pRoadmap->AddVertex(newCfg);

          //Connect New node to roadmap.
          ConnectMap<CfgType, WeightType>* connectmap = GetMPProblem()->GetMPStrategy()->GetConnectMap();
          typedef vector<string>::iterator J;
          for(J itr = m_vecStrNodeConnectionLabels.begin(); itr != m_vecStrNodeConnectionLabels.end(); ++itr)
          {
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

          //-------------------//
          // Classify New node //
          //-------------------//
          int nNumCurCCs = GetCCcount(*(region->roadmap.m_pRoadmap));
          if(nNumCurCCs < nNumPrevCCs) {
            //cc merge
            stat_ccmerge++;
            
            region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->SetLabel("CCMERGE",true);
            merge_node_stats(region,newVID,m_nodeOverheadStat);

            new_cc_dia =  region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetStat("NEW_CC_DIA");
            old_cc_dia =  region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetStat("OLD_CC_DIA");
            ratio_cc_dia = region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetStat("RATIO_CC_DIA");
            //vector<VID> my_cc;
            //GetCC(*(region->roadmap.m_pRoadmap),newVID,my_cc);
            //merged_cc_size = my_cc.size();
            //ratio_cc_size = double(merged_cc_size) / double(old_biggest_cc_size);

            //if(new_cc_dia > max_cc_dia_found) {max_cc_dia_found = new_cc_dia;};
            
          } else if(nNumCurCCs > nNumPrevCCs) {
            //cc create
            stat_cccreate++;
            /*
            region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->SetLabel("CCCREATE",true);
            //Run cccreate node through local cccreate tests
            cc_local_area(region,newVID,m_nodeOverheadStat);

            neighbor_create = (int) region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetStat("NEIGHBOR_CREATE");
            neighbor_merge = (int) region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetStat("NEIGHBOR_MERGE");
            neighbor_expand = (int) region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetStat("NEIGHBOR_EXPAND");
            neighbor_over = (int) region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetStat("NEIGHBOR_OVER");
            CDInfo tmpcdinfo;
            CenterOfMassDistance comd;
            DistanceMetric mydm(&comd);
            cc_create_clearance = region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->
                                  ApproxCSpaceClearance(GetMPProblem()->GetEnvironment(),
                                  m_nodeOverheadStat, 
                                  GetMPProblem()->GetCollisionDetection(), tmpcdinfo,
                                  GetMPProblem()->GetDistanceMetric(),
                                  50, false);
          //  cout << endl << endl << "cc_create_clearance: " << cc_create_clearance << endl << endl;
          */
          } else if(nNumCurCCs == nNumPrevCCs) {
            /*Characterize(region,newVID,m_nodeOverheadStat);

            if(region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->IsLabel("CCEXPAND")) {  
              if(region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetLabel("CCEXPAND")) {
                ++stat_ccexpand;
              }
            } else if (region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->IsLabel("CCOVERSAMPLE")) {  
              if(region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetLabel("CCOVERSAMPLE")) {
                ++stat_ccoversample;
              }
            }

            if (region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->IsLabel("CCOVERSAMPLE_RELAX")) {  
              if(region->roadmap.m_pRoadmap->GetReferenceofData(newVID)->GetLabel("CCOVERSAMPLE_RELAX")) {
                ++stat_oversample_relaxed;
              }
            }
            */
          } else {
            cout << "Logic error .. GET OUT" << endl; exit(-1);
          }
          one_node.StopClock();
          running_time+=one_node.GetClock_SEC();
          int n_num_nodes_tmp = region->roadmap.m_pRoadmap->GetVertexCount();
          Clock_Class query_time;
          Clock_Class dia_time;
 
          query_time.StartClock("query_time");
          if((n_num_nodes_tmp % 50 == 0) || (n_num_nodes_tmp == 1)) {
            witness_qry = ConnectionsWitnessToRoadmap(m_vecWitnessNodes,&(region->roadmap),m_queryStat);
            witness_converage = 100*witness_qry.first/m_vecWitnessNodes.size();
            out_qry = 100*witness_qry.second/witness_queries;
          }
          query_time.StopClock();

          dia_time.StartClock("dia_time");
          if((n_num_nodes_tmp % 50 == 0) || (n_num_nodes_tmp == 1)) {
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
          total_query_time += query_time.GetClock_SEC();
          total_dia_time += dia_time.GetClock_SEC();
          if(out_qry==100) m_query_solved = true;

          char_ofstream << region->roadmap.m_pRoadmap->GetVertexCount() << ":" << stat_cccreate 
            << ":" << stat_ccmerge 
            << ":" << pStatClass->GetIsCollTotal()  
            << ":" << m_queryStat.GetIsCollTotal() << ":" << witness_converage
            << ":" << out_qry << ":" << nNumCurCCs << ":" << running_time 
            << ":" << total_query_time << ":" << total_dia_time
            << ":" << largest_cc_dia << ":" << sum_cc_dia 
            << ":" << new_cc_dia << ":" << old_cc_dia << ":" << ratio_cc_dia << endl;
       // cout << "  Create: " << stat_cccreate << "  Merge: " << stat_ccmerge << "  Expand: " << stat_ccexpand 
       //     << "  OverSample:  " << stat_ccoversample << endl;
        }
      }
    }
    NodeGenClock.StopClock();

  }
  cout << "Stats I have found" << endl;;
  cout << "CC-Create = " << stat_cccreate << endl;
  cout << "CC-Merge = " << stat_ccmerge << endl;
  cout << "CC-Expand = " << stat_ccexpand << endl;
  cout << "CC-OverSample = " << stat_ccoversample << endl;

  total_ofstream << "env: " << envFileName << endl;
  total_ofstream << "nodegen: " << firstNodeGen << endl;
  total_ofstream << "connection: " << firstConnection << endl;
  total_ofstream << "nodes: " << region->roadmap.m_pRoadmap->GetVertexCount() << endl;
  total_ofstream << "ccs: " << GetCCcount(*(region->roadmap.m_pRoadmap)) << endl;
  total_ofstream << "iscoll: " <<  pStatClass->GetIsCollTotal() << endl;
  total_ofstream << "time: " << NodeGenClock.GetClock_SEC() << endl;
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
  NodeGenClock.PrintClock();
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
  LOG_DEBUG_MSG("~PRMIncrementalStrategy::()");
}

bool
PRMIncrementalStrategy::
IsFinished() {
  //if(m_query_solved) {return true;}
  if(m_totalSamples < m_iterations) {return false;}
  else {return true;}
}

void 
PRMIncrementalStrategy::
Characterize(MPRegion<CfgType,WeightType>* inout_pRegion, VID in_vid, Stat_Class& Stats) {
  //     LOG_DEBUG_MSG("CCExpandCharacterizer::Characterize()");
  Roadmap<CfgType,WeightType>* pRoadmap = inout_pRegion->GetRoadmap();
  RoadmapGraph<CfgType,WeightType>* pGraph = pRoadmap->m_pRoadmap;
  LocalPlanners < CfgType, WeightType > * lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  LPOutput< CfgType, WeightType > lp_output; 
  Environment * env = GetMPProblem()->GetEnvironment();
  CollisionDetection * cd = GetMPProblem()->GetCollisionDetection();
  DistanceMetric * dm = GetMPProblem()->GetDistanceMetric();
  double pos_res = GetMPProblem()->GetEnvironment()->GetPositionRes();
  double ori_res = GetMPProblem()->GetEnvironment()->GetOrientationRes();

  vector<VID> neighbors;
  if(pGraph->GetSuccessors(in_vid, neighbors) > 1) {
    //    cout << "Pls sort me first since your not using CHECKIF SMAE CC" << endl; exit(-1);
  }
  //Next find neighbor's neighbors
  vector<VID> neighbor_neighbor;
  pGraph->GetSuccessors(neighbors[0],neighbor_neighbor);
  bool is_expansion = true;
  vector<VID> singleVID;
  singleVID.push_back(in_vid);
  //We are searching all neighbor_neighbors but will sort by dist anyways
  vector<pair<VID,VID> > kpairs = dm->FindKClosestPairs(pRoadmap, singleVID,
                                                        neighbor_neighbor, 
                                                        neighbor_neighbor.size());
  //cout << "Neighbor_neighbor has " << neighbor_neighbor.size() << " neighbors " << endl;
  //cout << "Neighbor_neighbor pairs = " << kpairs.size() << endl;
  for(vector<pair<VID,VID> >::iterator i_pair = kpairs.begin(); 
        i_pair !=kpairs.end() && is_expansion; ++i_pair)
  {  //test connection to each;
    if(pRoadmap->IsCached((*i_pair).first,(*i_pair).second)) {
      if(!pRoadmap->GetCache((*i_pair).first,(*i_pair).second)) {
        is_expansion = false;
        cout << "PRMIncrementalStrategy::Characterize failed cache" << endl;
        continue;
      }
    }

    if(!(lp->IsConnected(env, Stats, cd, dm, 
                         pGraph->GetData((*i_pair).first),
                         pGraph->GetData((*i_pair).second),
                         &lp_output, pos_res, ori_res, true))) {
      is_expansion = false; // cannot connect in_vid to neighbor_neighbor
    } else if(i_pair == kpairs.begin()) {
      pGraph->GetReferenceofData(in_vid)->SetLabel("CCOVERSAMPLE_RELAX",true);
    }
  }
  if(!is_expansion) 
    pGraph->GetReferenceofData(in_vid)->SetLabel("CCEXPAND",true);
  else
    pGraph->GetReferenceofData(in_vid)->SetLabel("CCOVERSAMPLE",true);
  //  LOG_DEBUG_MSG("~CCExpandCharacterizer::Characterize()");
}

class is_same_cc {
public:
  is_same_cc(RoadmapGraph<CfgType,WeightType> * _graph) {m_graph = _graph;}

  bool operator()(VID _v1, VID _v2) { return IsSameCC(*(m_graph), _v1,_v2); }

private: 
  RoadmapGraph<CfgType,WeightType> *m_graph;
};

double 
PRMIncrementalStrategy::
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
  double length = DijkstraSSSP(*(pGraph),_cc, &farVID);
  return_val = DijkstraSSSP(*(pGraph),farVID, &tmpVID);
  return return_val;
}

double
PRMIncrementalStrategy::
cc_max_length(RoadmapGraph<CfgType,WeightType>* pGraph, VID _cc) {
  DistanceMetric* dm = GetMPProblem()->GetDistanceMetric();
  Environment* env = GetMPProblem()->GetEnvironment();

  vector<VID> cc_vids;
  GetCC ( *pGraph, _cc, cc_vids);
  double return_val = 0.0;
  for(int i=0; i<cc_vids.size(); ++i) {
   for(int j=0; j<cc_vids.size(); ++j) { 
    double length =0.0;
    if( i == j) continue;
    length = dm->Distance(env, pGraph->GetData(cc_vids[i]),
                               pGraph->GetData(cc_vids[j]));
    if(length > return_val) { return_val = length;}
    }
  }
  return return_val;
}

void 
PRMIncrementalStrategy::
merge_node_stats(MPRegion<CfgType,WeightType>* inout_pRegion, VID in_vid, Stat_Class& Stats) {
  Roadmap<CfgType,WeightType>* pRoadmap = inout_pRegion->GetRoadmap();
  RoadmapGraph<CfgType,WeightType>* pGraph = pRoadmap->m_pRoadmap;
  LocalPlanners < CfgType, WeightType > * lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  LPOutput< CfgType, WeightType > lp_output; 
  Environment * env = GetMPProblem()->GetEnvironment();
  CollisionDetection * cd = GetMPProblem()->GetCollisionDetection();
  DistanceMetric * dm = GetMPProblem()->GetDistanceMetric();
  double pos_res = env->GetPositionRes();
  double ori_res = env->GetOrientationRes();

  vector<VID> successors;
  pGraph->GetSuccessors(in_vid,successors);
  
  typedef pair<pair<VID,VID>, WeightType > vid_vid_weight;
  vector< vid_vid_weight > edge_history;
  map< pair<VID,VID> , WeightType> tempMap;
  
  for(int i=0; i<successors.size(); ++i) {
    //Save all weights and remove.
    vid_vid_weight first_edge = vid_vid_weight(pair<VID,VID>(in_vid,successors[i]),
                                               pGraph->GetEdgeWeight(in_vid,successors[i]));
    vid_vid_weight second_edge = vid_vid_weight(pair<VID,VID>(successors[i],in_vid),
                                                pGraph->GetEdgeWeight(successors[i],in_vid));
    tempMap[pair<VID,VID>(in_vid,successors[i])] =
                  pGraph->GetEdgeWeight(in_vid,successors[i]);
    tempMap[pair<VID,VID>(successors[i],in_vid)] =
                  pGraph->GetEdgeWeight(successors[i],in_vid);
    
    edge_history.push_back(first_edge);
    edge_history.push_back(second_edge);
    
    pGraph->DeleteEdge(in_vid,VID(successors[i]));
    pGraph->DeleteEdge(VID(successors[i]),in_vid);
  }

  successors.erase(std::unique(successors.begin(),
                               successors.end(),
                               is_same_cc(pGraph)),
                   successors.end());
  
  vector< double > vec_dist;
  for(int j=0; j<successors.size(); ++j) {
  //  cout << endl << endl << "Calling Dijkstra" << endl << endl;
  //  RoadmapGraph<CfgType,WeightType> *result; //check roadmap's graph and make the same type
  //  result = new RoadmapGraph<CfgType,WeightType>;
    double length = cc_diamater(pGraph,successors[j]);
  //  result->EraseGraph();
  //  cout << "Dijkstra finished" << endl;
  //  delete result;
    vec_dist.push_back(length);
  }

  for(int k=0; k<edge_history.size(); ++k) {
    pGraph->AddEdge(edge_history[k].first.first, 
                    edge_history[k].first.second, 
                    edge_history[k].second);
  }
  double old_cc_dia = vec_dist[0];
  for(int k=0; k<vec_dist.size(); ++k)
    { if (vec_dist[k] > old_cc_dia) {old_cc_dia = vec_dist[k]; }}
  
  double new_cc_dia = cc_diamater(pGraph,in_vid);
/*
  double max = vec_vid_dist[0].second;
  VID max_vid = vec_vid_dist[0].first;
  double sum = vec_vid_dist[0].second; 
  sum+= tempMap[pair<VID,VID>(in_vid,vec_vid_dist[0].first)].Weight();
  if(vec_vid_dist.size() > 1)
  for(int z=1; z<vec_vid_dist.size(); ++z) {
    if(vec_vid_dist[z].second > max) {
      max = vec_vid_dist[z].second; 
      max_vid = vec_vid_dist[z].first; 
    }
    sum += vec_vid_dist[z].second; 
    sum+= tempMap[pair<VID,VID>(in_vid,vec_vid_dist[z].first)].Weight();
  }
  max+=  tempMap[pair<VID,VID>(in_vid,max_vid)].Weight();
  //max+=  tempMap[pair<VID,VID>(in_vid,vec_vid_dist[max_vid].first)].Weight();
*/
  pGraph->GetReferenceofData(in_vid)->SetStat("NEW_CC_DIA", new_cc_dia);
  pGraph->GetReferenceofData(in_vid)->SetStat("OLD_CC_DIA", old_cc_dia);
  if(old_cc_dia != double(0)) {
    pGraph->GetReferenceofData(in_vid)->SetStat("RATIO_CC_DIA", new_cc_dia/old_cc_dia);
  } else {
    pGraph->GetReferenceofData(in_vid)->SetStat("RATIO_CC_DIA", new_cc_dia/1);
  }
}



void 
PRMIncrementalStrategy::
cc_local_area(MPRegion<CfgType,WeightType>* inout_pRegion, VID in_vid, Stat_Class& Stats) {
 //     LOG_DEBUG_MSG("CCExpandCharacterizer::Characterize()");
  Roadmap<CfgType,WeightType>* pRoadmap = inout_pRegion->GetRoadmap();
  RoadmapGraph<CfgType,WeightType>* pGraph = pRoadmap->m_pRoadmap;
  LocalPlanners < CfgType, WeightType > * lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  LPOutput< CfgType, WeightType > lp_output; 
  Environment * env = GetMPProblem()->GetEnvironment();
  CollisionDetection * cd = GetMPProblem()->GetCollisionDetection();
  DistanceMetric * dm = GetMPProblem()->GetDistanceMetric();
  double pos_res = GetMPProblem()->GetEnvironment()->GetPositionRes();
  double ori_res = GetMPProblem()->GetEnvironment()->GetOrientationRes();
  //We want to find local area stats.  Num neighbors that are cc_create, cc_merge, cc_expand, cc_oversample

  vector<VID> graph_vids;
  pGraph->GetVerticesVID(graph_vids);
  vector<VID> vec_vid;
  vec_vid.push_back(in_vid);
  vector<pair<VID,VID> > kpairs = dm->FindKClosestPairs(pRoadmap, vec_vid,
                                                        graph_vids, 10); 

  int ncreate,nmerge,nexpand,nover;
  ncreate = nmerge = nexpand = nover =0;
  for(int i=0; i<kpairs.size(); ++i) {
    if(kpairs[i].second == in_vid) {
      cout << "error incorrect vid" << endl; exit(-1);
    }
    if(pGraph->GetReferenceofData(kpairs[i].second)->IsLabel("CCCREATE")) {
      if(pGraph->GetReferenceofData(kpairs[i].second)->GetLabel("CCCREATE")) {
        ncreate++;
      }
    }

    if(pGraph->GetReferenceofData(kpairs[i].second)->IsLabel("CCMERGE")) {
      if(pGraph->GetReferenceofData(kpairs[i].second)->GetLabel("CCMERGE")) {
        nmerge++;
      }
    }

    if(pGraph->GetReferenceofData(kpairs[i].second)->IsLabel("CCEXPAND")) {
      if(pGraph->GetReferenceofData(kpairs[i].second)->GetLabel("CCEXPAND")) {
        nexpand++;
      }
    }

    if(pGraph->GetReferenceofData(kpairs[i].second)->IsLabel("CCOVERSAMPLE")) {
      if(pGraph->GetReferenceofData(kpairs[i].second)->GetLabel("CCOVERSAMPLE")) {
        nover++;
      }
    }
  }

  cout << "I am a CC-create node with neighborhood: create = " << ncreate 
       << " merge = " << nmerge << " expand = " << nexpand << " oversample " << nover << endl;

  pGraph->GetReferenceofData(in_vid)->SetStat("NEIGHBOR_CREATE", ncreate);
  pGraph->GetReferenceofData(in_vid)->SetStat("NEIGHBOR_MERGE", nmerge);
  pGraph->GetReferenceofData(in_vid)->SetStat("NEIGHBOR_EXPAND", nexpand);
  pGraph->GetReferenceofData(in_vid)->SetStat("NEIGHBOR_OVER", nover);

}

bool 
PRMIncrementalStrategy::
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




pair < unsigned int, unsigned int >
PRMIncrementalStrategy::
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
