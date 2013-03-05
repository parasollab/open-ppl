/////////////////////////
//Class BlindRRT
////////////////////////

#ifndef BLINDRRT_H_
#define BLINDRRT_H_

#include "MPStrategyMethod.h"
#include <boost/pointer_cast.hpp>
#include "graph/algorithms/count_hop_pairs.h"
#include "Utilities/MPUtils.h"
/*#include "IOUtils.h"
#include "MetricUtils.h"*/

template<class MPTraits>
class BlindRRT : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    BlindRRT();
    BlindRRT(MPProblemType* _problem, XMLNodeReader& _node, bool _warnXML = true);
    
    void InitializeParallel(CfgType _root, CfgType& _regionCand, vector<CfgType>* _neighbors, 
        bool _strictBranching, double _overlap, double _radius);

    virtual ~BlindRRT();

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os);

  protected:
    // Helper functions
    CfgType GoalBiasedDirection();
    CfgType SelectDir();
    virtual int ExpandTree(CfgType& _dir);
    void ConnectCCs();
    VID GetClosestCC(VID _node, VID _nodeCCVID);
   
    bool GetValidity(CfgType& _cfg);

    void EvaluateGoals();
    void RemoveInvalidNodes();

    vector<string> m_evaluators;
    string m_lp;
    string m_dm;
    string m_nf;
    string m_vc;
    Query<MPTraits>* m_query;
    string m_nc;
    double m_delta, m_minDist;
    bool m_evaluateGoal;
    vector<CfgType> m_goals, m_roots;
    vector<size_t> m_goalsNotFound;
    // how are the CCs are being connected, closest, random? 
    string m_CCconnection;
    size_t m_initialSamples;
    string m_expansionType; // describes how cfgs are added along the expansion

/*  Data members for parallel implementation */    
    CfgType m_regionCand;
    vector<CfgType>* m_neighbors;
    double m_overlap;
    double m_radius;
    vector<VID> m_branch; // keep track of the local branch
    bool m_strictBranching;

};

template<class MPTraits>
BlindRRT<MPTraits>::BlindRRT(): m_query(NULL){
  this->SetName("BlindRRT");
}

/* Parameters initilized in WorkFunction */
template<class MPTraits>
void
BlindRRT<MPTraits>::InitializeParallel(CfgType _root, CfgType& _regionCand, vector<CfgType>* _neighbors, 
    bool _strictBranching, double _overlap, double _radius) {

  m_roots.push_back(_root);
  m_branch.push_back(0);
  m_regionCand = _regionCand;
  m_neighbors = _neighbors;
  m_overlap = _overlap;
  m_radius = _radius;
  m_strictBranching = _strictBranching;

}



template<class MPTraits>
BlindRRT<MPTraits>::BlindRRT(MPProblemType* _problem, XMLNodeReader& _node, bool _warnXML) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_query(NULL){
    this->SetName("BlindRRT");
    ParseXML(_node);
    if (_warnXML) _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
BlindRRT<MPTraits>::~BlindRRT(){
  if(m_query != NULL)
    delete m_query;
}

template<class MPTraits>
void
BlindRRT<MPTraits>::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "Evaluator"){
      string evalMethod = citr->stringXMLParameter("label", true, "", "Evaluation Method");
      m_evaluators.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    } 
    else
      citr->warnUnknownNode();
  }

  m_delta = _node.numberXMLParameter("delta", false, 1.0, 0.0, MAX_DBL, "Delta Distance");
  m_minDist = _node.numberXMLParameter("minDist", false, 0.0, 0.0, MAX_DBL, "Minimum Distance");
  m_vc = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
  m_nf = _node.stringXMLParameter("nfLabel", true, "", "Neighborhood Finder");
  m_dm = _node.stringXMLParameter("dmLabel",true,"","Distance Metric");
  m_lp = _node.stringXMLParameter("lpLabel", true, "", "Local Planning Method");
  m_nc = _node.stringXMLParameter("connectorLabel",false,"","Node Connection Method");
  m_CCconnection = _node.stringXMLParameter("CCconnection",true,"","CC connection strategy");
  m_expansionType = _node.stringXMLParameter("expansionType",true,"","Expansion strategy (All, Jump, Blind)");
  m_initialSamples = _node.numberXMLParameter("initialSamples", true, 0, 0, MAX_INT, "Initial Sample size");
  m_evaluateGoal = _node.boolXMLParameter("evaluateGoal", false, false, "");

  //optionally read in a query and create a Query object.
  string query = _node.stringXMLParameter("query", false, "", "Query Filename");
  if(query != ""){
    m_query = new Query<MPTraits>(query);
    m_query->SetMPProblem(this->GetMPProblem());
    m_query->SetDebug(this->m_debug);
  }
}

template<class MPTraits>
void
BlindRRT<MPTraits>::PrintOptions(ostream& _os) {
  typedef vector<string>::iterator SIT;
  _os << "BlindRRT::PrintOptions" << endl;
  _os << "\tNeighorhood Finder:: " << m_nf << endl;
  _os << "\tDistance Metric:: " << m_dm << endl;
  _os << "\tValidity Checker:: " << m_vc << endl;
  _os << "\tLocal Planner:: " << m_lp << endl;
  _os << "\tConnection Method:: " << m_nc << endl;
  _os << "\tCC Connection:: " << m_CCconnection << endl;
  _os << "\tExpansion Type:: " << m_expansionType << endl;
  _os << "\tInitial Samples:: " << m_initialSamples << endl;
  _os << "\tEvaluate Goal:: " << m_evaluateGoal << endl;
  _os << "\tEvaluators:: " << endl;
  for(SIT sit = m_evaluators.begin(); sit!=m_evaluators.end(); sit++)
    _os << "\t\t" << *sit << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
}

//////////////////////
//Initialization Phase
/////////////////////
template<class MPTraits>
void 
BlindRRT<MPTraits>::Initialize(){
  if(this->m_debug) cout<<"\nInitializing BlindRRT::"<<endl;
  SRand(time(NULL));
  // Setup MP variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CDInfo cdInfo;
  string callee = "BlindRRT::RRT";
  // Setup RRT Variables
  CfgType tmp;
  if(m_query != NULL){
    vector<CfgType>& queryCfgs = m_query->GetQuery();
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit1 = queryCfgs.begin(), cit2 = cit1+1; cit2!=queryCfgs.end(); cit1++, cit2++){
      if (!this->GetMPProblem()->GetValidityChecker(m_vc)->
          IsValid(*cit1, env, *stats, cdInfo, &callee)){
        cit1->SetStat("Validity", NodeState::COLLISION);
      } else {
        cit1->SetStat("Validity", NodeState::FREE);
      }
      if (!this->GetMPProblem()->GetValidityChecker(m_vc)->
          IsValid(*cit2, env, *stats, cdInfo, &callee)){
        cit2->SetStat("Validity", NodeState::COLLISION);
      } else {
        cit2->SetStat("Validity", NodeState::FREE);
      }
      
      m_roots.push_back(*cit1);
      m_goals.push_back(*cit2);
      m_goalsNotFound.push_back(m_goals.size()-1);
    }
  }
  else{
    // Add root vertex/vertices
    tmp.GetRandomCfg(env);
    if (tmp.InBoundary(env)
        && this->GetMPProblem()->GetValidityChecker(m_vc)->IsValid(tmp, env, *stats, cdInfo, &callee)){
      m_roots.push_back(tmp);
      m_goals.push_back(tmp);
      m_goalsNotFound.push_back(1);
    }
  }

  for(typename vector<CfgType>::iterator C = m_roots.begin(); C!=m_roots.end(); C++){
    VID vid = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(*C);
     m_branch.push_back(vid);
  }

  if(this->m_debug) cout<<"\nEnding Initializing BlindRRT"<<endl;
}

////////////////
//Run/Start Phase
////////////////
template<class MPTraits>
void
BlindRRT<MPTraits>::Run() {
  if(this->m_debug) cout << "\nRunning BlindRRT::" << endl;

  // Setup MP Variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  stats->StartClock("BlindRRT Generation");

  bool mapPassedEvaluation = false;
  size_t samples = 0;
  while(!mapPassedEvaluation && samples < m_initialSamples){
    CfgType dir = SelectDir();
    //grow towards the direction
    int samplesMade = this->ExpandTree(dir);
    samples += samplesMade;

    //if(m_evaluateGoal)
    //  EvaluateGoals();
    //evaluate the roadmap
    bool evalMap = this->EvaluateMap(m_evaluators);
    mapPassedEvaluation = evalMap && ((m_evaluateGoal && m_goalsNotFound.size()==0) || !m_evaluateGoal);

    if( m_goalsNotFound.size()==0 && this->m_debug)
      cout << "RRT FOUND ALL GOALS" << endl;
  }

  // Did we exit because we found a goal, or because we met the number of nodes?
  if((m_evaluateGoal && m_goalsNotFound.size() !=0) || !m_evaluateGoal) {
    // Get VALID CCs
    RemoveInvalidNodes();
    ConnectCCs();
  }
  stats->StopClock("BlindRRT Generation");
  if(this->m_debug) {
    stats->PrintClock("BlindRRT Generation", cout);
    cout<<"\nEnd Running BlindRRT::" << endl;  
  }
}

/////////////////////
//Finalization phase
////////////////////
template<class MPTraits>
void
BlindRRT<MPTraits>::Finalize() {

  if(this->m_debug) cout<<"\nFinalizing BlindRRT::"<<endl;

  //setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string str;

  //perform query if query was given as input
  if(m_query != NULL){
    str = this->GetBaseFilename() + ".path";
    m_query->SetPathFile(str);
    if(m_evaluateGoal){
      if(m_query->PerformQuery(this->GetMPProblem()->GetRoadmap(), *stats)){
        if(this->m_debug) cout << "Query successful! Output written to " << str << "." << endl;
      }
      else{
        if(this->m_debug) cout << "Query unsuccessful." << endl;
      }
    }
  }

  //output final map
  str = this->GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();

  //output stats
  str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  stats->PrintClock("BlindRRT Generation", osStat);
  osStat.close();

  if(this->m_debug) cout<<"\nEnd Finalizing BlindRRT"<<endl;
}

template<class MPTraits>
bool
BlindRRT<MPTraits>::GetValidity(CfgType& _cfg){
  
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vc);
  string callee("BlindRRT::GetValidity");
  CDInfo  cdInfo;
  
  if(!_cfg.IsLabel("VALID"))  
    vc->IsValid(_cfg, env, *stats, cdInfo, &callee); 
  
  return _cfg.GetLabel("VALID");
}

template<class MPTraits>
typename MPTraits::CfgType 
BlindRRT<MPTraits>::SelectDir(){
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CfgType dir;
  if(m_neighbors == NULL || m_neighbors->size() == 0) // we never initialized or there are no neighbors
    dir.GetRandomCfg(env);
  else
    dir = SelectDirection (m_regionCand, *m_neighbors, m_radius, m_overlap);
  return dir; 
}

/* Expands Tree towards _dir, and returns the number of new nodes added  */

template<class MPTraits>
int
BlindRRT<MPTraits>::ExpandTree(CfgType& _dir){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vc);
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lp);
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  LPOutput<MPTraits> lpOutput;
  VID recentVID = INVALID_VID;
  CDInfo  cdInfo;
  string callee("BlindRRT::ExpandTree");
  // Find closest Cfg in map
  vector<VID> kClosest;
  vector<CfgType> cfgs;

  string kcloseClockName = "kclosest time ";
  stats->StartClock(kcloseClockName);
  
  nf->KClosest(rdmp, m_branch.begin(), m_branch.end(), _dir, 1, back_inserter(kClosest));
  
  stats->StopClock(kcloseClockName);

  VID nearestVID = kClosest[0];
  CfgType nearest;
  
  nearest = (rdmp->GetGraph()->GetCfg(nearestVID));

  string expandClockName = "BlindRRTExpand time ";
  stats->StartClock(expandClockName);
  
  ExpansionType::Expansion expansion;
  vector<pair<CfgType, int> > expansionCfgs;  // this will contain all cfgs from start to goal inclusive
  expansionCfgs.push_back(make_pair(nearest, 0));
  expansion = BlindRRTExpand<MPTraits>(this->GetMPProblem(), m_vc, m_dm, m_expansionType, 
      nearest, _dir, expansionCfgs, m_delta, cdInfo, 
      env->GetPositionRes(), env->GetOrientationRes());

  stats->StopClock(expandClockName);
  
  if (expansion == ExpansionType::NO_EXPANSION) {  
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
    return 0;
  }
  
  CfgType& newCfg = expansionCfgs.back().first; // last cfg in the returned array is delta away from nearest

  if(this->m_debug) cout<<"RRT expanded"<<endl;
  // If good to go, add to roadmap
  if(dm->Distance(env, newCfg, nearest) >= m_minDist && expansion != ExpansionType::OUT_OF_BOUNDARY ) {
    
    // Adding Nodes
    vector<VID> expansionVIDs;
    expansionVIDs.push_back(nearestVID);
    // expansionCfgs.front().first.SetStat("Parent", kClosest[0]);

    // we already added startCfg remember?
    for(size_t i=1; i<expansionCfgs.size(); i++ ) {
      CfgType& cfg2 = expansionCfgs[i].first;
      VID newVID = rdmp->GetGraph()->AddVertex(expansionCfgs[i].first);
      expansionVIDs.push_back(newVID );
      m_branch.push_back(newVID);
    }
    
    pair<WeightType, WeightType> weights;
    // Adding Edges
    int weight;
    for(size_t i=1; i<expansionCfgs.size(); i++ ) {

      // For some reason, not all nodes have the VALID label
      if(!expansionCfgs[i-1].first.IsLabel("VALID")) { 
        vc->IsValid(expansionCfgs[i-1].first, env, *stats, cdInfo, &callee); 
      }
      if(!expansionCfgs[i].first.IsLabel("VALID")) { 
        vc->IsValid(expansionCfgs[i].first, env, *stats, cdInfo, &callee); 
      }

      if(expansionCfgs[i-1].first.GetLabel("VALID") &&  
            expansionCfgs[i].first.GetLabel("VALID")) {
        weight = expansionCfgs[i].second - expansionCfgs[i-1].second; // Edge weight 
        weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
        rdmp->GetGraph()->AddEdge(expansionVIDs[i-1], expansionVIDs[i], weights);
      }
      
      if(expansion == ExpansionType::JUMPED) // we can only add one edge, start -> middle  
        break;
    }
    return expansionCfgs.size() - 1;    // substract one cause the start already belonged to the tree
  } else {  // did not reach minDist :(
    return 0;
  }

}

/*
 *  Get two random CCs and attemp a connection
 *  then reevaluate CCs
 */
template<class MPTraits>
void 
BlindRRT<MPTraits>::ConnectCCs() {

  //Setup MP variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);
  ConnectorPointer pConnection = this->GetMPProblem()->GetConnector(m_nc);

  stringstream clockName; clockName << "Component Connection";
  stats->StartClock(clockName.str());

  stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
  vector< pair<size_t,VID> > ccs;
  stapl::sequential::get_cc_stats(*(rdmp->GetGraph()),cmap, ccs);
  cmap.reset();
  if(ccs.size()==1) return;

  vector<VID> cc1;
  vector<VID> cc2;
  VID cc1VID; 
  VID cc2VID; 

  bool alt = false;
  bool mapPassedEvaluation = false;

  while(ccs.size() > 1 && !mapPassedEvaluation) {

    int rand1 = LRand() % ccs.size();
    // always expand from the root CC
    cc1VID = ccs[ 0 ].second; 

    stapl::sequential::get_cc(*(rdmp->GetGraph()),cmap,cc1VID,cc1);
    cmap.reset();
    if (cc1.size() == 1) {

      CfgType cfg = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetCfg(cc1[0]);
      if (!cfg.GetLabel("VALID"))
        continue;
    }
    if (m_CCconnection == "Random") {

      int rand2 = LRand() % ccs.size();
      if (rand1 == rand2) continue;
      cc2VID = ccs[ rand2 ].second; 

    } else if(m_CCconnection == "ClosestNode") {
      VID randomNode = cc1[LRand() % cc1.size()];
      cc2VID = GetClosestCC(randomNode, cc1VID);

    } else if (m_CCconnection == "ClosestCC") {
      // To be implemented 
    } else if(m_CCconnection == "Mixed") {
      if(alt) {
        int rand2 = LRand() % ccs.size();
        if (rand1 == rand2) continue;
        cc2VID = ccs[ rand2 ].second; 

      } else {
        VID randomNode = cc1[LRand() % cc1.size()];
        cc2VID = GetClosestCC(randomNode, cc1VID);
      }
      alt = !alt;

    } else {
      cout << "Unknown CC connection type: " << m_CCconnection << endl;
      exit(-1);

    }
    stapl::sequential::get_cc(*(rdmp->GetGraph()),cmap,cc2VID,cc2);
    cmap.reset();

    // Maybe this is an invalid node, don't use it
    if (cc2.size() == 1) {

      CfgType cfg = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetCfg(cc2[0]);
      if (!cfg.GetLabel("VALID") )
        continue;

    }
    // We got a pair of CCs, attempt to Connect them!
    pConnection->Connect(rdmp, *stats, cmap, cc1.begin(), cc1.end(), cc2.begin(), cc2.end()) ;

    stapl::sequential::get_cc_stats(*(rdmp->GetGraph()),cmap, ccs);
    cmap.reset();
    
    //if(m_evaluateGoal)
    //  EvaluateGoals();
    
    //evaluate the roadmap
    bool evalMap = this->EvaluateMap(m_evaluators);
    //mapPassedEvaluation = m_evaluateGoal && m_goalsNotFound.size()==0;
    mapPassedEvaluation = evalMap;
  }


  stats->StopClock(clockName.str());

}


template<class MPTraits>
typename BlindRRT<MPTraits>::VID 
BlindRRT<MPTraits>::GetClosestCC(VID _node, VID _nodeCCVID) {

  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);

  stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
  vector< pair<size_t,VID> > ccs;
  stapl::sequential::get_cc_stats(*(rdmp->GetGraph()),cmap, ccs);

  typedef typename vector<pair<size_t, VID> >::iterator CCSIT;

  // Key = nodeVID, Value = CCVID
  // For easy retrieval of the CC once the closest node is found
  map<VID, VID> nodesAndCCs;
  typedef typename vector<pair<size_t, VID> >::iterator CCSIT;
  vector<VID> closestNodesOtherCCs;
  //find closest VID from other CCS
  for(CCSIT ccsit = ccs.begin(); ccsit!=ccs.end(); ccsit++){
    
    if(ccsit->second == _nodeCCVID)
      continue;
    
    vector<VID> cc;
    stapl::sequential::get_cc(*(rdmp->GetGraph()),cmap,ccsit->second,cc);
    cmap.reset();
    vector<VID> closest;
    nf->KClosest(rdmp, cc.begin(), cc.end(), _node, 1, back_inserter(closest));
    if (closest.size() != 0) { 
      closestNodesOtherCCs.push_back(closest[0]);
      nodesAndCCs[closest[0]] = ccsit->second; 
    }
  }

  //find closest VID from other CCS reps
  vector<VID> closestNode;
  nf->KClosest(rdmp, closestNodesOtherCCs.begin(), 
      closestNodesOtherCCs.end(), _node, 1, back_inserter(closestNode));

  VID closestCC = nodesAndCCs[ closestNode[0] ];

  return closestCC;
}


template<class MPTraits>
void 
BlindRRT<MPTraits>::RemoveInvalidNodes() {

  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vc);
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  CDInfo  cdInfo;
  string callee("RemoveInvalidNodes");
  
  vector<VID> allVIDs;
  rdmp->GetGraph()->GetVerticesVID(allVIDs);

  for (size_t i=0; i<allVIDs.size(); i++) {
    CfgType cfg = (rdmp->GetGraph()->GetCfg(allVIDs[i]));
    
    if (!cfg.IsLabel("VALID") ) {
      vc->IsValid(cfg, env, *stats, cdInfo, &callee); 
      
    }
    
    if (!cfg.GetLabel("VALID") ) {
      rdmp->GetGraph()->delete_vertex(allVIDs[i]); 
      //VDRemoveNode(cfg); 
    }
  }

}

template<class MPTraits>
void
BlindRRT<MPTraits>::EvaluateGoals(){
  // Setup MP Variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dmp = this->GetMPProblem()->GetDistanceMetric(m_dm);
  LocalPlannerPointer lpp = this->GetMPProblem()->GetLocalPlanner(m_lp);
  NeighborhoodFinderPointer nfp = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);
  LPOutput<MPTraits> lpOutput;
  // Check if goals have been found
  for(vector<size_t>::iterator i = m_goalsNotFound.begin(); i!=m_goalsNotFound.end(); i++){
    vector<VID> closests;
    nfp->KClosest(this->GetMPProblem()->GetRoadmap(), m_goals[*i], 1, back_inserter(closests));     
    CfgType& closest = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetCfg(closests[0]);
    double dist = dmp->Distance(env, m_goals[*i], closest);
    if(this->m_debug) cout << "Distance to goal::" << dist << endl;
    CfgType col;
    if(dist < m_delta && lpp->IsConnected(env, *stats, dmp, closest, m_goals[*i], col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true, false, false)){
      if(this->m_debug) cout << "Goal found::" << m_goals[*i] << endl;
      if(!(this->GetMPProblem()->GetRoadmap()->GetGraph()->IsVertex( m_goals[*i])))
        this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(m_goals[*i]);
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(closest, m_goals[*i], lpOutput.edge);
      m_goalsNotFound.erase(i);
      i--;
    }
  }
}

#endif
