/**
 * BasicRRTStrategy.cpp
 *
 * Description: Main RRT Strategy, contains RRT method used in RRTconnect
 *
 * Author: Kasra Manavi
 * Last Edited: 04/08/2011
 */

#include "BasicRRTStrategy.h"
#include "MPProblem.h"
#include "MPRegion.h"
#include "MPStrategy.h"
#include "MapEvaluator.h"
#include "Sampler.h"

BasicRRTStrategy::BasicRRTStrategy(XMLNodeReader& _node, MPProblem* _problem) :
  MPStrategyMethod(_node, _problem), m_currentIteration(0){
    ParseXML(_node);
  }

void BasicRRTStrategy::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter("Method", true, "", "Evaluation Method");
      m_evaluators.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    } 
    else
      citr->warnUnknownNode();
  }

  m_delta = _node.numberXMLParameter("delta", false, 0.05, 0.0, 1.0, "Delta Distance");
  m_minDist = _node.numberXMLParameter("minDist", false, 0.0, 0.0, 1.0, "Minimum Distance");
  m_numRoots = _node.numberXMLParameter("numRoots", false, 1, 0, MAX_INT, "Number of Roots");
  m_growthFocus = _node.numberXMLParameter("growthFocus", false, 0.0, 0.0, 1.0, "#GeneratedTowardsGoal/#Generated");
  m_sampler = _node.stringXMLParameter("sampler", true, "", "Sampler Method");
  m_vc = _node.stringXMLParameter("vc", true, "", "Validity Test Method");
  m_nf = _node.stringXMLParameter("nf", true, "", "Neighborhood Finder");
  m_dm = _node.stringXMLParameter("dm",true,"","Distance Metric");
  m_lp = _node.stringXMLParameter("lp", true, "", "Local Planning Method");
  m_query = _node.stringXMLParameter("query", false, "", "Query Filename");
  _node.warnUnrequestedAttributes();

  if(m_debug) PrintOptions(cout);
}

void BasicRRTStrategy::PrintOptions(ostream& _os) {
  typedef vector<string>::iterator SIT;
  _os << "BasicRRTStrategy::PrintOptions" << endl;
  _os << "\tSampler:: " << m_sampler << endl;
  _os << "\tNeighorhood Finder:: " << m_nf << endl;
  _os << "\tDistance Metric:: " << m_dm << endl;
  _os << "\tValidity Checker:: " << m_vc << endl;
  _os << "\tLocal Planner:: " << m_lp << endl;
  _os << "\tEvaluators:: " << endl;
  for(SIT sit = m_evaluators.begin(); sit!=m_evaluators.end(); sit++)
    _os << "\t\t" << *sit << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
  _os << "\tnumber of roots:: " << m_numRoots << endl;
  _os << "\tgrowth focus:: " << m_growthFocus << endl;
}

void 
BasicRRTStrategy::Initialize(int _regionID){
  if(m_debug) cout<<"\nInitializing BasicRRTStrategy::"<<_regionID<<endl;
  
  // Setup MP variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  StatClass* regionStats = region->GetStatClass();
  Environment* env = region->GetRoadmap()->GetEnvironment();
  ValidityChecker<CfgType>* vc = GetMPProblem()->GetValidityChecker();
  CDInfo cdInfo;
  string callee = "BasicRRTStrategy::RRT";
  // Setup RRT Variables
  CfgType tmp;
  if(m_query != ""){
    ifstream ifs(m_query.c_str());
    while(1){
      tmp.Read(ifs);
      if(!ifs) break;
      m_roots.push_back(tmp);
      tmp.Read(ifs);
      m_goals.push_back(tmp);
      m_goalsNotFound.push_back(m_goals.size()-1);
    }
  }
  else{
    // Add root vertex/vertices
    for (int i=0; i<m_numRoots; ++i) {
      tmp.GetRandomCfg(env);
      if (tmp.InBoundingBox(env)
          && vc->IsValid(vc->GetVCMethod(m_vc), tmp, env, *regionStats, cdInfo, true, &callee)){
        m_roots.push_back(tmp);
        m_goals.push_back(tmp);
        m_goalsNotFound.push_back(m_goals.size()-1);
      }
      else 
        --i;
    }
  }
  for(vector<CfgType>::iterator C = m_roots.begin(); C!=m_roots.end(); C++){
    region->GetRoadmap()->m_pRoadmap->AddVertex(*C);
  }

  if(m_debug) cout<<"\nEnding Initializing BasicRRTStrategy"<<endl;
}

void BasicRRTStrategy::Run(int _regionID) {
  if(m_debug) cout << "\nRunning BasicRRTStrategy::" << _regionID << endl;

  // Setup MP Variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  StatClass* regionStats = region->GetStatClass();

  regionStats->StartClock("RRT Generation");

  CfgType dir;
  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation){
    //find my growth direction. Default is too randomly select node or bias towards a goal
    double randomRatio = DRand();
    if(randomRatio<m_growthFocus){
      dir = GoalBiasedDirection(_regionID);
    }
    else{
      dir = this->SelectDirection(_regionID);
    }
    
    //grow towards the direction
    VID recent = this->ExpandTree(_regionID, dir);
    if(recent != INVALID_VID){
      //conntect various trees together
      ConnectTrees(_regionID, recent);
      //see if tree is connected to goals
      EvaluateGoals(_regionID);
    }
    //evaluate the roadmap
    mapPassedEvaluation = EvaluateMap(_regionID);

    if(m_goalsNotFound.size()==0){
      if(m_debug) cout << "RRT FOUND ALL GOALS" << endl;
      mapPassedEvaluation = true;
    }
 }

  regionStats->StopClock("RRT Generation");
  if(m_debug) {
    regionStats->PrintClock("RRT Generation", cout);
    cout<<"\nEnd Running BasicRRTStrategy::" << _regionID << endl;  
  }
}

void BasicRRTStrategy::Finalize(int _regionID) {

  if(m_debug) cout<<"\nFinalizing BasicRRTStrategy::"<<_regionID<<endl;

  //setup region variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  StatClass* regionStats = region->GetStatClass();
  string str;

  //output final map
  str = GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  region->WriteRoadmapForVizmo(osMap);
  osMap.close();

  //output stats
  str = GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  regionStats->PrintAllStats(osStat, region->GetRoadmap());
  regionStats->PrintClock("RRT Generation", osStat);
  osStat.close();

  if(m_debug) cout<<"\nEnd Finalizing BasicRRTStrategy"<<_regionID<<endl;
}

CfgType
BasicRRTStrategy::GoalBiasedDirection(int _regionID){
  // Determine direction, make sure goal not found
  if (m_goalsNotFound.size() == 0){
    return CfgType(); 
  }
  else {
    size_t goalNum = LRand()%m_goalsNotFound.size();
    return m_goals[m_goalsNotFound[goalNum]];
  }
}

CfgType 
BasicRRTStrategy::SelectDirection(int _regionID){
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  Environment* env = region->GetRoadmap()->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}
    
BasicRRTStrategy::VID 
BasicRRTStrategy::ExpandTree(int _regionID, CfgType& _dir){
  // Setup MP Variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  Environment* env = GetMPProblem()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  VID recentVID = INVALID_VID;
  // Find closest Cfg in map
  vector<VID> kClosest;
  vector<CfgType> cfgs;

  nf->KClosest(nf->GetNFMethod(m_nf), region->GetRoadmap(), _dir, 1, back_inserter(kClosest));     
  CfgType nearest = region->GetRoadmap()->m_pRoadmap->find_vertex(kClosest[0])->property();
  CfgType newCfg;

  if(!RRTExpand(GetMPProblem(), _regionID, m_vc, m_dm, nearest, _dir, newCfg, m_delta)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
    return recentVID;
  }
  // If good to go, add to roadmap
  if(dm->Distance(env, newCfg, nearest) >= m_minDist) {
    recentVID = region->GetRoadmap()->m_pRoadmap->AddVertex(newCfg);
    //TODO fix weight
    pair<WeightType, WeightType> weights = make_pair(WeightType(), WeightType());
    region->GetRoadmap()->m_pRoadmap->AddEdge(nearest, newCfg, weights);
  } 
  return recentVID;
}

void 
BasicRRTStrategy::ConnectTrees(int _regionID, VID _recentlyGrown) {
  //Setup MP variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  Environment* env = GetMPProblem()->GetEnvironment();
  StatClass* regionStats = region->GetStatClass();
  Roadmap<CfgType, WeightType>* rdmp = region->GetRoadmap();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  shared_ptr<LocalPlannerMethod<CfgType, WeightType> > lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp);
  LPOutput<CfgType, WeightType> lpOutput;

  stringstream clockName; clockName << "Iteration " << m_currentIteration << ", Component Connection";
  regionStats->StartClock(clockName.str());

  stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
  vector< pair<size_t,VID> > ccs;
  get_cc_stats(*(rdmp->m_pRoadmap),cmap, ccs);
  cmap.reset();
  if(ccs.size()==1) return;

  vector<VID> vidCC;
  vidCC.push_back(_recentlyGrown);
  CfgType c1 = rdmp->m_pRoadmap->find_vertex(_recentlyGrown)->property();
  typedef vector<pair<size_t, VID> >::iterator CCSIT;
  vector<VID> closestNodesOtherCCs;

  //find closest VID from other CCS
  for(CCSIT ccsit = ccs.begin(); ccsit!=ccs.end(); ccsit++){
    vector<VID> cc;
    get_cc(*(rdmp->m_pRoadmap),cmap,ccsit->second,cc);
    cmap.reset();
    vector<VID> closest;
    nf->KClosest(nf->GetNFMethod(m_nf), rdmp, cc.begin(), cc.end(), _recentlyGrown, 1, back_inserter(closest));
    closestNodesOtherCCs.push_back(closest[0]);
  }

  //find closest VID from other CCS reps
  vector<VID> closestNode;
  nf->KClosest(nf->GetNFMethod(m_nf), rdmp, closestNodesOtherCCs.begin(), 
      closestNodesOtherCCs.end(), _recentlyGrown, 1, back_inserter(closestNode));

  //attempt connection
  CfgType c2 = rdmp->m_pRoadmap->find_vertex(closestNode[0])->property();
  if(m_debug) cout << "Attempt Connection " << c1 << "\t" << c2 << endl;
  CfgType col;
  if(dm->Distance(env, c1, c2)<m_delta &&
      lp->IsConnected(env, *regionStats, dm, c1, c2, col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes())){

    if(m_debug) cout << "Connected" << endl;
    rdmp->m_pRoadmap->AddEdge(_recentlyGrown, closestNode[0], lpOutput.edge);
  }
}

void
BasicRRTStrategy::EvaluateGoals(int _regionID){
  // Setup MP Variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  StatClass* regionStats = region->GetStatClass();
  Environment* env = region->GetRoadmap()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  LocalPlanners<CfgType,WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  LPOutput<CfgType,WeightType> lpOutput;
  // Check if goals have been found
  for(vector<size_t>::iterator i = m_goalsNotFound.begin(); i!=m_goalsNotFound.end(); i++){
    vector<VID> closests;
    nf->KClosest(nf->GetNFMethod(m_nf), region->GetRoadmap(), m_goals[*i], 1, back_inserter(closests));     
    CfgType closest = region->GetRoadmap()->m_pRoadmap->find_vertex(closests[0])->property();
    double dist = dm->Distance(env, m_goals[*i], closest);
    if(m_debug) cout << "Distance to goal::" << dist << endl;
    CfgType col;
    if(dist < m_delta && lp->GetMethod(m_lp)->IsConnected(env, *regionStats, dm, closest, m_goals[*i], col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true, false, false)){
      if(m_debug) cout << "Goal found::" << m_goals[*i] << endl;
      region->GetRoadmap()->m_pRoadmap->AddVertex(m_goals[*i]);
      region->GetRoadmap()->m_pRoadmap->AddEdge(closest, m_goals[*i], lpOutput.edge);
      m_goalsNotFound.erase(i);
      i--;
    }
  }
}

bool 
BasicRRTStrategy::EvaluateMap(int _regionID) {
  if (m_evaluators.empty()) {
    return true;
  }
  else{
    MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
    StatClass* stats = region->GetStatClass();
    
    bool mapPassedEvaluation = false;
    stringstream clockName; clockName << "Iteration " << m_currentIteration << ", Map Evaluation"; 
    stats->StartClock(clockName.str());
    mapPassedEvaluation = true;

    for (vector<string>::iterator I = m_evaluators.begin(); I != m_evaluators.end(); ++I) {
      MapEvaluator<CfgType, WeightType>::MapEvaluationMethodPtr evaluator;
      evaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      stringstream evaluatorClockName; evaluatorClockName << "Iteration " << m_currentIteration << ", " << evaluator->GetName();
      stats->StartClock(evaluatorClockName.str());
      if(m_debug) cout << "\n\t";
      mapPassedEvaluation = evaluator->operator()(_regionID);
      if(m_debug) cout << "\t";
      stats->StopClock(evaluatorClockName.str());
      if(m_debug) stats->PrintClock(evaluatorClockName.str(), cout);
      if(mapPassedEvaluation){
        if(m_debug) cout << "\t  (passed)\n";
      }
      else{
        if(m_debug) cout << "\t  (failed)\n";
      }
      if(!mapPassedEvaluation)
        break;
    }
    stats->StopClock(clockName.str());
    if(m_debug) stats->PrintClock(clockName.str(), cout);
    return mapPassedEvaluation;
  } 
}

