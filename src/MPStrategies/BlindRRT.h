#ifndef BLIND_RRT_H_
#define BLIND_RRT_H_

#include "MPStrategyMethod.h"

#include "MapEvaluators/Query.h"
#include "Utilities/MPUtils.h"
#include "ParallelMethods/WorkFunctions/RadialUtils.h"

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

    void EvaluateGoals();

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
    // how are the CCs are being connected: Centroid, Node?
    string m_CCconnection;
    // Num of iters blind rrt is ran before engaging in CC connection
    size_t m_initialSamples;
    // num of CC connections attempts
    size_t m_numCCIters;

    RadialUtils<MPTraits> m_radialUtils;
};

template<class MPTraits>
BlindRRT<MPTraits>::BlindRRT(): m_query(NULL){
  this->SetName("BlindRRT");
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
  m_initialSamples = _node.numberXMLParameter("initialSamples", true, 0, 0, MAX_INT, "Initial Sample size");
  m_numCCIters = _node.numberXMLParameter("ccIters", true, 0, 0, MAX_INT, "ccIterations");
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
  // Setup MP variables
  Environment* env = this->GetEnvironment();
  CDInfo cdInfo;
  string callee = "BlindRRT::RRT";
  // Setup RRT Variables
  CfgType tmp;
  if(m_query != NULL){
    vector<CfgType>& queryCfgs = m_query->GetQuery();
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit1 = queryCfgs.begin(), cit2 = cit1+1; cit2!=queryCfgs.end(); cit1++, cit2++){
      if (!this->GetValidityChecker(m_vc)->
          IsValid(*cit1, cdInfo, callee)){
      } else {
      }
      if (!this->GetValidityChecker(m_vc)->
          IsValid(*cit2, cdInfo, callee)){
      } else {
      }

      m_roots.push_back(*cit1);
      m_goals.push_back(*cit2);
      m_goalsNotFound.push_back(m_goals.size()-1);
    }
  }
  else{
    // Add root vertex/vertices
    tmp.GetRandomCfg(env);
    if (env->InBounds(tmp)
        && this->GetValidityChecker(m_vc)->IsValid(tmp, cdInfo, callee)){
      m_roots.push_back(tmp);
      m_goals.push_back(tmp);
      m_goalsNotFound.push_back(1);
    }
  }

  for(typename vector<CfgType>::iterator C = m_roots.begin(); C!=m_roots.end(); C++){
    VID vid = this->GetRoadmap()->GetGraph()->AddVertex(*C);
  }


  m_radialUtils = RadialUtils<MPTraits>(this->GetMPProblem(), NULL, m_dm, m_vc,
      m_nf, m_CCconnection, m_delta, m_minDist, m_numCCIters, this->m_debug);

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
  StatClass* stats = this->GetStatClass();

  stats->StartClock("BlindRRT Generation");

  bool mapPassedEvaluation = false;
  size_t samples = 0;


  vector<VID> branch;
  for(auto i : *this->GetRoadmap()->GetGraph())
    branch.push_back(i.descriptor());

  // branch is used in RadialUtils to track the VIDs
  // it is also used in parallel so it is the best way to adapt
  //RoadmapType* rdmp = this->GetRoadmap();
  //rdmp->GetGraph()->GetVerticesVID(branch);

  while(!mapPassedEvaluation && samples < m_initialSamples){
    CfgType dir = SelectDir();
    //grow towards the direction
    // the interesting part occurs inside this function
    int samplesMade = m_radialUtils.ExpandTree(branch, dir);
    // there can be more than one samples made in one expansion
    samples += samplesMade;

    bool evalMap = this->EvaluateMap(m_evaluators);
    mapPassedEvaluation = evalMap && ((m_evaluateGoal && m_goalsNotFound.size()==0) || !m_evaluateGoal);

    if( m_goalsNotFound.size()==0 && this->m_debug)
      cout << "RRT FOUND ALL GOALS" << endl;
  }

  // Did we exit because we found a goal, or because we met the number of nodes?
  if((m_evaluateGoal && m_goalsNotFound.size() !=0) || !m_evaluateGoal) {
    // Get VALID CCs
    m_radialUtils.RemoveInvalidNodes(branch);
    m_radialUtils.ConnectCCs();
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
  StatClass* stats = this->GetStatClass();
  string str;

#ifndef _PARALLEL
  //perform query if query was given as input
  if(m_query != NULL){
    str = this->GetBaseFilename() + ".path";
    m_query->SetPathFile(str);
    if(m_evaluateGoal){
      if(m_query->PerformQuery(this->GetRoadmap())){
        if(this->m_debug) cout << "Query successful! Output written to " << str << "." << endl;
      }
      else{
        if(this->m_debug) cout << "Query unsuccessful." << endl;
      }
    }
  }
#endif

  //output final map
  str = this->GetBaseFilename() + ".map";
  this->GetRoadmap()->Write(str, this->GetEnvironment());

  //output stats
  str = this->GetBaseFilename() + ".stat";
  ofstream osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  stats->PrintAllStats(osStat, this->GetRoadmap());
  stats->PrintClock("BlindRRT Generation", osStat);
  osStat.close();

  if(this->m_debug) cout<<"\nEnd Finalizing BlindRRT"<<endl;
}

template<class MPTraits>
void
BlindRRT<MPTraits>::EvaluateGoals(){
  // Setup MP Variables
  StatClass* stats = this->GetStatClass();
  Environment* env = this->GetEnvironment();
  DistanceMetricPointer dmp = this->GetDistanceMetric(m_dm);
  LocalPlannerPointer lpp = this->GetLocalPlanner(m_lp);
  NeighborhoodFinderPointer nfp = this->GetNeighborhoodFinder(m_nf);
  LPOutput<MPTraits> lpOutput;
  // Check if goals have been found
  for(vector<size_t>::iterator i = m_goalsNotFound.begin(); i!=m_goalsNotFound.end(); i++){
    vector<VID> closests;
    nfp->KClosest(this->GetRoadmap(), m_goals[*i], 1, back_inserter(closests));
    CfgType& closest = this->GetRoadmap()->GetGraph()->GetCfg(closests[0]);
    double dist = dmp->Distance(env, m_goals[*i], closest);
    if(this->m_debug) cout << "Distance to goal::" << dist << endl;
    CfgType col;
    if(dist < m_delta && lpp->IsConnected(env, *stats, dmp, closest, m_goals[*i], col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true, false, false)){
      if(this->m_debug) cout << "Goal found::" << m_goals[*i] << endl;
      if(!(this->GetRoadmap()->GetGraph()->IsVertex( m_goals[*i])))
        this->GetRoadmap()->GetGraph()->AddVertex(m_goals[*i]);
      this->GetRoadmap()->GetGraph()->AddEdge(closest, m_goals[*i], lpOutput.edge);
      m_goalsNotFound.erase(i);
      i--;
    }
  }
}


template<class MPTraits>
typename MPTraits::CfgType
BlindRRT<MPTraits>::SelectDir(){
  Environment* env = this->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}


#endif
