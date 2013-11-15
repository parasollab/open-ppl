/*
 *  This method constructs a roadmap based on visibility:
 *
 *  Nodes are sampled one at a time, and will be classified as either guards
 *    or connections.
 *  If the new node is not visible to any guard, it becomes a guard.  Guards map
 *    the visibilty of the environment.  They can be connected by connection
 *    nodes to form connected components, but no guard is visible to another
 *    guard.
 *  If the new node is visible from two or more guard CCs, it becomes a
 *    connection.  It will be connected to one visible guard node in each CC.
 *    Those CCs will then be merged into a single CC through the connection
 *    node.
 *  If the new node is visible from only one guard set, it is discarded.
 *
 */
#ifndef VISIBILITYBASEDPRM_H_
#define VISIBILITYBASEDPRM_H_

#include "MPStrategyMethod.h"
#include "Utilities/IOUtils.h"

template<class MPTraits>
class VisibilityBasedPRM : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    VisibilityBasedPRM(const string _sampler = "",
        const string _vc = "", const string _lp = "", const string _dm = "",
        const int _maxFailedIterations = 10);
    VisibilityBasedPRM(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  protected:
    //Input parameters
    string m_samplerLabel;
    string m_vcLabel;
    string m_lpLabel;
    string m_dmLabel;
    int m_maxFailedIterations;

    //Local data
    vector<vector<CfgType> > m_guards;

    //Auxiliary Functions
    void GenerateNode(vector<CfgType>& _outNode);
    bool ConnectVisibleGuardSets(vector<CfgType>& _outNode);
};


template<class MPTraits>
VisibilityBasedPRM<MPTraits>::VisibilityBasedPRM(
    const string _sampler,
    const string _vc, const string _lp, const string _dm,
    const int _maxFailedIterations) :
    m_samplerLabel(_sampler),
    m_vcLabel(_vc), m_lpLabel(_lp), m_dmLabel(_dm),
    m_maxFailedIterations(_maxFailedIterations) {
  this->SetName("VisibilityBasedPRM");
}


template<class MPTraits>
VisibilityBasedPRM<MPTraits>::VisibilityBasedPRM(
    MPProblemType* _problem, XMLNodeReader& _node) :
    MPStrategyMethod<MPTraits>(_problem, _node) {
  this->SetName("VisibilityBasedPRM");
  ParseXML(_node);
}


template<class MPTraits>
void
VisibilityBasedPRM<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_samplerLabel = _node.stringXMLParameter("sampler", true, "",
      "Node Sampler Method");
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Checker");
  m_lpLabel = _node.stringXMLParameter("lpLabel", true, "sl", "Local Planner");
  m_dmLabel = _node.stringXMLParameter("dmLabel", true, "", "Distance Metric");
  m_maxFailedIterations = _node.numberXMLParameter("maxFailedIterations", true, 10, 1,
      MAX_INT, "Maximum consecutive failed iterations");
}


template<class MPTraits>
void
VisibilityBasedPRM<MPTraits>::PrintOptions(ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\n\tSampler: " << m_samplerLabel
      << "\n\tValidity Checker: " << m_vcLabel
      << "\n\tLocal Planner: " << m_lpLabel
      << "\n\tDistance Metric: " << m_dmLabel
      << "\n\tMaximum consecutive failed iterations: " << m_maxFailedIterations
      << endl;
}


template<class MPTraits>
void
VisibilityBasedPRM<MPTraits>::Initialize() { }


template<class MPTraits>
void
VisibilityBasedPRM<MPTraits>::Run() {
  //Book keeping
  if(this->m_debug) cout << "\nRunning VisibilityBasedPRM::\n";
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  if(this->m_recordKeep) stats->StartClock("Map Generation");

  int failedIterations = 0; //tracks consecutive failed attempts to create a guard

  //Create map
  while(failedIterations < m_maxFailedIterations) {

    if(this->m_debug)
      cout << "\nCreating node, currently "
           << this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices()
           << " nodes and " << m_guards.size() << " guard sets.";

    //Sample one node
    vector<CfgType> tempNode;
    GenerateNode(tempNode);

    //Iterate over guard subsets: try to connect tempNode to one node in each subset
    bool guardCreated = ConnectVisibleGuardSets(tempNode);

    //Update failedIterations
    if(guardCreated) failedIterations = 0;
    else failedIterations++;

    if(this->m_debug) cout << "\n\tfailedIterations = " << failedIterations;
  }

  //Book keeping
  if(this->m_recordKeep) {
    stats->StopClock("Map Generation");
    if(this->m_debug) {cout << endl; stats->PrintClock("Map Generation", cout);}
  }

  if(this->m_debug) cout << "\nFinished running VisibilityBasedPRM.\n";
}


template<class MPTraits>
void
VisibilityBasedPRM<MPTraits>::Finalize() {
  if(this->m_debug) cout << "\nFinalizing VisibilityBasedPRM::\n";

  //Setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string fileName;

  //Output .map file
  fileName = this->GetBaseFilename() + ".map";
  ofstream osMap(fileName.c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();

  //Output .stat file
  fileName = this->GetBaseFilename() + ".stat";
  ofstream osStat(fileName.c_str());
  osStat << "Visibility-Based PRM Stats\n";
  stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  osStat << "\nNumber of connected guard components: " << m_guards.size();
  osStat.close();

  if(this->m_debug) cout << "\nFinished finalizing VisibilityBasedPRM.\n";
}


template<class MPTraits>
void
VisibilityBasedPRM<MPTraits>::GenerateNode(vector<CfgType>& _outNode) {

  SamplerPointer sampler = this->GetMPProblem()->GetSampler(m_samplerLabel);
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  string callee("VisibilityBasedPRM::GenerateNodes");

  do {
    //Sample one node
    do {
      sampler->Sample(env, *stats, 1, 1, back_inserter(_outNode));
    } while (_outNode.size() <= 0);

    //Check validity
    if(!_outNode[0].IsLabel("VALID")) {
      vc->IsValid(_outNode[0], callee);
    }

    //Discard and resample if invalid
  } while (!_outNode[0].GetLabel("VALID"));

  if(this->m_debug) cout << "\nNode created, searching for connections:";
}


template<class MPTraits>
bool
VisibilityBasedPRM<MPTraits>::ConnectVisibleGuardSets(vector<CfgType>& _outNode) {

  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  typedef typename vector<CfgType>::iterator CIT;
  typedef typename vector<vector<CfgType> >::iterator GIT; //Guard subset iterator
  typedef pair<WeightType, WeightType> LPEdge;

  LPOutput<MPTraits> lpOutput;
  CfgType col;
  vector< pair<VID, LPEdge> > validEdges;
  vector< GIT > visibleGuardSets;

  if(this->m_debug) VDAddNode(_outNode[0]);

  //find all guard subsets that are visible to _outNode
  for(GIT git = m_guards.begin(); git != m_guards.end(); git++) {
    for(CIT cit = git->begin(); cit != git->end(); cit++) {

      if(this->m_debug) cout << "\n\tAttempting connection to node " << g->GetVID(*cit);

      //Attempt connection to current guard node *cit
      if(lp->IsConnected(env, *stats, dm, _outNode[0], *cit, col, &lpOutput,
            env->GetPositionRes(), env->GetOrientationRes(), true)) {
        vector<CfgType>* validSubset = &(*git);
        CfgType* validNode = &(*cit);
        validEdges.push_back(make_pair(g->GetVID(*validNode), lpOutput.m_edge));
        visibleGuardSets.push_back(git);
        if(this->m_debug) {
          cout << "\n\tConnection found.";
          VDAddEdge(_outNode[0], *cit);
        }
        break;
      }
      //If connection fails and debug is on, add failed path to vizmo debug
      else if(this->m_debug) {
        VDAddEdge(_outNode[0], *cit);
        if(!g->IsVertex(col)) {
          VDAddNode(col);
          VDRemoveNode(col);
        }
        VDRemoveEdge(_outNode[0], *cit);
      }
    }
  }

  //_outNode is a connection if visibleGuardSets.size() > 1.
  if(visibleGuardSets.size() > 1) {
    //Add _outNode to the roadmap
    VID newNode = g->AddVertex(_outNode[0]);

    //Add edges to graph
    for(typename vector<pair<VID, LPEdge> >::iterator eit = validEdges.begin();
        eit != validEdges.end(); eit++){
      g->AddEdge(newNode, eit->first, eit->second);
    }

    //Merge guard subsets that are connected by _outNode
    vector<CfgType>* firstSet = &(**(visibleGuardSets.begin()));
    for(typename vector< GIT >::iterator vit = visibleGuardSets.begin() + 1;
        vit != visibleGuardSets.end(); vit++) {
      copy((*vit)->begin(), (*vit)->end(), back_inserter(*firstSet));
      m_guards.erase(*vit);
    }

    if(this->m_debug)
      cout << "\n\tNode is connector joining " << visibleGuardSets.size() << " guard sets.";
  }

  //_outNode[0] is a guard iff visibleGuardSets.size() == 0
  //It it is, add the node to Roadmap and create new guard subset
  if(visibleGuardSets.size() == 0) {
    g->AddVertex(_outNode[0]);
    m_guards.push_back(_outNode);
    if(this->m_debug) cout << "\n\tNode is a guard, adding a new guard set.";
    return true;
  }

  //If _outNode is neither a guard nor a connection, it is discarded
  if(this->m_debug && visibleGuardSets.size() == 1) {
    cout << "\n\tOnly one connection, node discarded.";
    VDRemoveEdge(_outNode[0], g->GetVertex(validEdges[0].first));
    VDRemoveNode(_outNode[0]);
  }

  return false;
}


#endif
