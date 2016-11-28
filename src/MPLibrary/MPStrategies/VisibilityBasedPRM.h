#ifndef VISIBILITY_BASED_PRM_H_
#define VISIBILITY_BASED_PRM_H_

#include "MPStrategyMethod.h"
#include "Utilities/IOUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Constructs roadmap based upon visibility
///
///  Nodes are sampled one at a time, and will be classified as either guards
///    or connections.
///  If the new node is not visible to any guard, it becomes a guard.  Guards map
///    the visibilty of the environment.  They can be connected by connection
///    nodes to form connected components, but no guard is visible to another
///    guard.
///  If the new node is visible from two or more guard CCs, it becomes a
///    connection.  It will be connected to one visible guard node in each CC.
///    Those CCs will then be merged into a single CC through the connection
///    node.
///  If the new node is visible from only one guard set, it is discarded.
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class VisibilityBasedPRM : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    VisibilityBasedPRM(const string _sampler = "",
        const string _vc = "", const string _lp = "",
        const size_t _maxFailedIterations = 10);
    VisibilityBasedPRM(XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize() {}
    virtual void Iterate();
    virtual bool EvaluateMap();
    virtual void Finalize();

  protected:

    //Input parameters
    string m_samplerLabel;
    string m_vcLabel;
    string m_lpLabel;
    size_t m_maxFailedIterations; ///< Number of attempts allowed per guard.
    size_t m_failedIterations;    ///< Tracks consecutive failed attempts.

    //Local data
    vector<vector<CfgType> > m_guards;

    //Auxiliary Functions
    void GenerateNode(vector<CfgType>& _outNode);
    bool ConnectVisibleGuardSets(vector<CfgType>& _outNode);
};


template <typename MPTraits>
VisibilityBasedPRM<MPTraits>::
VisibilityBasedPRM(const string _sampler, const string _vc, const string _lp,
    const size_t _maxFailedIterations) : m_samplerLabel(_sampler),
    m_vcLabel(_vc), m_lpLabel(_lp), m_maxFailedIterations(_maxFailedIterations),
    m_failedIterations(0) {
  this->SetName("VisibilityBasedPRM");
}


template <typename MPTraits>
VisibilityBasedPRM<MPTraits>::
VisibilityBasedPRM(XMLNode& _node) :
    MPStrategyMethod<MPTraits>(_node), m_failedIterations(0) {
  this->SetName("VisibilityBasedPRM");
  ParseXML(_node);
}


template <typename MPTraits>
void
VisibilityBasedPRM<MPTraits>::
ParseXML(XMLNode& _node) {
  m_samplerLabel = _node.Read("sampler", true, "", "Node Sampler Method");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Checker");
  m_lpLabel = _node.Read("lpLabel", true, "sl", "Local Planner");
  m_maxFailedIterations = _node.Read("maxFailedIterations", true, 10, 1,
      MAX_INT, "Maximum consecutive failed iterations");
}


template <typename MPTraits>
void
VisibilityBasedPRM<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\n\tSampler: " << m_samplerLabel
      << "\n\tValidity Checker: " << m_vcLabel
      << "\n\tLocal Planner: " << m_lpLabel
      << "\n\tMaximum consecutive failed iterations: " << m_maxFailedIterations
      << endl;
}


template <typename MPTraits>
void
VisibilityBasedPRM<MPTraits>::
Iterate() {
  if(this->m_debug)
    cout << "\nCreating node, currently "
         << this->GetRoadmap()->GetGraph()->get_num_vertices()
         << " nodes and " << m_guards.size() << " guard sets.";

  //Sample one node
  vector<CfgType> tempNode;
  GenerateNode(tempNode);

  //Iterate over guard subsets and try to connect tempNode to one node in each
  //subset
  bool guardCreated = ConnectVisibleGuardSets(tempNode);

  //Update m_failedIterations
  if(guardCreated) m_failedIterations = 0;
  else m_failedIterations++;

  if(this->m_debug)
    cout << "\n\tFailed Iterations = " << m_failedIterations;
}


template <typename MPTraits>
bool
VisibilityBasedPRM<MPTraits>::
EvaluateMap() {
  return m_failedIterations < m_maxFailedIterations;
}


template <typename MPTraits>
void
VisibilityBasedPRM<MPTraits>::
Finalize() {
  if(this->m_debug)
    cout << "\nFinalizing VisibilityBasedPRM::\n";

  //Output .map file
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map",
      this->GetEnvironment());

  //Output .stat file
  StatClass* stats = this->GetStatClass();
  string fileName = this->GetBaseFilename() + ".stat";
  ofstream osStat(fileName.c_str());
  osStat << "Visibility-Based PRM Stats\n";
  stats->PrintAllStats(osStat, this->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  osStat << "\nNumber of connected guard components: " << m_guards.size();
  osStat.close();

  if(this->m_debug)
    cout << "\nFinished finalizing VisibilityBasedPRM.\n";
}


template <typename MPTraits>
void
VisibilityBasedPRM<MPTraits>::
GenerateNode(vector<CfgType>& _outNode) {
  auto sampler = this->GetSampler(m_samplerLabel);
  auto vc = this->GetValidityChecker(m_vcLabel);

  string callee("VisibilityBasedPRM::GenerateNodes");

  do {
    //Sample one node
    do {
      sampler->Sample(1, 1, this->GetEnvironment()->GetBoundary(),
          back_inserter(_outNode));
    } while (_outNode.size() <= 0);

    //Check validity
    if(!_outNode[0].IsLabel("VALID")) {
      vc->IsValid(_outNode[0], callee);
    }

    //Discard and resample if invalid
  } while (!_outNode[0].GetLabel("VALID"));

  if(this->m_debug) cout << "\nNode created, searching for connections:";
}


template <typename MPTraits>
bool
VisibilityBasedPRM<MPTraits>::
ConnectVisibleGuardSets(vector<CfgType>& _outNode) {
  auto lp = this->GetLocalPlanner(m_lpLabel);
  GraphType* g = this->GetRoadmap()->GetGraph();
  Environment* env = this->GetEnvironment();

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

      if(this->m_debug)
        cout << "\n\tAttempting connection to node " << g->GetVID(*cit);

      //Attempt connection to current guard node *cit
      if(lp->IsConnected(_outNode[0], *cit, col, &lpOutput,
            env->GetPositionRes(), env->GetOrientationRes(), true)) {
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
      cout << "\n\tNode is connector joining " << visibleGuardSets.size()
           << " guard sets.";
  }

  //_outNode[0] is a guard iff visibleGuardSets.size() == 0
  //It it is, add the node to Roadmap and create new guard subset
  if(visibleGuardSets.size() == 0) {
    g->AddVertex(_outNode[0]);
    m_guards.push_back(_outNode);
    if(this->m_debug)
      cout << "\n\tNode is a guard, adding a new guard set.";
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
