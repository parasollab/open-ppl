#ifndef DISASSEMBLY_SEQUENTIAL_GRAPH_H_
#define DISASSEMBLY_SEQUENTIAL_GRAPH_H_

#include <chrono>
#include <queue>

#include "DisassemblyMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Basic serial disassembly method
///
///
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyExhaustiveGraph : public DisassemblyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::GraphType      GraphType;
    typedef typename MPTraits::WeightType        WeightType;
    typedef typename RoadmapType::VID            VID;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode DisassemblyNode;
    typedef vector<unsigned int>                 Subassembly;
    typedef typename DisassemblyMethod<MPTraits>::Approach Approach;
    typedef typename DisassemblyMethod<MPTraits>::State State;
    typedef pair<Subassembly, map<Approach, bool> > AttemptEntry;

    DisassemblyExhaustiveGraph(
        const map<string, pair<size_t, size_t> >& _matingSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const map<string, pair<size_t, size_t> >& _rrtSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const string _vc = "", const string _singleVc = "",
        const string _lp = "", const string _ex = "",
        const string _dm = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    DisassemblyExhaustiveGraph(XMLNode& _node);
    virtual ~DisassemblyExhaustiveGraph() {}

    virtual void Iterate();

  protected:
    virtual DisassemblyNode* SelectExpansionNode() override;
    virtual Subassembly SelectSubassembly(DisassemblyNode* _q) override;
    virtual std::pair<bool, vector<CfgType>> Expand(DisassemblyNode* _q,
                                      const Subassembly& _subassembly) override;

    std::pair<bool, vector<CfgType>> UnWeightedExpand(DisassemblyNode* _q,
                                              const Subassembly& _subassembly);
    std::pair<bool, vector<CfgType>> WeightedExpand(DisassemblyNode* _q,
                                          const Subassembly& _subassembly);

    void RunUnitTests();

    //Bucketing interface for faster node matching within the tree:
    void InitializeBuckets();
    void AddToBuckets(DisassemblyNode* _n);
    DisassemblyNode* GetNodeFromBuckets(DisassemblyNode* _n);

    //Since we do vector equality, we need to ensure all vectors are sorted
    // in the same way.
    void EnforcePartSorting(DisassemblyNode* const _node);

    void HandleNewNode(DisassemblyNode* const _node);

    //Merges _node into _existingNode, handling the roadmap edges if needed.
    void MergeNodes(DisassemblyNode* const _node,
                    DisassemblyNode* const _existingNode);

    void RecursiveUpdateChildrenWeights(DisassemblyNode* const _n,
                                        DisassemblyNode* const _newParent,
                                        const double _newCumulativeWeight);

    class compNodes {
      public:
        bool operator()(DisassemblyNode* a, DisassemblyNode* b) const {
          return (a->bestCumulativeWeight < b->bestCumulativeWeight);
        }
    };

    //We instead will use a priority queue, sorted based on node->weight.
    // This will still use a vector as its underlying structure, and will be
    // sorted in ascending weights.
    std::priority_queue<DisassemblyNode*, std::vector<DisassemblyNode*>,
                        compNodes> m_nodeQueueAStar;

    std::list<DisassemblyNode*> m_nodeQueueBFS;

    std::vector<std::vector<DisassemblyNode*> > m_nodeBuckets;

    bool m_useRRT{true};

    const bool m_aStarSearch{false};//TODO: add in parameter once implemented

    using DisassemblyMethod<MPTraits>::m_disNodes;
};

template <typename MPTraits>
DisassemblyExhaustiveGraph<MPTraits>::
DisassemblyExhaustiveGraph(
    const map<string, pair<size_t, size_t> >& _matingSamplerLabels,
    const map<string, pair<size_t, size_t> >& _rrtSamplerLabels,
    const string _vc, const string _singleVc, const string _lp,
    const string _ex, const string _dm, const vector<string>& _evaluatorLabels):
    DisassemblyMethod<MPTraits>(_matingSamplerLabels, _rrtSamplerLabels, _vc,
      _singleVc, _lp, _ex, _dm, _evaluatorLabels) {
  this->SetName("DisassemblyExhaustiveGraph");
  this->m_graphMethod = true;
}

template <typename MPTraits>
DisassemblyExhaustiveGraph<MPTraits>::
DisassemblyExhaustiveGraph(XMLNode& _node) : DisassemblyMethod<MPTraits>(_node){
  this->SetName("DisassemblyExhaustiveGraph");
  this->m_graphMethod = true;

  m_useRRT = _node.Read("useRRT", false, m_useRRT, "Flag to turn off RRT");
}

template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
InitializeBuckets() {
  RunUnitTests();

  //For now, just bucket based on number of remaining parts plus one, since we
  // want buckets for 0 parts AND for all remaining parts.
  m_nodeBuckets.resize(this->m_rootNode->NumberOfRemainingParts() + 1,
                       std::vector<DisassemblyNode*>());
}

template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
RunUnitTests() {
  //Just resize to 30 buckets, which should be more than enough:
  m_nodeBuckets.resize(30, std::vector<DisassemblyNode*>());


  //Unit Test Set 1: Bucketing tests. These DO NOT check sorting, so all inputs
  // should be sorted already. This is to confirm entry and retrieval/matching
  // via buckets is functioning properly.
  DisassemblyNode n1, n2, n3, n4, n5;
  n1.initialParts = {1,3,5,6};

  n2.usedSubassemblies = {{2,3,4,5},{1,8,9},{6}};

  n3.initialParts = {1,2,3,4,5,6,7,8,9};
  n3.usedSubassemblies = {{10,11},{12}};

  n4.initialParts = {1,2,3,4,5,6,7,8,9};
  n4.usedSubassemblies = {{10,12},{11}};
  //n5 will be an empty one, to emulate a fully disassembled node in the queue.

  AddToBuckets(&n1);
  AddToBuckets(&n2);
  AddToBuckets(&n3);
  AddToBuckets(&n4);
  AddToBuckets(&n5);

  auto n5ptr = GetNodeFromBuckets(&n5);
  auto n3ptr = GetNodeFromBuckets(&n3);
  auto n1ptr = GetNodeFromBuckets(&n1);
  auto n2ptr = GetNodeFromBuckets(&n2);
  auto n4ptr = GetNodeFromBuckets(&n4);

  if(n1ptr != &n1 ||
     n2ptr != &n2 ||
     n3ptr != &n3 ||
     n4ptr != &n4 ||
     n5ptr != &n5)
    throw RunTimeException(WHERE, "First retrieval from buckets test failed!");

  DisassemblyNode p;
  p.initialParts = {1,2,3,4,5,6,7,8,9};
  p.usedSubassemblies = {{10,11},{12}};
  n3ptr = GetNodeFromBuckets(&p);
  if(n3ptr != &n3 ||
     n3ptr == &p)
    throw RunTimeException(WHERE, "Second retrieval from buckets test failed!");


  //Unit Test Set 2: Sorting tests. These only check that the sorting is
  // functioning, with no bother of bucketing.
  DisassemblyNode sortMe;
  sortMe.initialParts = {5,6,1,4,3};
  sortMe.usedSubassemblies = {{7,8},{12,11,9,10},{2},{21},{20,19,0,18,17}};
  EnforcePartSorting(&sortMe);

  if(sortMe.initialParts != Subassembly({1,3,4,5,6}))
    throw RunTimeException(WHERE, "Initial Parts sorting test failed!!!");

  std::vector<Subassembly> sortedSubs =
                                  {{0,17,18,19,20},{2},{7,8},{9,10,11,12},{21}};
  if(sortMe.usedSubassemblies != sortedSubs)
    throw RunTimeException(WHERE, "Subassembly sorting test failed!!!");

  m_nodeBuckets.clear();//Clean up
}

template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
EnforcePartSorting(DisassemblyNode* const _node) {
  //Sort all initial parts in ascending order:
  std::sort(_node->initialParts.begin(), _node->initialParts.end());

  //Sort each subassembly:
  for(auto& sub : _node->usedSubassemblies)
    std::sort(sub.begin(), sub.end());

  //Sort subassemblies by increasing number of first part:
  struct SortByFirstPart {
    bool operator() (const Subassembly& i, const Subassembly& j) {
      return (i.at(0) < j.at(0));
    }
  } sortFirstPart;
  sort(_node->usedSubassemblies.begin(), _node->usedSubassemblies.end(),
       sortFirstPart);
}

template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
AddToBuckets(DisassemblyNode* _n) {
  //This doesn't check if the node is already in a bucket
  m_nodeBuckets[_n->NumberOfRemainingParts()].push_back(_n);
}


template <typename MPTraits>
typename DisassemblyMethod<MPTraits>::DisassemblyNode*
DisassemblyExhaustiveGraph<MPTraits>::
GetNodeFromBuckets(DisassemblyNode* _n) {
  //NOTE: This relies on parts and subassemblies being sorted!
  //There needs to be an ordering of parts in a vector, and an order of
  // subassemblies for a node (just sort based on first part's value, as long
  // as that subassembly is sorted already).
  if(!_n)
    return nullptr;

  const size_t bucketNum = _n->NumberOfRemainingParts();
  for(size_t i = 0; i < m_nodeBuckets[bucketNum].size(); ++i) {
    DisassemblyNode* node = m_nodeBuckets[bucketNum][i];
    if(node->initialParts == _n->initialParts &&
       node->usedSubassemblies == _n->usedSubassemblies)
      return node;
  }
  return nullptr;
}

template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
Iterate() {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Iterate()" << endl;

  //Only one or the other should be used, so one queue should always be empty:
  if(!m_disNodes.empty() && m_nodeQueueAStar.empty() && m_nodeQueueBFS.empty()){
    this->m_successful = true;
    cout << endl << "Successful disassembling!" << endl << endl;
    return;
  }

  DisassemblyNode* node = SelectExpansionNode();
  if(!node) {
    cout << "Error: returned nullptr node" << endl;
    return;
  }
  else if(node->GetCompletePartList().empty()) {
    cout << "Error: select Node with empty parts" << endl;
    return;
  }

  Expand(node, Subassembly());
}

template <typename MPTraits>
typename DisassemblyExhaustiveGraph<MPTraits>::DisassemblyNode*
DisassemblyExhaustiveGraph<MPTraits>::
SelectExpansionNode() {
  const size_t candSize = m_aStarSearch ? m_nodeQueueAStar.size() :
                                          m_nodeQueueBFS.size();
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::SelectExpansionCfg() | "
         << "Size of candidates = " << candSize << endl;

  // check if first iteration
  if (m_disNodes.empty()) {
    vector<unsigned int> robotParts;
    for (unsigned int i = 0; i < this->m_numParts; ++i)
      robotParts.push_back(i);

    DisassemblyNode node;
    node.vid = this->m_rootVid;
    node.initialParts = robotParts;
    m_disNodes.push_back(node);
    this->m_rootNode = &m_disNodes.back();

    //Clear out both queues, no matter if A* or not:
    m_nodeQueueAStar = std::priority_queue<
        DisassemblyNode*, std::vector<DisassemblyNode*>, compNodes>();
    m_nodeQueueBFS.clear();
    InitializeBuckets();

    EnforcePartSorting(this->m_rootNode);//Shouldn't be strictly necessary.

    return this->m_rootNode;
  }

  //One will always be empty, so if both are, we are out of nodes.
  if(m_nodeQueueAStar.empty() && m_nodeQueueBFS.empty())
    return nullptr;

  // return the first node of the queue and remove it at the same time
  DisassemblyNode* node = nullptr;
  if(m_aStarSearch) {
    node = m_nodeQueueAStar.top();
    m_nodeQueueAStar.pop();
  }
  else {
    node = m_nodeQueueBFS.front();
    m_nodeQueueBFS.pop_front();
  }
  return node;
}


template <typename MPTraits>
vector<unsigned int>
DisassemblyExhaustiveGraph<MPTraits>::
SelectSubassembly(DisassemblyNode* _q) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::SelectSubassembly()" << endl;
  throw RunTimeException(WHERE, "Not used by this method");
  return Subassembly();
}

template <typename MPTraits>
std::pair<bool, std::vector<typename DisassemblyExhaustiveGraph<MPTraits>::CfgType>>
DisassemblyExhaustiveGraph<MPTraits>::
Expand(DisassemblyNode* _node, const Subassembly& _subassembly) {
  if(m_aStarSearch)
    return WeightedExpand(_node, _subassembly);
  else
    return UnWeightedExpand(_node, _subassembly);
}

template <typename MPTraits>
std::pair<bool, std::vector<typename DisassemblyExhaustiveGraph<MPTraits>::CfgType>>
DisassemblyExhaustiveGraph<MPTraits>::
UnWeightedExpand(DisassemblyNode* _node, const Subassembly& _subassembly) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Expand with single-part subassemblies"
         << endl;

  VID newVID;
  std::vector<std::vector<CfgType> > removingPaths;
  Subassembly singleSub;
  DisassemblyNode* newNode = nullptr;

  // first test all single bodies for expansion with mating and rrt approach
  // generate list with all parts (at initial pose and usedSubassemblies)
  auto parts = _node->initialParts;
  for (auto &usedSub : _node->usedSubassemblies)
    for (auto &part : usedSub)
      parts.push_back(part);

  //1. Attempt all single parts: mating, then RRT if mating fails.
  for (auto &part : parts) {
    Subassembly sub = {part};
    singleSub = sub;
    newNode = nullptr;
    // expand
    auto path = this->ExpandMatingApproach(_node->vid, singleSub, newVID);
    if (!path.empty()) {
      removingPaths = {path};
      newNode = this->GenerateNode(_node, singleSub, removingPaths, false);
    }
    else if(m_useRRT) {
      path = this->ExpandRRTApproach(_node->vid, singleSub, newVID);
      if (!path.empty()) {
        removingPaths = {path};
        newNode = this->GenerateNode(_node, singleSub, removingPaths, false);
      }
    }
    // add node to queue, if not completed
    if (newNode && !newNode->GetCompletePartList().empty())
      HandleNewNode(newNode);
  }

  if (this->m_debug)
    cout << this->GetNameAndLabel() << "::Expand with multi-part subassemblies"
         << endl;

  //2. Generate and attempt subassemblies based on the remaining parts in their
  // initial positions (not previously grouped and removed as a subassembly).
  auto subassemblies =
                  this->GenerateSubassemblies(_node->vid, _node->initialParts);
  for (auto &multiSub : subassemblies) {
    // no filter for root position, because initialParts are by definition at
    // initial position
    newNode = nullptr;
    // expand
    auto path = this->ExpandMatingApproach(_node->vid, multiSub, newVID);
    if (!path.empty()) {
      removingPaths = {path};
      newNode = this->GenerateNode(_node, multiSub, removingPaths, true);
    }
    else if(m_useRRT) {
      path = this->ExpandRRTApproach(_node->vid, multiSub, newVID);
      if (!path.empty()) {
        removingPaths = {path};
        newNode = this->GenerateNode(_node, multiSub, removingPaths, true);
      }
    }
    if (newNode)
      HandleNewNode(newNode);
  }

  //3. All subassemblies that have previously been removed also need to be
  // attempted. We have already tried all single parts in step 1, so we only
  // need to attempt "sub-subassemblies" of each subassembly previously removed.
  for (auto &usedSubassembly : _node->usedSubassemblies) {
    subassemblies = this->GenerateSubassemblies(_node->vid, usedSubassembly);
    for (auto &multiSub : subassemblies) {
      if (multiSub.size() == usedSubassembly.size()) // skip complete subs
        continue;

      newNode = nullptr;
      // expand
      auto path = this->ExpandMatingApproach(_node->vid, multiSub, newVID);
      if (!path.empty()) {
        removingPaths = {path};
        newNode = this->GenerateNode(_node, multiSub, removingPaths, true);
      }
      else if(m_useRRT) {
        path = this->ExpandRRTApproach(_node->vid, multiSub, newVID);
        if (!path.empty()) {
          removingPaths = {path};
          newNode = this->GenerateNode(_node, multiSub, removingPaths, true);
        }
      }
      if (newNode)
        HandleNewNode(newNode);
    }
  }

  return std::make_pair(true, std::vector<CfgType>());
}


template <typename MPTraits>
std::pair<bool, std::vector<typename DisassemblyExhaustiveGraph<MPTraits>::CfgType>>
DisassemblyExhaustiveGraph<MPTraits>::
WeightedExpand(DisassemblyNode* _node, const Subassembly& _subassembly) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Expand with single-part subassemblies"
         << endl;

  throw RunTimeException(WHERE, "While this function is implemented, A* search"
                                " for disassembly graphs is not.");

  VID newVID;
  vector<vector<CfgType>> removingPaths;
  Subassembly singleSub;
  DisassemblyNode* newNode = nullptr;

  // first test all single bodies for expansion with mating and rrt approach
  // generate list with all parts (at initial pose and usedSubassemblies)
  auto parts = _node->initialParts;
  for (auto &usedSub : _node->usedSubassemblies)
    for (auto &part : usedSub)
      parts.push_back(part);


  //1. Attempt all single parts: mating, then RRT if mating fails.
  for (auto &part : parts) {
    Subassembly sub = {part};
    singleSub = sub;
    newNode = nullptr;
    // expand
    ClockClass timer;//Time is the heuristic/weight for this strategy.
    timer.StartClock();
    auto path = this->ExpandMatingApproach(_node->vid, singleSub, newVID);
    timer.StopClock();
    if (!path.empty()) {
      const double time = timer.GetSeconds();
      removingPaths = {path};
      newNode = this->GenerateNode(_node, singleSub, removingPaths, false, time);
    }
    else if(m_useRRT) {
      ClockClass timerRRT;
      timerRRT.StartClock();
      path = this->ExpandRRTApproach(_node->vid, singleSub, newVID);
      timerRRT.StopClock();
      if (!path.empty()) {
        const double time = timerRRT.GetSeconds();
        removingPaths = {path};
        newNode = this->GenerateNode(_node, singleSub, removingPaths, false, time);
      }
    }
    // add node to queue, if not completed
    if (newNode && !newNode->GetCompletePartList().empty())
      HandleNewNode(newNode);
  }

  if (this->m_debug)
    cout << this->GetNameAndLabel() << "::Expand with multi-part subassemblies"
         << endl;

  //2. Generate and attempt subassemblies based on the remaining parts in their
  // initial positions (not previously grouped and removed as a subassembly).
  auto subassemblies =
                  this->GenerateSubassemblies(_node->vid, _node->initialParts);
  for (auto &multiSub : subassemblies) {
    // no filter for root position, because initialParts are by definition at
    // initial position
    newNode = nullptr;
    // expand
    ClockClass timer;
    timer.StartClock();
    auto path = this->ExpandMatingApproach(_node->vid, multiSub, newVID);
    timer.StopClock();
    if (!path.empty()) {
      const double time = timer.GetSeconds();
      removingPaths = {path};
      newNode = this->GenerateNode(_node, multiSub, removingPaths, true, time);
    }
    else if(m_useRRT) {
      ClockClass timerRRT;
      timerRRT.StartClock();
      path = this->ExpandRRTApproach(_node->vid, multiSub, newVID);
      timerRRT.StopClock();
      if (!path.empty()) {
        const double time = timerRRT.GetSeconds();
        removingPaths = {path};
        newNode = this->GenerateNode(_node, multiSub, removingPaths, true, time);
      }
    }
    if (newNode)
      HandleNewNode(newNode);
  }

  //3. All subassemblies that have previously been removed also need to be
  // attempted. We have already tried all single parts in step 1, so we only
  // need to attempt "sub-subassemblies" of each subassembly previously removed.
  for (auto &usedSubassembly : _node->usedSubassemblies) {
    subassemblies = this->GenerateSubassemblies(_node->vid, usedSubassembly);
    for (auto &multiSub : subassemblies) {
      if (multiSub.size() == usedSubassembly.size()) // skip complete subs
        continue;

      newNode = nullptr;
      // expand
      ClockClass timer;
      timer.StartClock();
      auto path = this->ExpandMatingApproach(_node->vid, multiSub, newVID);
      timer.StopClock();
      if (!path.empty()) {
        const double time = timer.GetSeconds();
        removingPaths = {path};
        newNode = this->GenerateNode(_node, multiSub, removingPaths, true, time);
      }
      else if(m_useRRT) {
        ClockClass timerRRT;
        timerRRT.StartClock();
        path = this->ExpandRRTApproach(_node->vid, multiSub, newVID);
        timerRRT.StopClock();
        if (!path.empty()) {
          const double time = timerRRT.GetSeconds();
          removingPaths = {path};
          newNode = this->GenerateNode(_node, multiSub, removingPaths, true, time);
        }
      }
      if (newNode)
        HandleNewNode(newNode);
    }
  }

  return make_pair(true, vector<CfgType>());
}

template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
HandleNewNode(DisassemblyNode* const _node) {
  //Make sure the new node is following the same sorting rules:
  EnforcePartSorting(_node);

  //Check if we already have this node bucketed. If not, we'll have a nullptr.
  DisassemblyNode* const existingNode = GetNodeFromBuckets(_node);
  //Keep in mind, even if _node is matched in the buckets, _node is not the same
  // node in memory (identical only in general state of the assembly).

  if(existingNode) {
    MergeNodes(_node, existingNode);

    //Remove from tree (since it's added in GenerateNode; must pop_back
    // of this->m_disNodes)
    //IMPORTANT: this will destroy _node!
    this->m_disNodes.pop_back();
  }
  else {
    //New node, add pointer to the queue and buckets for later expansion.
    if(m_aStarSearch)
      m_nodeQueueAStar.push(_node);
    else
      m_nodeQueueBFS.push_back(_node);

    AddToBuckets(_node);
  }
}

template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
MergeNodes(DisassemblyNode* const _node, DisassemblyNode* const _existingNode) {
  //This takes in _node that has been matched as the same state as _existingNode
  // and updates pointers in _existingNode and _node's parent, and deletes _node

  if(_node != &this->m_disNodes.back())
    throw RunTimeException(WHERE, "Something went wrong: a duplicate node "
                           "was added, but its tree entry can't be found!");
  if(_node->parents.size() > 1 || _node->children.size() > 0)
    throw RunTimeException(WHERE, "Duplicate node's parent's or children "
                           "have more elements than should be possible!!!");

  DisassemblyNode* const newParent = _node->parents.at(0);
  if(newParent->children.back().second != _node)
    throw RunTimeException(WHERE, "The parent of the duplicate node's child "
                                  "pointer couldn't be found!");
  //Since children got updated for newParent->_node, must remove this bad pointer:
  newParent->children.pop_back();

  //Update parent + children pointers in existingNode and newParent
  //NOTE: _node only has one parent right now, since it's a new node.
  _existingNode->parents.push_back(newParent);
  //Record that newParent can reach the state of _existingNode with the weight
  // found for _node:
  newParent->children.push_back(make_pair(_node->localWeight, _existingNode));

  //Update the removal paths (only one in _node since it only has one parent):
  //There is not an elegant solution in place for having multiple paths
  // given multiple parents. I think we should be safe to make sure to just
  // keep the vector for removal paths and parents aligned, like how things
  // are happening now. This means being careful of this when doing A*.
  _existingNode->removalPaths.push_back(_node->removalPaths[0]);

  //Make sure the single-removal assumption is correct (meaning that only in
  // the case of merging nodes should there be multiple incoming edges):
  if(_existingNode->removalPaths.size() != _existingNode->parents.size())
    throw RunTimeException(WHERE, "In exhaustive, the removal paths should "
                                  "equal the number of parents!");

  if(_existingNode->vid != _node->vid) {
    //Note: The only case where this should happen is for subassemblies
    //  taken out at different depths. Since we offset it proportional to the
    //  depth, the same state might have its subassemblies at different depths
    auto graph = this->GetRoadmap()->GetGraph();

    //Create the roadmap connection newVid -> existingVid, so that both
    // parents have a path from its vid to existingVid.
    const VID newVid = _node->vid;
    const VID existingVid = _existingNode->vid;

    CfgType newCfg = graph->GetVertex(newVid);
    CfgType existingCfg = graph->GetVertex(existingVid);
    const std::vector<CfgType> fakePath = {newCfg, existingCfg};
    WeightType edge("", 1., fakePath);//won't be reproduced
    edge.SetSkipEdge();
    graph->AddEdge(newVid, existingVid, edge);
    if(this->m_debug)
      std::cout << "When merging _node with vid " << _node->vid << " into node"
                << " with vid " << _existingNode->vid << " we had to manually "
                << "create an edge between _node's vid " << newVid
                << " and existingNode's vid " << existingVid
                << std::endl;
  }

  //TODO: when doing A*: check to update cost and everything.
  //  If we update the cost, we need to get all children too.
  if(_node->bestCumulativeWeight < _existingNode->bestCumulativeWeight) {
    //The only local weight that can change is _existingNode's, so do that here:
    _existingNode->localWeight = _node->localWeight;
    //The cumulative weight may change deeper down, recursion will handle that.
    RecursiveUpdateChildrenWeights(_existingNode, newParent, _node->bestCumulativeWeight);
  }
}


template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
RecursiveUpdateChildrenWeights(DisassemblyNode* const _n,
                               DisassemblyNode* const _newParent,
                               const double _newCumulativeWeight) {
  //TODO: Update _n's weights.
  // Then find what position _newParent is in _n->parents, and swap the first
  // element and the one just found. Do the same elemental swap in
  // _n->removalPaths, since we must keep the paths lined up with
  // corresponding parent. (See note above where this is called for why we
  // don't need to update any child's local weight)

  //Then for all children c of _n, if
  // c->localWeight + newCumulativeWeight < c->bestCumulativeWeight, we need to
  // update that child recursively (which handles its children as needed, above)
  // passing (c, _n, c->localWeight + newCumulativeWeight)
}



#endif
