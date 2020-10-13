#ifndef PMPL_DISASSEMBLY_EXHAUSTIVE_GRAPH_H_
#define PMPL_DISASSEMBLY_EXHAUSTIVE_GRAPH_H_

#include <chrono>
#include <queue>

#include "DisassemblyMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Basic serial disassembly method
///
///
/// This is the Exhaustive BFS method that is described in:
/// T. Ebinger, S. Kaden, S. Thomas, R. Andre, N. M. Amato, and U. Thomas,
/// “A general and flexible search framework for disassembly planning,”
/// in International Conference on Robotics and Automation, May 2018.
///
/// This also contains an option to to A* search, which is a future work
/// that is discussed in the above paper.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyExhaustiveGraph : public DisassemblyMethod<MPTraits> {

  public:

    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename GroupCfgType::Formation     Formation;
    typedef typename MPTraits::GroupPathType     GroupPath;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPTraits::GroupWeightType   GroupWeightType;
    typedef typename GroupRoadmapType::VID       VID;
    typedef std::vector<GroupCfgType>            CfgPath;
    typedef std::vector<VID>                     VIDPath;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode  DisassemblyNode;
    typedef typename DisassemblyMethod<MPTraits>::Approach         Approach;
    typedef typename DisassemblyMethod<MPTraits>::State            State;
    typedef std::pair<Formation, std::map<Approach, bool> >                  AttemptEntry;

    DisassemblyExhaustiveGraph();
    DisassemblyExhaustiveGraph(XMLNode& _node);
    virtual ~DisassemblyExhaustiveGraph() {}

    virtual void Iterate();

  protected:
    virtual DisassemblyNode* SelectExpansionNode() override;
    virtual Formation SelectSubassembly(DisassemblyNode* _q) override;
    virtual std::pair<bool, VIDPath> Expand(DisassemblyNode* _q,
                                      const Formation& _subassembly) override;

    std::pair<bool, VIDPath> UnWeightedExpand(DisassemblyNode* _q,
                                              const Formation& _subassembly);
    std::pair<bool, VIDPath> WeightedExpand(DisassemblyNode* _q,
                                          const Formation& _subassembly);

    void RunUnitTests();
    void RunAStarTests();

    //Bucketing interface for faster node matching within the tree:
    void InitializeBuckets();
    void AddToBuckets(DisassemblyNode* _n);
    DisassemblyNode* GetNodeFromBuckets(DisassemblyNode* _n);

    //Since we do vector equality, we need to ensure all vectors are sorted
    // in the same way.
    void EnforcePartSorting(DisassemblyNode* const _node);

    void HandleNewNode(DisassemblyNode* const _node);

    void FixRoadmapVids(DisassemblyNode* const _node,
                        DisassemblyNode* const _existingNode);

    //Merges _node into _existingNode, handling the roadmap edges if needed.
    void MergeNodes(DisassemblyNode* const _node,
                    DisassemblyNode* const _existingNode);

    void RecursiveUpdateChildrenWeights(DisassemblyNode* const _n,
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

    bool m_runUnitTests{false};

    using DisassemblyMethod<MPTraits>::m_disNodes;
};

template <typename MPTraits>
DisassemblyExhaustiveGraph<MPTraits>::
DisassemblyExhaustiveGraph() {
  this->SetName("DisassemblyExhaustiveGraph");
  this->m_graphMethod = true;
}

template <typename MPTraits>
DisassemblyExhaustiveGraph<MPTraits>::
DisassemblyExhaustiveGraph(XMLNode& _node) : DisassemblyMethod<MPTraits>(_node) {
  this->SetName("DisassemblyExhaustiveGraph");
  this->m_graphMethod = true;
  this->m_aStarSearch = _node.Read("aStarSearch", false,
                                  this->m_aStarSearch, "Flag to use A* search");

  this->m_runUnitTests = _node.Read("runUnitTests", false, this->m_runUnitTests,
                            "Flag run unit tests (won't run actual strategy!)");

  m_useRRT = _node.Read("useRRT", false, m_useRRT, "Flag to turn off RRT");
}

template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
InitializeBuckets() {
  if(m_runUnitTests) {
    RunUnitTests();
    // Kill execution, since the tests mess with the roadmap.
    throw RunTimeException(WHERE, "Unit tests passed! Now go turn the "
        "runUnitTests flag to false in your XML file to run the strategy.");
  }


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

  if(sortMe.initialParts != Formation({1,3,4,5,6}))
    throw RunTimeException(WHERE, "Initial Parts sorting test failed!!!");

  std::vector<Formation> sortedSubs =
                                  {{0,17,18,19,20},{2},{7,8},{9,10,11,12},{21}};
  if(sortMe.usedSubassemblies != sortedSubs)
    throw RunTimeException(WHERE, "Formation sorting test failed!!!");

  m_nodeBuckets.clear();//Clean up
  this->m_disNodes.clear();


  if(this->m_aStarSearch)
    RunAStarTests();
}


template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
RunAStarTests() {
  ////////////////////////UNIT TEST///////////////////////////
  //2- Make root node
  //3- Generate graph
  //4- Verify

  //1//- Create configurations including sub&parts motion
  const auto g = this->GetGroupRoadmap();

  const GroupCfgType root = g->GetVertex(0);
  const size_t dofsPerPart = root.PosDOF() + root.OriDOF();
  if(root.DOF()/dofsPerPart < 5)
    throw RunTimeException(WHERE, "The problem was not set up with enough parts"
                                  " to properly run the unit test!");

  GroupCfgType cfg0 = root, cfg1 = root, cfg2 = root, cfg3 = root, cfg4 = root,
          cfg5 = root, cfg6 = root, cfg7 = root, cfg8 = root;

  const Vector3d xTrans(5,0,0);
  const Vector3d yTrans(0,5,0);

  cfg0.AddDofsForRobots(xTrans, {4, 5});
//  cfg0[dofsPerPart*4] += 5;
//  CFG0[DOFSPERPART*5] += 5;//x trans of sub 4&5

  cfg1.AddDofsForRobots(xTrans, {3});
//  cfg1[dofsPerPart*3] += 5;// x trans of part 3

  cfg2.AddDofsForRobots(xTrans, {2, 3});
//  cfg2[dofsPerPart*2] += 5;
//  cfg2[dofsPerPart*3] += 5;//x trans of sub 2&3

  cfg3.AddDofsForRobots(xTrans, {2});
//  cfg3[dofsPerPart*2] += 5;//x trans of part 2

  cfg4.AddDofsForRobots(xTrans, {1, 2});
//  cfg4[dofsPerPart*1] += 5;
//  cfg4[dofsPerPart*2] += 5;//x trans of sub 1&2

  cfg5.AddDofsForRobots(xTrans, {1});
//  cfg5[dofsPerPart*1] += 5;//x trans of part 1
  cfg6.AddDofsForRobots(yTrans, {1});
//  cfg6[dofsPerPart*1 + 1] += 5;//y trans of part 1

  cfg7.AddDofsForRobots(yTrans, {4});
//  cfg7[dofsPerPart*4 + 1] += 5;//y trans part 4

  cfg8.AddDofsForRobots(xTrans, {3});
//  cfg8[dofsPerPart*3] += 5; //x trans of part 3

  //2//- Create the nodes

  DisassemblyNode l0_obj;
  DisassemblyNode* l0 = &l0_obj;
  l0->initialParts = {1,2,3,4};
  l0->usedSubassemblies = {{5,6}};
  l0->vid = g->AddVertex(cfg0);


  //3//- Define motion from parent to child

  /// TODO: Update all of this to use VIDs instead of cfgs and rerun (was
  ///       working with cfg version though).

//  std::vector<std::vector<GroupCfgType> > v1, v2, v3, v4, v5, v6, v7, v8;
//  v1.push_back({cfg0, cfg1});
//  v2.push_back({cfg0, cfg2});
//  v3.push_back({cfg1, cfg3});
//  v4.push_back({cfg1, cfg4});
//  v5.push_back({cfg2, cfg5});
//  v6.push_back({cfg4, cfg6});
//  v7.push_back({cfg4, cfg7});
//  v8.push_back({cfg8, cfg1});
//
//  //4//- Generate the three
//  DisassemblyNode* l1 = this->GenerateNode(l0, Formation({4}), v1, true, 3);
//  HandleNewNode(l1);
//
//  DisassemblyNode* l2 = this->GenerateNode(l0, Formation({3, 4}), v2, true, 4);
//  HandleNewNode(l2);
//
//  DisassemblyNode* l3 = this->GenerateNode(l1, Formation({3}), v3, true, 4);
//  HandleNewNode(l3);
//
//  DisassemblyNode* l4 = this->GenerateNode(l1, Formation({2, 3}), v4, true, 1);
//  HandleNewNode(l4);
//
//  DisassemblyNode* l5 = this->GenerateNode(l2, Formation({2}), v5, true, 4);
//  HandleNewNode(l5);
//
//  DisassemblyNode* l6 = this->GenerateNode(l4, Formation({2}), v6, true, 1);
//  HandleNewNode(l6);
//
//  DisassemblyNode* l7 = this->GenerateNode(l4, Formation({5}), v7, true, 2);
//  HandleNewNode(l7);
//
//  DisassemblyNode* l8 = this->GenerateNode(l0, Formation({4}), v8, true, 2);
//  HandleNewNode(l8);
//
//  //5//- Verify the data
//  //compare the expected weight VS the recursive function result finalWeight
//  if (l6->bestCumulativeWeight != 4 || l7->bestCumulativeWeight != 5
//      || l3->bestCumulativeWeight != 6 || l1->bestCumulativeWeight != 2)
//    throw RunTimeException(WHERE, "Weight checking test failed!");
//  else
//    std::cout << "Weight checking test success!" << std::endl;
//
//  //if the newPrent is now l8, the function have correctly choose the cost-effective path
//  if (l1->parents[0] == l0 && l3->parents[0] == l1 && l4->parents[0] == l1
//      && l6->parents[0] == l4 && l7->parents[0] == l4)
//    std::cout << "Parents checking test successful!" << std::endl;
//  else
//    throw RunTimeException(WHERE, "Parents checking test failed!!!");
//
//  m_nodeBuckets.clear(); // Clean up
//  this->m_disNodes.clear();
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
    bool operator() (const Formation& i, const Formation& j) {
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
    std::cout << this->GetNameAndLabel() << "::Iterate()" << std::endl;

  //Only one or the other should be used, so one queue should always be empty:
  if(!m_disNodes.empty() && m_nodeQueueAStar.empty() && m_nodeQueueBFS.empty()){
    this->m_successful = true;
    std::cout << std::endl << "Successful disassembling!" << std::endl << std::endl;
    return;
  }

  DisassemblyNode* node = SelectExpansionNode();
  if(!node) {
    std::cout << "Error: returned nullptr node" << std::endl;
    return;
  }
  else if(node->GetCompletePartList().empty()) {
    std::cout << "Error: select Node with empty parts" << std::endl;
    return;
  }

  Expand(node, Formation());
}

template <typename MPTraits>
typename DisassemblyExhaustiveGraph<MPTraits>::DisassemblyNode*
DisassemblyExhaustiveGraph<MPTraits>::
SelectExpansionNode() {
  const size_t candSize = this->m_aStarSearch ? m_nodeQueueAStar.size() :
                                                m_nodeQueueBFS.size();
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectExpansionCfg() | "
         << "Size of candidates = " << candSize << std::endl;

  // check if first iteration
  if (m_disNodes.empty()) {
    std::vector<size_t> robotParts;
    for (size_t i = 0; i < this->m_numParts; ++i)
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
  if(this->m_aStarSearch) {
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
typename DisassemblyExhaustiveGraph<MPTraits>::Formation
DisassemblyExhaustiveGraph<MPTraits>::
SelectSubassembly(DisassemblyNode* _q) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectSubassembly()" << std::endl;
  throw RunTimeException(WHERE, "Not used by this method");
  return Formation();
}

template <typename MPTraits>
std::pair<bool, typename DisassemblyExhaustiveGraph<MPTraits>::VIDPath>
DisassemblyExhaustiveGraph<MPTraits>::
Expand(DisassemblyNode* _node, const Formation& _subassembly) {
  if(this->m_aStarSearch)
    return WeightedExpand(_node, _subassembly);
  else
    return UnWeightedExpand(_node, _subassembly);
}

template <typename MPTraits>
std::pair<bool, typename DisassemblyExhaustiveGraph<MPTraits>::VIDPath>
DisassemblyExhaustiveGraph<MPTraits>::
UnWeightedExpand(DisassemblyNode* _node, const Formation& _subassembly) {
  /*if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Expand with single-part subassemblies"
         << std::endl;

  VID newVID;
  std::vector<VIDPath> removingPaths;
  Formation singleSub;
  DisassemblyNode* newNode = nullptr;

  // first test all single bodies for expansion with mating and rrt approach
  // generate list with all parts (at initial pose and usedSubassemblies)
  auto parts = _node->initialParts;
  for (auto &usedSub : _node->usedSubassemblies)
    for (auto &part : usedSub)
      parts.push_back(part);

  //1. Attempt all single parts: mating, then RRT if mating fails.
  for (auto &part : parts) {
    Formation sub = {part};
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
    std::cout << this->GetNameAndLabel() << "::Expand with multi-part subassemblies"
         << std::endl;

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
*/
  return std::make_pair(true, VIDPath());
}


template <typename MPTraits>
std::pair<bool, typename DisassemblyExhaustiveGraph<MPTraits>::VIDPath>
DisassemblyExhaustiveGraph<MPTraits>::
WeightedExpand(DisassemblyNode* _node, const Formation& _subassembly) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Expand with single-part subassemblies"
              << std::endl;

  // TODO: To implement a different metric and heuristic, simply swap out the
  // timer values with the addition of the metric value and the heuristic
  // estimate to the goal. Just like in normal A*, as long as the heuristic is
  // an underestimate with respect to the current state's optimal distance to
  // the goal, then the strategy will still return the optimal path.

  VID newVID;
  std::vector<VIDPath> removingPaths;
  Formation singleSub;
  DisassemblyNode* newNode = nullptr;

  // first test all single bodies for expansion with mating and rrt approach
  // generate list with all parts (at initial pose and usedSubassemblies)
  auto parts = _node->initialParts;
  for (auto &usedSub : _node->usedSubassemblies)
    for (auto &part : usedSub)
      parts.push_back(part);

  //1. Attempt all single parts: mating, then RRT if mating fails.
  for (auto &part : parts) {
    Formation sub = {part};
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
    std::cout << this->GetNameAndLabel() << "::Expand with multi-part subassemblies"
              << std::endl;

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

  return std::make_pair(true, VIDPath());
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
    if(this->m_aStarSearch)
      m_nodeQueueAStar.push(_node);
    else
      m_nodeQueueBFS.push_back(_node);

    AddToBuckets(_node);
  }
}


template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
FixRoadmapVids(DisassemblyNode* const _node,
               DisassemblyNode* const _existingNode) {
  if(_existingNode->vid != _node->vid) {
    //  The only case where this should happen is for subassemblies
    //  taken out at different depths. Since we offset it proportional to the
    //  depth, the same state might have its subassemblies at different offsets.
    auto graph = this->GetGroupRoadmap();

    //Create the roadmap connection newVid -> existingVid, so that both
    // parents have a path from its vid to existingVid.
    const VID newVid = _node->vid;
    const VID existingVid = _existingNode->vid;

    const GroupCfgType& newCfg = graph->GetVertex(newVid);
    const GroupCfgType& existingCfg = graph->GetVertex(existingVid);
    const std::vector<GroupCfgType> fakePath = {newCfg, existingCfg};


    GroupWeightType edge(graph, "", 1., fakePath);
    GroupLPOutput<MPTraits> lp(graph, edge);
    lp.SetSkipEdge();// Won't be reproduced.
    graph->AddEdge(newVid, existingVid, lp.m_edge);
    if(this->m_debug)
      std::cout << "When merging _node with vid " << _node->vid << " into node"
                << " with vid " << _existingNode->vid << " we had to manually "
                << "create an edge between _node's vid " << newVid
                << " and existingNode's vid " << existingVid
                << std::endl;
  }
}

template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
MergeNodes(DisassemblyNode* const _node, DisassemblyNode* const _existingNode) {
  //This takes in _node that has been matched as the same state as _existingNode
  // and updates pointers in _existingNode and _node's parent.
  DisassemblyNode* const newParent = _node->parents.at(0);
  if(this->m_debug) {
    if(_node != &this->m_disNodes.back())
      throw RunTimeException(WHERE, "Something went wrong: a duplicate node "
                             "was added, but its tree entry can't be found!");
    if(_node->parents.size() > 1 || _node->children.size() > 0)
      throw RunTimeException(WHERE, "Duplicate node's parent's or children "
                             "have more elements than should be possible!!!");
    if(newParent->children.back().second != _node)
      throw RunTimeException(WHERE, "The parent of the duplicate node's child "
                                    "pointer couldn't be found!");
    //Make sure the single-removal assumption is correct (meaning that only in
    // the case of merging nodes should there be multiple incoming edges):
    if(_existingNode->removalPaths.size() != _existingNode->parents.size())
      throw RunTimeException(WHERE, "In exhaustive, the removal paths should "
                                    "equal the number of parents!");
  }
  //Since children got updated for newParent->_node (in GenerateNode), must
  // remove this soon-to-be bad pointer:
  newParent->children.pop_back();

  //Update parent + children pointers in existingNode and newParent
  //NOTE: _node only has one parent right now, since it's a new node.
  _existingNode->parents.push_back(newParent);
  //Record that newParent can reach the state of _existingNode with the weight
  // found for _node:
  newParent->children.push_back(std::make_pair(_node->localWeight, _existingNode));

  //Update the removal paths (only one in _node since it only has one parent):
  // Note that we HAVE to keep this aligned with the node's parents vector!
  _existingNode->removalPaths.push_back(_node->removalPaths[0]);

  //If subassemblies caused the vids to not match, fix the roadmap:
  FixRoadmapVids(_existingNode, _node);

  //  If we update the cost, we need to check all children recursively.
  if(_node->bestCumulativeWeight < _existingNode->bestCumulativeWeight) {
    //The recursion starts with _existingNode's children, so first update it:
    _existingNode->bestCumulativeWeight = _node->bestCumulativeWeight;
    //The only local weight that can change is _existingNode's, so do that here:
    _existingNode->localWeight = _node->localWeight;

    // We have the new data at the back of the two vectors; since it's better,
    // swap both to the front to indicate it's the new best path.
    const size_t last = _existingNode->parents.size() - 1;
    std::swap(_existingNode->parents[0], _existingNode->parents[last]);
    std::swap(_existingNode->removalPaths[0], _existingNode->removalPaths[last]);

    //The cumulative weight may change deeper down, recursion will handle that.
    RecursiveUpdateChildrenWeights(_existingNode, _node->bestCumulativeWeight);
  }
}


template <typename MPTraits>
void
DisassemblyExhaustiveGraph<MPTraits>::
RecursiveUpdateChildrenWeights(DisassemblyNode* const _n,
                               const double _newCumulativeWeight) {
  if(this->m_debug)
    std::cout << "RecursiveUpdateChildrenWeights: on node with vid " << _n->vid
              << " and with _newCumulativeWeight = " << _newCumulativeWeight
              << std::endl;

  for (size_t i = 0; i < _n->children.size(); ++i) {
    //Check if this child needs updating:
    DisassemblyNode* const child = _n->children[i].second;
    const double newWeight = _newCumulativeWeight + child->localWeight;
    if (child->bestCumulativeWeight > newWeight) {
      child->bestCumulativeWeight = newWeight;
      // Swap the child's pointer to the parent to the front, as that should
      // always correspond to the best path towards the root from this node.
      for (size_t j = 0; j < child->parents.size(); ++j) {
        if (child->parents[j] == _n) {
          // Swap this value with the first element of the list
          std::swap(child->parents[0], child->parents[j]);
          std::swap(child->removalPaths[0], child->removalPaths[j]);
          break;
        }
      }

      // Recurse deeper since this child was updated.
      RecursiveUpdateChildrenWeights(child, newWeight);
    }
  }
}


#endif
