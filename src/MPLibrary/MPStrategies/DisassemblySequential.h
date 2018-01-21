#ifndef DISASSEMBLY_SEQUENTIAL_H_
#define DISASSEMBLY_SEQUENTIAL_H_

#include <chrono>

#include "DisassemblyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Basic serial disassembly method
///
///
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblySequential : public DisassemblyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::GraphType      GraphType;
    typedef typename RoadmapType::VID            VID;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode DisassemblyNode;
    typedef vector<unsigned int>                 Subassembly;
    typedef typename DisassemblyMethod<MPTraits>::Approach Approach;
    typedef typename DisassemblyMethod<MPTraits>::State State;
    typedef pair<Subassembly, map<Approach, bool> > AttemptEntry;

    DisassemblySequential(
        const map<string, pair<size_t, size_t> >& _matingSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const map<string, pair<size_t, size_t> >& _rrtSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const string _vc = "", const string _singleVc = "",
        const string _lp = "", const string _ex = "",
        const string _dm = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    DisassemblySequential(XMLNode& _node);
    virtual ~DisassemblySequential() {}

    virtual void Iterate();

  protected:
    virtual DisassemblyNode* SelectExpansionNode() override;
    virtual Subassembly SelectSubassembly(DisassemblyNode* _q) override;
    virtual pair<bool, vector<CfgType>> Expand(DisassemblyNode* _q,
                                      const Subassembly& _subassembly) override;

    list<DisassemblyNode*> m_nodeQueue;

    bool m_useRRT{true};

    using DisassemblyMethod<MPTraits>::m_disNodes;
};

template <typename MPTraits>
DisassemblySequential<MPTraits>::
DisassemblySequential(
    const map<string, pair<size_t, size_t> >& _matingSamplerLabels,
    const map<string, pair<size_t, size_t> >& _rrtSamplerLabels,
    const string _vc, const string _singleVc,
    const string _lp, const string _ex,
    const string _dm, const vector<string>& _evaluatorLabels) :
    DisassemblyMethod<MPTraits>(_matingSamplerLabels, _rrtSamplerLabels, _vc,
      _singleVc, _lp, _ex, _dm, _evaluatorLabels) {
  this->SetName("DisassemblySequential");
}

template <typename MPTraits>
DisassemblySequential<MPTraits>::
DisassemblySequential(XMLNode& _node) : DisassemblyMethod<MPTraits>(_node) {
  this->SetName("DisassemblySequential");

  m_useRRT = _node.Read("useRRT", false, m_useRRT, "Flag to turn off RRT");
}

template <typename MPTraits>
void
DisassemblySequential<MPTraits>::
Iterate() {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Iterate()" << endl;

  if(m_nodeQueue.empty() && !m_disNodes.empty()) {
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
typename DisassemblySequential<MPTraits>::DisassemblyNode*
DisassemblySequential<MPTraits>::
SelectExpansionNode() {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::SelectExpansionCfg() | "
         << "Size of candidates = " << m_nodeQueue.size() << endl;

  // check if first iteration
  if (m_disNodes.empty()) {
    vector<unsigned int> robotParts;
    for (unsigned int i = 0; i < this->m_numParts; ++i)
      robotParts.push_back(i);

    DisassemblyNode node;
    node.vid = this->m_rootVid;
    node.initialParts = robotParts;
    m_disNodes.push_back(node);

    m_nodeQueue.clear();

    this->m_rootNode = &m_disNodes.back();

    return this->m_rootNode;
  }

  if(m_nodeQueue.empty())
    return nullptr;

  // return the first node of the queue and remove it at the same time
  auto node = m_nodeQueue.front();
  m_nodeQueue.pop_front();
  return node;
}


template <typename MPTraits>
vector<unsigned int>
DisassemblySequential<MPTraits>::
SelectSubassembly(DisassemblyNode* _q) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::SelectSubassembly()" << endl;
  return Subassembly();
}

template <typename MPTraits>
pair<bool, vector<typename DisassemblySequential<MPTraits>::CfgType>>
DisassemblySequential<MPTraits>::
Expand(DisassemblyNode* _node, const Subassembly& _subassembly) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Expand with single-part subassemblies"
         << endl;

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
      m_nodeQueue.push_back(newNode);
  }

  if (this->m_debug)
    cout << this->GetNameAndLabel() << "::Expand with multi-part subassemblies"
         << endl;
  // second test all possible multi part subassemblies
  // here are two different kinds, as first the initialParts at initial position
  // and secondly the used subassemblies

  // remaining part subassemblies
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
      m_nodeQueue.push_back(newNode);
  }

  // usedSubassembly subassemblies
  for (auto &usedSubassembly : _node->usedSubassemblies) {
    subassemblies = this->GenerateSubassemblies(_node->vid, usedSubassembly);
    for (auto &multiSub : subassemblies) {
      if (multiSub.size() == usedSubassembly.size()) // skip if entire assembly
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
        m_nodeQueue.push_back(newNode);
    }
  }

  return make_pair(true, vector<CfgType>());
}

#endif
