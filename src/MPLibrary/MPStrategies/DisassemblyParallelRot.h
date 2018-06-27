#ifndef DISASSEMBLY_PARALLEL_ROT_H_
#define DISASSEMBLY_PARALLEL_ROT_H_

#include "DisassemblyMethod.h"
#include "nonstd.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Basic serial disassembly method
///
/// TODO remove this strategy once we are positive subassembly rotations are
/// 100% good.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyParallelRot : public DisassemblyMethod<MPTraits> {
  public:
//    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename DisassemblyMethod<MPTraits>::VID            VID;
    typedef typename DisassemblyMethod<MPTraits>::VIDPath        VIDPath;
    typedef typename DisassemblyMethod<MPTraits>::Formation       Formation;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode DisassemblyNode;
    typedef typename DisassemblyMethod<MPTraits>::Approach        Approach;
    typedef typename DisassemblyMethod<MPTraits>::State           State;

    DisassemblyParallelRot(
        const map<string, pair<size_t, size_t> >& _matingSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const map<string, pair<size_t, size_t> >& _rrtSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const string _vc = "", const string _singleVc = "",
        const string _lp = "", const string _ex = "",
        const string _dm = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    DisassemblyParallelRot(XMLNode& _node);
    virtual ~DisassemblyParallelRot() {}

    virtual void Iterate();

  protected:
    virtual DisassemblyNode* SelectExpansionNode() override;
    virtual Formation SelectSubassembly(DisassemblyNode* _q) override;
    virtual pair<bool, VIDPath> Expand(DisassemblyNode* _q,
                                   const Formation& _subassembly) override;

    void AppendNode(DisassemblyNode* _parent,
                    const vector<size_t>& _removedParts,
                    const vector<VIDPath>& _removingPaths,
                    const bool _isMultiPart);

    void ComputeSubassemblies(DisassemblyNode* _node);

    //Generates 2 nodes after finding a way to remove a subassembly:
    // 1. The subassembly itself as its own node, so that it gets disassembled.
    // 2. The current assembly without the subassembly, to return to after
    //      finishing with the subassembly.
    void GenerateFormationNodes(DisassemblyNode* _parent,
                                  const Formation& _subassembly);

    Approach m_approach = Approach::mating;
    State m_state = State::singlePart;

    DisassemblyNode* m_lastNode = nullptr;
    size_t m_curSubIndex = 0;

    bool m_noSubassemblies{false};

    // subassembly candidates
    vector<Formation> m_subassemblies;

    using DisassemblyMethod<MPTraits>::m_disNodes;
    using DisassemblyMethod<MPTraits>::m_numParts;
};

template <typename MPTraits>
DisassemblyParallelRot<MPTraits>::
DisassemblyParallelRot(
    const map<string, pair<size_t, size_t> >& _matingSamplerLabels,
    const map<string, pair<size_t, size_t> >& _rrtSamplerLabels,
    const string _vc, const string _singleVc,
    const string _lp, const string _ex,
    const string _dm, const vector<string>& _evaluatorLabels) :
    DisassemblyMethod<MPTraits>(_matingSamplerLabels, _rrtSamplerLabels, _vc,
      _singleVc, _lp, _ex, _dm, _evaluatorLabels) {
  this->SetName("DisassemblyParallelRot");
}

template <typename MPTraits>
DisassemblyParallelRot<MPTraits>::
DisassemblyParallelRot(XMLNode& _node) : DisassemblyMethod<MPTraits>(_node) {
  this->SetName("DisassemblyParallelRot");

  m_noSubassemblies = _node.Read("noSubassemblies", false, m_noSubassemblies,
                                 "Use multi-part subassemblies or not.");
}

template <typename MPTraits>
void
DisassemblyParallelRot<MPTraits>::
Iterate() {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Iterate()" << endl;

  if (!m_lastNode && !m_disNodes.empty()) { // check if disassembly is complete
    cout << endl << endl << "Disassembling complete!" << endl << endl;
    this->m_successful = true;
    return;
  }

  vector<size_t> removedParts;
  vector<VIDPath> removingPaths;
  Formation subassembly;

  DisassemblyNode* node = SelectExpansionNode();



  m_approach = Approach::rrt; //So we skip everything, do RRT immediately.
  if (removedParts.empty()) {
    m_state = State::multiPart;
    ComputeSubassemblies(node);
    for (auto &sub : m_subassemblies) {
      pair<bool, VIDPath> result = Expand(node, sub);
      if (result.first) {
        removedParts = sub;
        removingPaths.push_back(result.second);
        break;
      }
    }
  }

  this->m_successful = true;

  if (removedParts.empty()) {
    this->m_successful = true;
    std::cout << std::endl << std::endl << "Disassembling not completed!!!"
              << std::endl << std::endl;
    return;
  }

  if (!removedParts.empty()) { // valid separation
    if (m_state == State::singlePart)
      AppendNode(node, removedParts, removingPaths, false);
    else if (m_state == State::multiPart)
      AppendNode(node, removedParts, removingPaths, true);

    return;
  }
}

template <typename MPTraits>
typename DisassemblyParallelRot<MPTraits>::DisassemblyNode*
DisassemblyParallelRot<MPTraits>::
SelectExpansionNode() {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::SelectExpansionCfg()" << endl;

  // check if first iteration
  if (m_disNodes.empty()) {
    vector<size_t> robotParts;
    for (size_t i = 0; i < m_numParts; ++i)
      robotParts.push_back(i);

    DisassemblyNode node;
    node.initialParts = robotParts;
    node.vid = this->m_rootVid;
    m_disNodes.push_back(node);
    m_lastNode = &m_disNodes.back();
    this->m_rootNode = &m_disNodes.back();

    //All remaining members to reset:
    m_approach = Approach::mating;
    m_state = State::singlePart;
    m_curSubIndex = 0;
    m_subassemblies.clear();

    return this->m_rootNode;
  }

  return m_lastNode;
}

template <typename MPTraits>
typename DisassemblyMethod<MPTraits>::Formation
DisassemblyParallelRot<MPTraits>::
SelectSubassembly(DisassemblyNode* _q) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::SelectSubassembly()" << endl;

  return Formation();
}

template <typename MPTraits>
pair<bool, typename DisassemblyParallelRot<MPTraits>::VIDPath>
DisassemblyParallelRot<MPTraits>::
Expand(DisassemblyNode* _q, const Formation& _subassembly) {
  if (_subassembly.empty())
    return make_pair(false, VIDPath());
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Expand with Formation: "
              << _subassembly << std::endl;

  VID newVID;
  VIDPath path;
  if (m_approach == Approach::rrt) // choose between RRT and mating approach
    path = this->ExpandRRTApproach(_q->vid, _subassembly, newVID);
  else
    path = this->ExpandMatingApproach(_q->vid, _subassembly, newVID);

  if (!path.empty())
    return make_pair(true, path);
  else
    return make_pair(false, path);
}

template <typename MPTraits>
void
DisassemblyParallelRot<MPTraits>::
AppendNode(DisassemblyNode* _parent, const vector<size_t>& _removedParts,
           const vector<VIDPath>& _removingPaths,
           const bool _isMultiPart) {

  // update node to new state
  m_lastNode = this->GenerateNode(_parent, _removedParts, _removingPaths,
                                  _isMultiPart);
  //If there are no remaining parts, pop this off to trigger success.
  if(m_lastNode->GetCompletePartList().empty())
    m_lastNode = nullptr;
}

template <typename MPTraits>
void
DisassemblyParallelRot<MPTraits>::
ComputeSubassemblies(DisassemblyNode* _node) {
  m_subassemblies.clear();
  if(!m_noSubassemblies) {
    vector<Formation> subassemblies =
                  this->GenerateSubassemblies(_node->vid, _node->initialParts);
    for (const auto &usedSub : _node->usedSubassemblies) {
      vector<Formation> subs =
                               this->GenerateSubassemblies(_node->vid, usedSub);
      if (!subs.empty())
        subassemblies.insert(subassemblies.end(), subs.begin(), subs.end());
    }
    m_subassemblies = subassemblies;
  }
}

#endif
