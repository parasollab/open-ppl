#ifndef DISASSEMBLY_PARALLEL_H_
#define DISASSEMBLY_PARALLEL_H_

#include "DisassemblyMethod.h"
#include "nonstd.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is the Preemptive DFS technique, with a few minor variants
///        built in as options.
///
///
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyParallel : public DisassemblyMethod<MPTraits> {

  public:
//    typedef typename MPTraits::GroupCfgType                       GroupCfgType;
    typedef typename DisassemblyMethod<MPTraits>::VID             VID;
    typedef typename DisassemblyMethod<MPTraits>::VIDPath         VIDPath;
    typedef typename DisassemblyMethod<MPTraits>::Formation       Formation;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode DisassemblyNode;
    typedef typename DisassemblyMethod<MPTraits>::Approach        Approach;
    typedef typename DisassemblyMethod<MPTraits>::State           State;

    DisassemblyParallel(
        const map<string, pair<size_t, size_t> >& _matingSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const map<string, pair<size_t, size_t> >& _rrtSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const string _vc = "", const string _singleVc = "",
        const string _lp = "", const string _ex = "",
        const string _dm = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    DisassemblyParallel(XMLNode& _node);
    virtual ~DisassemblyParallel() {}

    virtual void Iterate();

  protected:
    virtual DisassemblyNode* SelectExpansionNode() override;
    virtual Formation SelectSubassembly(DisassemblyNode* _q) override {
      throw RunTimeException(WHERE, "Unused by this strategy");
      return Formation();
    }
    virtual pair<bool, VIDPath> Expand(DisassemblyNode* _q,
                                   const Formation& _subassembly) override;

    void AppendNode(DisassemblyNode* _parent,
                    const vector<size_t>& _removedParts,
                    const vector<VIDPath>& _removingPaths,
                    const bool _isMultiPart);

    void ComputeSubassemblies(DisassemblyNode* _node);

    //Generates 2 nodes after finding a way to remove a subassembly:
    // 1. The subassembly itself as its own node, so it gets fully disassembled.
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
    std::vector<Formation> m_subassemblies;

    // Determine whether to do (pseudo) in-parallel removals or not:
    // setting default to false as this is not yet supported for groups
    bool m_useParallelRemoval{false};

    using DisassemblyMethod<MPTraits>::m_disNodes;
    using DisassemblyMethod<MPTraits>::m_numParts;
};

template <typename MPTraits>
DisassemblyParallel<MPTraits>::
DisassemblyParallel(
    const map<string, pair<size_t, size_t> >& _matingSamplerLabels,
    const map<string, pair<size_t, size_t> >& _rrtSamplerLabels,
    const string _vc, const string _singleVc,
    const string _lp, const string _ex,
    const string _dm, const vector<string>& _evaluatorLabels) :
    DisassemblyMethod<MPTraits>(_matingSamplerLabels, _rrtSamplerLabels, _vc,
      _singleVc, _lp, _ex, _dm, _evaluatorLabels) {
  this->SetName("DisassemblyParallel");
}

template <typename MPTraits>
DisassemblyParallel<MPTraits>::
DisassemblyParallel(XMLNode& _node) : DisassemblyMethod<MPTraits>(_node) {
  this->SetName("DisassemblyParallel");

  m_noSubassemblies = _node.Read("noSubassemblies", false, m_noSubassemblies,
                                 "Use multi-part subassemblies or not.");

  m_useParallelRemoval = _node.Read("useParallelRemoval", false,
                                    m_useParallelRemoval,
                                   "Use (pseudo) in-parallel removals.");
  if(m_useParallelRemoval)
    throw RunTimeException(WHERE, "Parallel removal is not currently supported "
                                  "for groups.");
}

template <typename MPTraits>
void
DisassemblyParallel<MPTraits>::
Iterate() {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Iterate()" << std::endl;

  if (!m_lastNode && !m_disNodes.empty()) { // check if disassembly is complete
    std::cout << std::endl << std::endl << "Disassembling complete!"
              << std::endl << std::endl << std::endl;
    this->m_successful = true;
    return;
  }

  std::vector<size_t> removedParts;
  std::vector<VIDPath > removingPaths;
  Formation subassembly;

  DisassemblyNode* node = SelectExpansionNode();

  m_state = State::singlePart; // Always will start with single parts
  if(!node->determinismExhausted)
    m_approach = Approach::mating;
  else
    m_approach = Approach::rrt;

  Formation completePartList = node->GetCompletePartList();





  // TODO: remove this, just an optimization while testing simple coax with
  // group stuff:
  std::reverse(completePartList.begin(), completePartList.end());





  if(!node->determinismExhausted) { // In here are the deterministic methods:
    // Mating vector single parts:
    if(m_state == State::singlePart && m_approach == Approach::mating) {
      for (const size_t id : completePartList) {
        subassembly = {id};
        std::pair<bool,VIDPath> result = Expand(node, subassembly);
        if (result.first) {
          removedParts.push_back(id);
          removingPaths.push_back(result.second);
          if(!m_useParallelRemoval) // This will make it so we put off the part
            break;         // to the side, then come back later for other parts
        }
      }
    }

    if (removedParts.empty()) {
      // Mating vector multiple parts:
      m_state = State::multiPart;
      ComputeSubassemblies(node);
      for (auto &sub : m_subassemblies) {
        if (sub.size() == completePartList.size() && node->initialParts.empty())
          continue; // step over complete subassemblies apart from init position
        std::pair<bool, VIDPath > result = Expand(node, sub);
        if (result.first) {
          removedParts = sub;
          removingPaths.push_back(result.second);
          break;
        }
      }
    }
    node->determinismExhausted = true; // Never retry mating for this node.
  }
  else { // Determinism exhausted, attempt RRT until timeout or state change.
    // RRT single part:
    for (const size_t id : completePartList) {
      subassembly = Formation({id});
      std::pair<bool, VIDPath > result = Expand(node, subassembly);
      if (result.first) {
        removedParts.push_back(id);
        removingPaths.push_back(result.second);
        break; // We only do "simultaneous" removals for mating single parts.
      }
    }

    // RRT multi-part:
    if (removedParts.empty()) {
      m_state = State::multiPart;
      ComputeSubassemblies(node);
      for (const Formation& sub : m_subassemblies) {
        std::pair<bool, VIDPath > result = Expand(node, sub);
        if (result.first) {
          removedParts = sub;
          removingPaths.push_back(result.second);
          break;
        }
      }
    }
  }

  if(!removedParts.empty()) // Means a successful separation occurred.
    AppendNode(node, removedParts, removingPaths, m_state == State::multiPart);
}

template <typename MPTraits>
typename DisassemblyParallel<MPTraits>::DisassemblyNode*
DisassemblyParallel<MPTraits>::
SelectExpansionNode() {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectExpansionCfg()"
              << std::endl;

  // check if first iteration
  if (m_disNodes.empty()) {
    std::vector<size_t> robotParts;
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
std::pair<bool, typename DisassemblyParallel<MPTraits>::VIDPath >
DisassemblyParallel<MPTraits>::
Expand(DisassemblyNode* _q, const Formation& _subassembly) {
  if(_subassembly.empty())
    return std::make_pair(false, VIDPath());

  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Expand with Formation: "
              << _subassembly << std::endl << "And remaining parts: "
              << _q->GetCompletePartList() << std::endl;

  VID newVID = 0; // This will be set by the chosen Expand approach.
  VIDPath path;
  // choose between RRT and mating approach
  if (m_approach == Approach::rrt)
    path = this->ExpandRRTApproach(_q->vid, _subassembly, newVID);
  else
    path = this->ExpandMatingApproach(_q->vid, _subassembly, newVID);

  if(path.empty()) {
    if(this->m_keepBestRRTPathOnFailure) {
      if(newVID != 0) {
        //Then a path ending at newVID was found. Here it should always be the
        //case that _q->vid can reach newVID in the roadmap, so we can update
        //the node's vid so that we start from this progress in the future
        //whenever we next attempt this node (usually immediately)
        std::cout << "Unsuccessful RRT, but updating node's vid from "
                  << _q->vid << " to " << newVID << " and resetting determinism"
                  << " flag for this node." << std::endl;
        _q->vid = newVID;
        // Reset node's flag; can now retry mating from new sub-state:
        _q->determinismExhausted = false;
      }
    }
    return std::make_pair(false, path); // Don't create a new disassembly node.
  }

  return std::make_pair(true, path);
}

template <typename MPTraits>
void
DisassemblyParallel<MPTraits>::
AppendNode(DisassemblyNode* _parent, const vector<size_t>& _removedParts,
       const vector<VIDPath>& _removingPaths, const bool _isMultiPart) {

  // update node to new state
  m_lastNode = this->GenerateNode(_parent, _removedParts, _removingPaths,
                                  _isMultiPart);
  //If there are no remaining parts, pop this off to trigger success.
  if(m_lastNode->GetCompletePartList().empty())
    m_lastNode = nullptr;
}

template <typename MPTraits>
void
DisassemblyParallel<MPTraits>::
ComputeSubassemblies(DisassemblyNode* _node) {
  m_subassemblies.clear();
  if(!m_noSubassemblies) {
    //First get initial part subassemblies:
    m_subassemblies =
                  this->GenerateSubassemblies(_node->vid, _node->initialParts);
    //Get all sub-subassemblies that could be used from usedSubassemblies:
    for (const auto &usedSub : _node->usedSubassemblies) {
      vector<Formation> subs =
                              this->GenerateSubassemblies(_node->vid, usedSub);
      if (!subs.empty()) {
        //Don't reuse any complete subassemblies already in usedSubassemblies:
        for(const auto& sub : subs) {
          bool add = true;
          for(const auto& usedSub : _node->usedSubassemblies)
            if(usedSub == sub)
              add = false; // This subassembly is pointless to re-attempt.

          if(add)
            m_subassemblies.push_back(sub);
        }
      }
    }
  }
  if(this->m_debug) {
    if(!m_subassemblies.empty())
      std::cout << "After ComputeSubassemblies: m_subassemblies = "
                << m_subassemblies << std::endl;
    else
      std::cout << "No subassemblies found in ComputeSubassemblies!" << std::endl;
  }
}

#endif
