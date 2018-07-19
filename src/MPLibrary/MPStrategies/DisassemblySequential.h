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
    typedef typename DisassemblyMethod<MPTraits>::VID            VID;
    typedef typename DisassemblyMethod<MPTraits>::VIDPath        VIDPath;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode DisassemblyNode;
    typedef typename DisassemblyMethod<MPTraits>::Formation       Formation;
    typedef typename DisassemblyMethod<MPTraits>::Approach Approach;
    typedef typename DisassemblyMethod<MPTraits>::State State;
    typedef std::pair<Formation, std::map<Approach, bool> > AttemptEntry;

    DisassemblySequential(
        const std::map<std::string, std::pair<size_t, size_t> >& _matingSamplerLabels =
            std::map<std::string, std::pair<size_t, size_t> >(),
        const std::map<std::string, std::pair<size_t, size_t> >& _rrtSamplerLabels =
            std::map<std::string, std::pair<size_t, size_t> >(),
        const std::string _vc = "", const std::string _singleVc = "",
        const std::string _lp = "", const std::string _ex = "",
        const std::string _dm = "",
        const std::vector<std::string>& _evaluatorLabels = std::vector<std::string>());
    DisassemblySequential(XMLNode& _node);
    virtual ~DisassemblySequential() {}

    virtual void Iterate();

  protected:
    virtual DisassemblyNode* SelectExpansionNode() override;
    virtual Formation SelectSubassembly(DisassemblyNode* _q) override;
    virtual std::pair<bool, VIDPath> Expand(DisassemblyNode* _q,
                                      const Formation& _subassembly) override;

    list<DisassemblyNode*> m_nodeQueue;

    bool m_useRRT{true};

    using DisassemblyMethod<MPTraits>::m_disNodes;
};

template <typename MPTraits>
DisassemblySequential<MPTraits>::
DisassemblySequential(
    const std::map<std::string, std::pair<size_t, size_t> >& _matingSamplerLabels,
    const std::map<std::string, std::pair<size_t, size_t> >& _rrtSamplerLabels,
    const std::string _vc, const std::string _singleVc,
    const std::string _lp, const std::string _ex,
    const std::string _dm, const std::vector<std::string>& _evaluatorLabels) :
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
    std::cout << this->GetNameAndLabel() << "::Iterate()" << std::endl;

  if(m_nodeQueue.empty() && !m_disNodes.empty()) {
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
typename DisassemblySequential<MPTraits>::DisassemblyNode*
DisassemblySequential<MPTraits>::
SelectExpansionNode() {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectExpansionCfg() | "
         << "Size of candidates = " << m_nodeQueue.size() << std::endl;

  // check if first iteration
  if (m_disNodes.empty()) {
    std::vector<size_t> robotParts;
    for (size_t i = 0; i < this->m_numParts; ++i)
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
typename DisassemblyMethod<MPTraits>::Formation
DisassemblySequential<MPTraits>::
SelectSubassembly(DisassemblyNode* _q) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectSubassembly()" << std::endl;
  return Formation();
}

template <typename MPTraits>
std::pair<bool, typename DisassemblySequential<MPTraits>::VIDPath>
DisassemblySequential<MPTraits>::
Expand(DisassemblyNode* _node, const Formation& _subassembly) {
  if(this->m_debug)
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
      m_nodeQueue.push_back(newNode);
  }

  if (this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Expand with multi-part subassemblies"
         << std::endl;
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

  return std::make_pair(true, VIDPath());
}

#endif
