#ifndef DISASSEMBLY_THANH_LE_H_
#define DISASSEMBLY_THANH_LE_H_

#include "DisassemblyMethod.h"
#include "nonstd.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Basic serial disassembly method
///
///
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyThanhLe : public DisassemblyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::GraphType      GraphType;
    typedef typename RoadmapType::VID            VID;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode DisassemblyNode;
    typedef unsigned int                         Part;
    typedef vector<Part>                         Subassembly;

    DisassemblyThanhLe(
        const map<string, pair<size_t, size_t> >& _matingSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const map<string, pair<size_t, size_t> >& _rrtSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const string _vc = "", const string _singleVc = "",
        const string _lp = "", const string _ex = "",
        const string _dm = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    DisassemblyThanhLe(XMLNode& _node);
    virtual ~DisassemblyThanhLe() {}

    virtual void Iterate();

  protected:
    virtual DisassemblyNode* SelectExpansionNode() override;
    virtual Subassembly SelectSubassembly(DisassemblyNode* _q) override;
    virtual pair<bool, vector<CfgType>> Expand(DisassemblyNode* _q,
                                   const Subassembly& _subassembly) override;

    DisassemblyNode* m_lastNode{nullptr};
    VID m_qInitVid{0};

    Subassembly m_sub;

    using DisassemblyMethod<MPTraits>::m_disNodes;
    using DisassemblyMethod<MPTraits>::m_numParts;
    using DisassemblyMethod<MPTraits>::m_robot;
};

template <typename MPTraits>
DisassemblyThanhLe<MPTraits>::
DisassemblyThanhLe(
    const map<string, pair<size_t, size_t> >& _matingSamplerLabels,
    const map<string, pair<size_t, size_t> >& _rrtSamplerLabels,
    const string _vc, const string _singleVc,
    const string _lp, const string _ex,
    const string _dm, const vector<string>& _evaluatorLabels) :
    DisassemblyMethod<MPTraits>(_matingSamplerLabels, _rrtSamplerLabels, _vc,
      _singleVc, _lp, _ex, _dm, _evaluatorLabels) {
  this->SetName("DisassemblyThanhLe");
  this->m_keepBestRRTPathOnFailure = true;
}

template <typename MPTraits>
DisassemblyThanhLe<MPTraits>::
DisassemblyThanhLe(XMLNode& _node) : DisassemblyMethod<MPTraits>(_node) {
  this->SetName("DisassemblyThanhLe");
  this->m_keepBestRRTPathOnFailure = true;
}

template <typename MPTraits>
void
DisassemblyThanhLe<MPTraits>::
Iterate() {
  // check if first iteration
  if (m_disNodes.empty()) {
    vector<unsigned int> robotParts;
    for (unsigned int i = 0; i < m_numParts; ++i)
      robotParts.push_back(i);

    DisassemblyNode node;
    node.initialParts = robotParts;
    node.vid = this->m_rootVid;
    m_disNodes.push_back(node);
    m_lastNode = &m_disNodes.back();
    this->m_rootNode = &m_disNodes.back();

    m_lastNode = this->m_rootNode;
    m_qInitVid = this->m_rootVid;
  }

  // check for initialParts of last node; if empty, strategy was successful
  if (m_lastNode->initialParts.empty()) {
    this->m_successful = true;
    return;
  }

  m_sub = SelectSubassembly(m_lastNode);

  if(this->m_debug)
    std::cout << "Thanh Le Expanding with sub = " << m_sub << std::endl
              << "And remaining parts = " << m_lastNode->GetCompletePartList()
              << std::endl;

  VID newVid = 0;
  const vector<CfgType> path =
      this->ExpandRRTApproach(m_qInitVid, m_sub, newVid);

  //NOTE: we will usually update the cfg for RRT to expand from, but we only
  // create DT nodes if a part was actually removed.

  if(!path.empty()) {
    if(this->m_debug)
      std::cout << "Successful removal!" << std::endl;
    //Whatever VID is put in needs to be set as the next node to start from.
    std::vector< std::vector<CfgType> > removingPaths = {path};

    /// TODO: See if this works:
    /// here I should set the m_lastNode.vid = newVid, then when the new node
    /// is generated, this will keep the progress!
    m_lastNode->vid = newVid; // So that the new node keeps any other parts' progress from the rrt.


    m_lastNode = this->GenerateNode(m_lastNode, m_sub, removingPaths, false);
    m_qInitVid = m_lastNode->vid;
  }
  else if(newVid != 0) {
    if(this->m_debug)
      std::cout << "No removal, but updated the current vid to expand from to "
                << newVid << std::endl;
    m_qInitVid = newVid;
    //While we could update m_lastNode's vid to this as well (or instead of),
    // there's not much of a reason to do so. In effect, it shouldn't matter.
  }
  else if(this->m_debug)
    std::cout << "RRT didn't return a new VID and also didn't remove a part!!!"
              << std::endl;
}

template <typename MPTraits>
typename DisassemblyThanhLe<MPTraits>::DisassemblyNode*
DisassemblyThanhLe<MPTraits>::
SelectExpansionNode() {
  throw RunTimeException(WHERE,"Unused by this strategy, not implemented");
  return new DisassemblyNode();
}


template <typename MPTraits>
pair<bool, vector<typename DisassemblyThanhLe<MPTraits>::CfgType>>
DisassemblyThanhLe<MPTraits>::
Expand(DisassemblyNode* _q, const Subassembly& _subassembly) {
  throw RunTimeException(WHERE,"Not implemented");
  return pair<bool, vector<CfgType>>(false, vector<CfgType>());
}


template <typename MPTraits>
vector<unsigned int>
DisassemblyThanhLe<MPTraits>::
SelectSubassembly(DisassemblyNode* _q) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectSubassembly()" << std::endl;

  // check count of initialParts and return empty Subassembly if empty
  if (_q->initialParts.empty()) {
    std::cout << "initialParts are empty!" << std::endl;
    return Subassembly();
  }

  ///@TODO: Try returning parts sequentially!!! Also fix node generation so that
  ///       it isn't resetting some of the partial rrt progress (not sure exactly
  ///       why it even is though...)
  static unsigned int lastIndexTried = _q->initialParts.size();
  if(lastIndexTried > 1 && lastIndexTried <= _q->initialParts.size())
    return Subassembly({_q->initialParts[--lastIndexTried]});
  else {
    lastIndexTried = _q->initialParts.size();
    return Subassembly({_q->initialParts[0]});
  }

  //Always return just a random single part from the remaining parts.
  return Subassembly({_q->initialParts[(LRand() % _q->initialParts.size())]});
}

#endif
