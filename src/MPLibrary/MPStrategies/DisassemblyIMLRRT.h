#ifndef DISASSEMBLY_IMLRRT_H_
#define DISASSEMBLY_IMLRRT_H_

#include "DisassemblyMethod.h"
#include "nonstd.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Implementation of the I-ML-RRT disassembly method.
///
///
/// This is the comparison method from our publication:
/// T. Ebinger, S. Kaden, S. Thomas, R. Andre, N. M. Amato, and U. Thomas,
/// “A general and flexible search framework for disassembly planning,”
/// in International Conference on Robotics and Automation, May 2018.
///
/// The actual paper it is from is:
/// D. T. Le, J. Cortés, and T. Siméon, “A path planning approach to
/// (dis) assembly sequencing,” in Automation Science and Engineering,
/// 2009. CASE 2009. IEEE International Conference on. IEEE, 2009,
/// pp. 286–291.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyIMLRRT : public DisassemblyMethod<MPTraits> {
  public:
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename GroupCfgType::Formation     Formation;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename GroupRoadmapType::VID       VID;
    typedef std::vector<VID>                     VIDPath;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode DisassemblyNode;
    typedef typename DisassemblyMethod<MPTraits>::Approach        Approach;
    typedef typename DisassemblyMethod<MPTraits>::State           State;

    DisassemblyIMLRRT(
        const map<string, pair<size_t, size_t> >& _matingSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const map<string, pair<size_t, size_t> >& _rrtSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const string _vc = "", const string _singleVc = "",
        const string _lp = "", const string _ex = "",
        const string _dm = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    DisassemblyIMLRRT(XMLNode& _node);
    virtual ~DisassemblyIMLRRT() {}

    virtual void Iterate();

  protected:
    virtual DisassemblyNode* SelectExpansionNode() override;
    virtual Formation SelectSubassembly(DisassemblyNode* _q) override;
    virtual pair<bool, VIDPath> Expand(DisassemblyNode* _q,
                                   const Formation& _subassembly) override;

    DisassemblyNode* m_lastNode{nullptr};
    VID m_qInitVid{0};

    Formation m_sub;

    using DisassemblyMethod<MPTraits>::m_disNodes;
    using DisassemblyMethod<MPTraits>::m_numParts;
};

template <typename MPTraits>
DisassemblyIMLRRT<MPTraits>::
DisassemblyIMLRRT(
    const map<string, pair<size_t, size_t> >& _matingSamplerLabels,
    const map<string, pair<size_t, size_t> >& _rrtSamplerLabels,
    const string _vc, const string _singleVc,
    const string _lp, const string _ex,
    const string _dm, const vector<string>& _evaluatorLabels) :
    DisassemblyMethod<MPTraits>(_matingSamplerLabels, _rrtSamplerLabels, _vc,
      _singleVc, _lp, _ex, _dm, _evaluatorLabels) {
  this->SetName("DisassemblyIMLRRT");
  this->m_keepBestRRTPathOnFailure = true;
}

template <typename MPTraits>
DisassemblyIMLRRT<MPTraits>::
DisassemblyIMLRRT(XMLNode& _node) : DisassemblyMethod<MPTraits>(_node) {
  this->SetName("DisassemblyIMLRRT");
  this->m_keepBestRRTPathOnFailure = true;
}

template <typename MPTraits>
void
DisassemblyIMLRRT<MPTraits>::
Iterate() {
  // check if first iteration
  if(m_disNodes.empty()) {
    vector<size_t> robotParts;
    for(size_t i = 0; i < m_numParts; ++i)
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
  if(m_lastNode->initialParts.empty()) {
    this->m_successful = true;
    return;
  }

  m_sub = SelectSubassembly(m_lastNode);

  if(this->m_debug)
    std::cout << "Thanh Le Expanding with sub = " << m_sub << std::endl
              << "And remaining parts = " << m_lastNode->GetCompletePartList()
              << std::endl;

  VID newVid = 0;
  const VIDPath path = this->ExpandRRTApproach(m_qInitVid, m_sub, newVid);

  //NOTE: we will usually update the cfg for RRT to expand from, but we only
  // create DT nodes if a part was actually removed.

  if(!path.empty()) {
    if(this->m_debug)
      std::cout << "Successful removal!" << std::endl;
    //Whatever VID is put in needs to be set as the next node to start from.
    std::vector<VIDPath> removingPaths = {path};

    // So that a new node keeps any other parts' progress from the RRT.
    m_lastNode->vid = newVid;

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
typename DisassemblyIMLRRT<MPTraits>::DisassemblyNode*
DisassemblyIMLRRT<MPTraits>::
SelectExpansionNode() {
  throw RunTimeException(WHERE,"Unused by this strategy, not implemented");
  return new DisassemblyNode();
}


template <typename MPTraits>
pair<bool, typename DisassemblyIMLRRT<MPTraits>::VIDPath>
DisassemblyIMLRRT<MPTraits>::
Expand(DisassemblyNode* _q, const Formation& _subassembly) {
  throw RunTimeException(WHERE,"Not implemented");
  return make_pair(false, VIDPath());
}


template <typename MPTraits>
typename DisassemblyIMLRRT<MPTraits>::Formation
DisassemblyIMLRRT<MPTraits>::
SelectSubassembly(DisassemblyNode* _q) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectSubassembly()" << std::endl;

  // check count of initialParts and return empty Formation if empty
  if(_q->initialParts.empty()) {
    std::cout << "initialParts are empty!" << std::endl;
    return Formation();
  }

  /// Return parts sequentially.
  static size_t lastIndexTried = _q->initialParts.size();
  if(lastIndexTried > 1 && lastIndexTried <= _q->initialParts.size())
    return Formation({_q->initialParts[--lastIndexTried]});
  else {
    lastIndexTried = _q->initialParts.size();
    return Formation({_q->initialParts[0]});
  }

  //Always return just a random single part from the remaining parts.
  return Formation({_q->initialParts[(LRand() % _q->initialParts.size())]});
}

#endif
