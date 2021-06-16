#ifndef PMPL_DISASSEMBLY_PARALLELIZED_SAS_H_
#define PMPL_DISASSEMBLY_PARALLELIZED_SAS_H_

#include "DisassemblyMethod.h"
#include "nonstd.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This strategy is designed to use predefined subassemblies (supplied
///        from the XML file) which will be the first attempt. It is meant to
///        demonstrate the time savings possible if we parallelized the
///        disassembly of each of the subassemblies after initial removal.
///
///
/// This is a modified version of Preemtive DFS from the below publication.
/// It is modified to show the psuedo-parallel speedup described above. See
/// Table III in the paper to see the data from this method.
/// T. Ebinger, S. Kaden, S. Thomas, R. Andre, N. M. Amato, and U. Thomas,
/// “A general and flexible search framework for disassembly planning,”
/// in International Conference on Robotics and Automation, May 2018.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyParallelizedSAs : public DisassemblyMethod<MPTraits> {
  public:
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef std::vector<size_t>                  RobotFormation;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename GroupRoadmapType::VID       VID;
    typedef std::vector<VID>                     VIDPath;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode DisassemblyNode;
    typedef typename DisassemblyMethod<MPTraits>::Approach        Approach;
    typedef typename DisassemblyMethod<MPTraits>::State           State;

    DisassemblyParallelizedSAs();
    DisassemblyParallelizedSAs(XMLNode& _node);
    virtual ~DisassemblyParallelizedSAs() {}

    virtual void Iterate();

  protected:
    virtual DisassemblyNode* SelectExpansionNode() override;
    virtual RobotFormation SelectSubassembly(DisassemblyNode* _q) override;
    virtual std::pair<bool, VIDPath> Expand(DisassemblyNode* _q,
                                   const RobotFormation& _subassembly) override;

    void AppendNode(DisassemblyNode* _parent,
                    const std::vector<size_t>& _removedParts,
                    const std::vector<VIDPath>& _removingPaths,
                    const bool _isMultiPart);

    Approach m_approach = Approach::mating;

    DisassemblyNode* m_lastNode = nullptr;
    size_t m_curSubIndex = 0;

    bool m_initialMatingRemoval{true};//A flag to keep track of Iterate state

    double m_maxRobotFormationTime = 0;

    // subassembly candidates
    std::vector<RobotFormation> m_subassemblies;

    using DisassemblyMethod<MPTraits>::m_disNodes;
    using DisassemblyMethod<MPTraits>::m_numParts;
};

template <typename MPTraits>
DisassemblyParallelizedSAs<MPTraits>::
DisassemblyParallelizedSAs() {
  this->SetName("DisassemblyParallelizedSAs");
}

template <typename MPTraits>
DisassemblyParallelizedSAs<MPTraits>::
DisassemblyParallelizedSAs(XMLNode& _node) : DisassemblyMethod<MPTraits>(_node){
  this->SetName("DisassemblyParallelizedSAs");
}

template <typename MPTraits>
void
DisassemblyParallelizedSAs<MPTraits>::
Iterate() {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Iterate()" << std::endl;

  auto stats = this->GetStatClass();
  const std::string tempClockName = "Temporary timer for each SA removal";
  const std::string initialRemovalClockName =
                                            "Initial subassembly removal time";

  if (!m_lastNode && !m_disNodes.empty()) { // check if disassembly is complete
    std::cout << std::endl << std::endl << "Disassembling complete!" << std::endl << std::endl;
    this->m_successful = true;
    const double initialTime = stats->GetSeconds(initialRemovalClockName);
    stats->SetStat("Total time for in-parallel removal",
                   initialTime + m_maxRobotFormationTime);
    return;
  }

  DisassemblyNode* node = SelectExpansionNode();

  m_approach = Approach::mating;
  //NOTE: for now, this strategy has no RRT support.

  if(m_initialMatingRemoval) {
    stats->StartClock(initialRemovalClockName);
    if(this->m_debug && node != this->m_rootNode)
      std::cout << "Warning! Initial node to remove assemblies from isn't the"
                   " root node!!!" << std::endl;
    m_initialMatingRemoval = false;//Only attempt once.
    //Pass the node (will be the root) and its initial parts as the subassembly
    // so that all parts are attempted.
    for(RobotFormation& sub : this->m_predefinedSubassemblies) {
      if(this->m_debug)
        std::cout << "Attempting initial sub = " << sub << std::endl;
      std::pair<bool, VIDPath> result = Expand(node, sub);
      if (result.first)
        AppendNode(node, sub, {result.second}, sub.size() > 1);
      else {
        //TODO: To make this strategy most generic, allow RRT to be attempted
        // if there's a failure. For the ICRA test, we don't need this though,
        // so printing a message will be the most helpful.
        if(this->m_debug)
          std::cout << "Warning! Removing the subassembly " << sub
                    << " failed in the initial step!!!" << std::endl
                    << std::endl;
      }
      node = SelectExpansionNode();//Update to the newly generated node, repeat.
    }
    stats->StopClock(initialRemovalClockName);
  }
  else {
    //Not on the initial SA removal step, so attempt each subassembly
    // and time how long it takes for each one, recording the max time.
    for(size_t i = 0; i < this->m_predefinedSubassemblies.size(); i++) {
      RobotFormation sub = this->m_predefinedSubassemblies.at(i);
      //Create new clock so we don't accumulate each time:
      const std::string thisClockName = tempClockName + std::to_string(i);
      if(this->m_debug)
        std::cout << "Disassembling subassembly " << sub << std::endl;
      stats->StartClock(thisClockName);
      while(!sub.empty()) {
        std::vector<size_t> removedParts;
        std::vector<VIDPath> removalPaths;
        for(size_t j = 0; j < sub.size(); ++j) {
          const size_t part = sub.at(j);
          const RobotFormation partSub({part});
          std::pair<bool, VIDPath> result = Expand(node, partSub);
          if (result.first) {
            sub.erase(remove(sub.begin(), sub.end(), part), sub.end());
            --j;//decrement so that we don't skip parts when removing
            removedParts.push_back(part);
            removalPaths.push_back(result.second);
          }
          else if(this->m_debug)
            std::cout << "Warning! Removing part from subassembly failed!"
                      << std::endl;
        }

        if(this->m_debug)
          std::cout << "sub = " << sub << std::endl;

        if(!removedParts.empty())
          AppendNode(node, removedParts, removalPaths, false);
        node = SelectExpansionNode();//Update to the newly generated node.
      }
      stats->StopClock(thisClockName);
      const double timeTaken = stats->GetSeconds(thisClockName);
      if(timeTaken > m_maxRobotFormationTime)
        m_maxRobotFormationTime = timeTaken;
    }
  }
}

template <typename MPTraits>
typename DisassemblyParallelizedSAs<MPTraits>::DisassemblyNode*
DisassemblyParallelizedSAs<MPTraits>::
SelectExpansionNode() {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectExpansionCfg()" << std::endl;

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
    m_curSubIndex = 0;
    m_subassemblies.clear();

    //Reset the flag for initial mating removal attempt:
    m_initialMatingRemoval = true;

    //Do some error checking (that part numbers are valid and #parts in the
    // predefined subs are exactly the same as #parts in problem):
    std::vector<size_t> initParts = this->m_rootNode->GetCompletePartList();
    size_t numberOfParts = 0;
    for(auto& sub : this->m_predefinedSubassemblies) {
      numberOfParts += sub.size();
      for(auto part : sub)
        if(part >= initParts.size())
          throw RunTimeException(WHERE, "A part in a subassembly was larger "
                                        "than the number of parts defined!");
    }
    if(numberOfParts != initParts.size())
      throw RunTimeException(WHERE, "There are too many or too few parts "
                                    "predefined for subassemblies!");

    return this->m_rootNode;
  }

  return m_lastNode;
}

template <typename MPTraits>
typename DisassemblyParallelizedSAs<MPTraits>::RobotFormation
DisassemblyParallelizedSAs<MPTraits>::
SelectSubassembly(DisassemblyNode* _q) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectSubassembly()" << std::endl;

  return RobotFormation();
}

template <typename MPTraits>
std::pair<bool, typename DisassemblyParallelizedSAs<MPTraits>::VIDPath>
DisassemblyParallelizedSAs<MPTraits>::
Expand(DisassemblyNode* _q, const RobotFormation& _subassembly) {
  if (_subassembly.empty())
    return std::make_pair(false, VIDPath());
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Expand with RobotFormation: "
         << _subassembly << std::endl;

  VID newVID;
  VIDPath path;
  if (m_approach == Approach::rrt) // choose between RRT and mating approach
    path = this->ExpandRRTApproach(_q->vid, _subassembly, newVID);
  else
    path = this->ExpandMatingApproach(_q->vid, _subassembly, newVID);

  if (!path.empty())
    return std::make_pair(true, path);
  else
    return std::make_pair(false, path);
}

template <typename MPTraits>
void
DisassemblyParallelizedSAs<MPTraits>::
AppendNode(DisassemblyNode* _parent, const std::vector<size_t>& _removedParts,
           const std::vector<VIDPath>& _removingPaths,
           const bool _isMultiPart) {

  // update node to new state
  m_lastNode = this->GenerateNode(_parent, _removedParts, _removingPaths,
                                  _isMultiPart);
  //If there are no remaining parts, pop this off to trigger success.
  if(m_lastNode->GetCompletePartList().empty())
    m_lastNode = nullptr;
}

#endif
