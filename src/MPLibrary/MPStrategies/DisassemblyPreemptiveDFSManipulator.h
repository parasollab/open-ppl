#ifndef DISASSEMBLY_PREEMPTIVE_DFS_MANIPULATOR_H_
#define DISASSEMBLY_PREEMPTIVE_DFS_MANIPULATOR_H_

#include "DisassemblyMethod.h"
#include "nonstd.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is the Preemptive DFS technique, with a few minor variants
///        built in as options.
///
///
/// This is the modified (to use manipulators) method Preemptive DFS from:
/// T. Ebinger, S. Kaden, S. Thomas, R. Andre, N. M. Amato, and U. Thomas,
/// “A general and flexible search framework for disassembly planning,”
/// in International Conference on Robotics and Automation, May 2018.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyPreemptiveDFSManipulator : public DisassemblyMethod<MPTraits> {

  public:
    typedef typename MPTraits::MPSolution        MPSolution;
    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename GroupCfgType::Formation     Formation;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename GroupRoadmapType::VID       VID;
    typedef std::vector<VID>                     VIDPath;
    typedef typename DisassemblyMethod<MPTraits>::DisassemblyNode DisassemblyNode;
    typedef typename DisassemblyNode::MotionInformation MotionInformation;
    typedef typename DisassemblyMethod<MPTraits>::Approach        Approach;
    typedef typename DisassemblyMethod<MPTraits>::State           State;
    typedef typename DisassemblyMethod<MPTraits>::DGNodePath      DGNodePath;

    struct EffectorPlacementInfo {
        EffectorPlacementInfo(const size_t _part = 0, const size_t _body = 0,
                              const size_t _polygon = 0)
                                : partIndex(_part), bodyIndex(_body), polygonIndex(_polygon) {}
        size_t partIndex;    // The loop's index of the part used for placement.
        size_t bodyIndex;    // The loop's index of the body used for placement.
        size_t polygonIndex; // The loop's index of the polygon used for placement.
    };

    typedef std::pair<VIDPath, EffectorPlacementInfo> EffectorPathInfo;

    DisassemblyPreemptiveDFSManipulator(
        const map<string, pair<size_t, size_t> >& _matingSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const map<string, pair<size_t, size_t> >& _rrtSamplerLabels =
            map<string, pair<size_t, size_t> >(),
        const string _vc = "", const string _singleVc = "",
        const string _lp = "", const string _ex = "",
        const string _dm = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    DisassemblyPreemptiveDFSManipulator(XMLNode& _node);
    virtual ~DisassemblyPreemptiveDFSManipulator() {}

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



    // Effector placement entry point function.
    void PlaceEndEffector();

    // Writes all effector/manipulator-specific outputs.
    void WriteAllOutputs(const GroupPath<MPTraits>& _path,
                         const size_t _pathIndex);

    // Places the effector along a vector of cfgs to match.
    VIDPath PlaceEffectorAlongRemoval(
               const Formation& _removedFormation,
               const std::vector<GroupCfgType>& _removalCfgs,
               GroupRoadmapType* const _effectorMap,
               EffectorPlacementInfo& _placementInfo = EffectorPlacementInfo());

    // Finds a valid contact point for a single cfg, or checks if the one
    // provided is valid for the cfg.
    size_t AttemptEffectorPlacement(Body* const _body,
                                    GroupCfgType& _manipulatorCfg,
                                    const size_t _lastCandidateAttempted = 0,
                                    const bool _isFirstInPath = false);

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


    size_t m_numContactCandidates{100};

    // The solution and roadmap for the assembly with the end effector as a free
    // body.
    MPSolution* m_effectorSolution{nullptr};
    GroupRoadmapType* m_effectorRoadmap{nullptr};
    GroupRoadmapType* m_failedEffectorPlacementsRoadmap{nullptr};

    // The solution and roadmap for the assembly with the entire manipulator.
    MPSolution* m_manipulatorSolution{nullptr};
    GroupRoadmapType* m_manipulatorRoadmap{nullptr};

    // This flag will keep local planning between a removal and the next
    // placement of the end effector from failing a removal, since a straight
    // line plan is not always feasible for that.
    bool m_skipPlacementMotionAfterRemoval{true};

    // The LP to move the effector from removal position to the next placement.
    std::string m_effectorPlacementLP;

    using DisassemblyMethod<MPTraits>::m_disNodes;
    using DisassemblyMethod<MPTraits>::m_numParts;
};

template <typename MPTraits>
DisassemblyPreemptiveDFSManipulator<MPTraits>::
DisassemblyPreemptiveDFSManipulator(
    const map<string, pair<size_t, size_t> >& _matingSamplerLabels,
    const map<string, pair<size_t, size_t> >& _rrtSamplerLabels,
    const string _vc, const string _singleVc,
    const string _lp, const string _ex,
    const string _dm, const vector<string>& _evaluatorLabels) :
    DisassemblyMethod<MPTraits>(_matingSamplerLabels, _rrtSamplerLabels, _vc,
      _singleVc, _lp, _ex, _dm, _evaluatorLabels) {
  this->SetName("DisassemblyPreemptiveDFSManipulator");
}

template <typename MPTraits>
DisassemblyPreemptiveDFSManipulator<MPTraits>::
DisassemblyPreemptiveDFSManipulator(XMLNode& _node) : DisassemblyMethod<MPTraits>(_node) {
  this->SetName("DisassemblyPreemptiveDFSManipulator");

  m_noSubassemblies = _node.Read("noSubassemblies", false, m_noSubassemblies,
                                 "Use multi-part subassemblies or not.");

  m_useParallelRemoval = _node.Read("useParallelRemoval", false,
                                    m_useParallelRemoval,
                                   "Use (pseudo) in-parallel removals.");
  if(m_useParallelRemoval)
    throw RunTimeException(WHERE, "Parallel removal is not currently supported "
                                  "for groups.");

  m_skipPlacementMotionAfterRemoval = _node.Read(
    "skipPlacementMotionAfterRemoval", false, m_skipPlacementMotionAfterRemoval,
    "Skip the straight line plan between effector's removal position and next "
    "placement.");

  // This is required if m_skipPlacementMotionAfterRemoval is false.
  m_effectorPlacementLP = _node.Read("effectorPlacementLP",
          !m_skipPlacementMotionAfterRemoval, "", "The LP for placing the end "
          "effector if we are not skipping that motion.");

  m_numContactCandidates = _node.Read("numContactCandidates", false,
          m_numContactCandidates, (size_t)0, std::numeric_limits<size_t>::max(),
          "Number of programmatic contact candidates to consider.");
}

template <typename MPTraits>
void
DisassemblyPreemptiveDFSManipulator<MPTraits>::
Iterate() {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Iterate()" << std::endl;

  if(!m_lastNode && !m_disNodes.empty()) { // check if disassembly is complete
    std::cout << std::endl << std::endl << std::endl
        << "------------------------------------------------------------"
        << std::endl << "Sequence found! Placing the manipulator." << std::endl
        << "------------------------------------------------------------"
        << std::endl << std::endl << std::endl;
    this->m_successful = true; // So we stop iterating after placing effector.
    // This no longer means strategy success, it just means we have some motions
    // to match with the manipulator.
    PlaceEndEffector();
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
      for(const size_t id : completePartList) {
        subassembly = {id};
        std::pair<bool,VIDPath> result = Expand(node, subassembly);
        if(result.first) {
          removedParts.push_back(id);
          removingPaths.push_back(result.second);
          if(!m_useParallelRemoval) // This will make it so we put off the part
            break;         // to the side, then come back later for other parts
        }
      }
    }

    if(removedParts.empty()) {
      // Mating vector multiple parts:
      m_state = State::multiPart;
      ComputeSubassemblies(node);
      for(auto &sub : m_subassemblies) {
        if(sub.size() == completePartList.size() && node->initialParts.empty())
          continue; // step over complete subassemblies apart from init position
        std::pair<bool, VIDPath > result = Expand(node, sub);
        if(result.first) {
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
    for(const size_t id : completePartList) {
      subassembly = Formation({id});
      std::pair<bool, VIDPath > result = Expand(node, subassembly);
      if(result.first) {
        removedParts.push_back(id);
        removingPaths.push_back(result.second);
        break; // We only do "simultaneous" removals for mating single parts.
      }
    }

    // RRT multi-part:
    if(removedParts.empty()) {
      m_state = State::multiPart;
      ComputeSubassemblies(node);
      for(const Formation& sub : m_subassemblies) {
        std::pair<bool, VIDPath > result = Expand(node, sub);
        if(result.first) {
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
typename DisassemblyPreemptiveDFSManipulator<MPTraits>::DisassemblyNode*
DisassemblyPreemptiveDFSManipulator<MPTraits>::
SelectExpansionNode() {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::SelectExpansionCfg()"
              << std::endl;

  // check if first iteration
  if(m_disNodes.empty()) {
    std::vector<size_t> robotParts;
    for(size_t i = 0; i < m_numParts; ++i)
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
typename DisassemblyPreemptiveDFSManipulator<MPTraits>::Formation
DisassemblyPreemptiveDFSManipulator<MPTraits>::
SelectSubassembly(DisassemblyNode* _q) {
  throw RunTimeException(WHERE, "Unused by this strategy");
  return Formation();
}


template <typename MPTraits>
std::pair<bool, typename DisassemblyPreemptiveDFSManipulator<MPTraits>::VIDPath >
DisassemblyPreemptiveDFSManipulator<MPTraits>::
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
  if(m_approach == Approach::rrt)
    path = this->ExpandRRTApproach(_q->vid, _subassembly, newVID, _q);
  else
    path = this->ExpandMatingApproach(_q->vid, _subassembly, newVID, _q);

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
DisassemblyPreemptiveDFSManipulator<MPTraits>::
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
DisassemblyPreemptiveDFSManipulator<MPTraits>::
ComputeSubassemblies(DisassemblyNode* _node) {
  m_subassemblies.clear();
  if(!m_noSubassemblies) {
    //First get initial part subassemblies:
    m_subassemblies =
                  this->GenerateSubassemblies(_node->vid, _node->initialParts);
    //Get all sub-subassemblies that could be used from usedSubassemblies:
    for(const auto &usedSub : _node->usedSubassemblies) {
      vector<Formation> subs =
                              this->GenerateSubassemblies(_node->vid, usedSub);
      if(!subs.empty()) {
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


template <typename MPTraits>
void
DisassemblyPreemptiveDFSManipulator<MPTraits>::
PlaceEndEffector() {
  RobotGroup* const effectorGroup = this->GetGroupTask()->GetEndEffectorGroup();
  auto const env = this->GetEnvironment();

  // Construct the manipulator group map that is stored locally to the class:
  m_effectorSolution = new MPSolution(effectorGroup);
  m_effectorRoadmap = new GroupRoadmapType(effectorGroup, m_effectorSolution);
  m_failedEffectorPlacementsRoadmap = new GroupRoadmapType(effectorGroup, m_effectorSolution);
  GroupPath<MPTraits> path(m_effectorRoadmap);

  const size_t effectorRobotIndex = effectorGroup->Size() - 1;

  GroupRoadmapType* const groupMap = this->GetGroupRoadmap();
  auto const lp = this->GetLocalPlanner(this->m_lpLabel);

  // Iterates through the DG and builds paths with VIDs and DG nodes, which then
  // will be iterated through to place the effector.
  this->GetSequenceStats();

  // m_pathVids has the vids that are connected through the roadmap and
  // m_nodePaths has the DG node pointer and the index in m_pathVids where
  // the node starts. Now iterate through the DG nodes so that we can attempt
  // the end effector placement at the start of each removal:

  //For easier generalization later, pretend there can be multiple path indices
  // we want to attempt:
  std::vector<size_t> pathIndexes = {0};
  for(const size_t pathIndex : pathIndexes) {
    const DGNodePath& nodePath = this->m_nodePaths.at(pathIndex);
    VID lastCfgVID = std::numeric_limits<size_t>::max();
    GroupCfgType effectorGroupCfg(m_effectorRoadmap);

    // Go through each DG node (each representing one removal path).
    // Note the last one is excluded since it doesn't represent motion, just the
    // final state of the disassembly.
    for(size_t nodeIndex = 0; nodeIndex < nodePath.size()-1; ++nodeIndex) {
      DisassemblyNode* const currNode = nodePath.at(nodeIndex).second;

      // Loop until we successfully remove this part using an end effector, OR
      // we run out of possibilities.
      bool removalSuccess = false;

      while(!removalSuccess) {
        VIDPath removalPath;
        for(size_t motionIndex = 0;
              motionIndex < currNode->formationMotions.size(); ++motionIndex) {

          const MotionInformation& motionInfo = currNode->formationMotions[motionIndex];
          const Formation& movedParts = motionInfo.movedFormation;
          VIDPath motionVIDs = motionInfo.motionVIDs;

          // Remove the last vid from the last motion, since it will be the "set
          // aside" vid, which needs special handling after successful removal.
          bool lastMotion;
          VID setAsideVID = std::numeric_limits<VID>::max();
          if(&motionInfo == &currNode->formationMotions.back()) {
            setAsideVID = motionVIDs.back();
            lastMotion = true;
            motionVIDs.pop_back();
          }
          else
            lastMotion = false;

          std::vector<GroupCfgType> removalCfgs;
          for(const VID currVID : motionVIDs)
            removalCfgs.push_back(groupMap->GetVertex(currVID));

          // Note the vids in removalPath are wrt the roadmap passed in.
          EffectorPlacementInfo placementInfo;
          removalPath = PlaceEffectorAlongRemoval(movedParts, removalCfgs,
                                              m_effectorRoadmap, placementInfo);
          const bool motionSuccess = removalPath.size() > 1;

          // If we couldn't find a contact for the first vid, handle replanning.
          if(!motionSuccess) {
            // Failure condition for now (replanning is needed). Output all
            // the data we've gathered so far.
            WriteAllOutputs(path, pathIndex);
            this->Finalize();
            throw RunTimeException(WHERE) << "No valid effector placement "
                "could be found! This case is not yet properly handled, as some"
                " amount of replanning is necessary. Wrote all outputs.";
          }

          // Handle the case of connecting the edges BETWEEN removals and motion
          // infos, so whether teleporting or planning for the effector's
          // placement. Only the effector will be moving in this situation.
          if(nodeIndex > 0) {
            if(m_skipPlacementMotionAfterRemoval) {
              // Skip local planning and teleport end effector
              GroupLPOutput<MPTraits> lpOutput(m_effectorRoadmap);
              lpOutput.SetActiveRobots({effectorRobotIndex});
              lpOutput.SetIndividualEdges({effectorRobotIndex});
              lpOutput.SetEdgeWeights(1);
              lpOutput.SetSkipEdge();

              if(this->m_debug)
                std::cout << "Adding inter-motion edge (" << lastCfgVID << ", "
                          << removalPath.front() << ")" << std::endl;
              m_effectorRoadmap->AddEdge(lastCfgVID, removalPath.front(),
                                         lpOutput.m_edge);
            }
            else {
              // Perform local planning to place the end effector.
              auto effectorLP = this->GetLocalPlanner(m_effectorPlacementLP);
              const Formation effector({effectorRobotIndex});

              // Note: I'm copying this because for some reason the reference to
              // the cfg was having its group roadmap changed at some point, and
              // I couldn't track it down since it doesn't seem to affect any
              // other code than this region.
              const GroupCfgType fromCfg = m_effectorRoadmap->GetVertex(lastCfgVID);

              bool connected = false;
              while(!connected and placementInfo.partIndex < movedParts.size()){

                GroupLPOutput<MPTraits> lpOutput(m_effectorRoadmap);

                const GroupCfgType& toCfg = m_effectorRoadmap->GetVertex(removalPath.front());
                GroupCfgType col(m_effectorRoadmap);

                if(this->m_debug)
                  std::cout << "Doing effector placement local plan between "
                            << fromCfg.PrettyPrint(4) << " and "
                            << toCfg.PrettyPrint(4) << std::endl << std::endl;

                connected = effectorLP->IsConnected(fromCfg, toCfg, col,
                     &lpOutput, env->GetPositionRes(), env->GetOrientationRes(),
                     true, true, effector);
                if(connected) {
                  // The moving robots are the removed parts (already set) along
                  // with the manipulator.
                  lpOutput.SetLPLabel(m_effectorPlacementLP);
                  lpOutput.SetEdgeWeights(1);
                  lpOutput.SetActiveRobots(effector);
                  lpOutput.SetIndividualEdges(effector);

                  const std::vector<GroupCfgType> lpPath = lpOutput.m_path;
                  lpOutput.m_path.clear(); // So we can safely use lpOutput for each edge.

                  // Add all of the path cfgs the LP found to the roadmap and to
                  // the removalPath. We need to connect lastCfgVID -> effector
                  // placement VIDs -> removalPath.front()
                  std::vector<VID> effectorPlacementVIDs;
                  VID prevVID = lastCfgVID;
                  for(const GroupCfgType& lpCfg : lpPath) {
                    const VID newVID = m_effectorRoadmap->AddVertex(lpCfg);

                    if(newVID == 21)
                      std::cout << "Break here! Should be the start of the bad path!" << std::endl;

                    // If not a new vid, skip it because this can validly happen
                    // for the first one (I think that's the only one that can).
                    if(newVID != prevVID) {
                      // Add cfg to unified roadmap, and the needed edge to it.
                      effectorPlacementVIDs.push_back(newVID);
                      m_effectorRoadmap->AddEdge(prevVID,
                                                 effectorPlacementVIDs.back(),
                                                 lpOutput.m_edge);

                      // Important to only do this after the edge addition.
                      prevVID = newVID;
                    }
                  }

                  // Now we need to connect the last LP cfg to the start of the
                  // just found removal path; checking that the VIDs are unique.
                  if(prevVID != removalPath.front())
                    m_effectorRoadmap->AddEdge(prevVID, removalPath.front(),
                                               lpOutput.m_edge);

                  // If the end of the LP and the start of the next path have
                  // the same VIDs, remove it from the end of the effector path.
                  if(effectorPlacementVIDs.back() == removalPath.front())
                    effectorPlacementVIDs.pop_back();

                  if(this->m_debug)
                    std::cout << "Added inter-motion effector edges ("
                              << lastCfgVID << ", ";
                    if(!effectorPlacementVIDs.empty())
                      std::cout << effectorPlacementVIDs << ", ";
                    std::cout << removalPath.front() << ")." << std::endl;

                  // Add the new LP VID(s) to the start of removalPath, since
                  // it's the first step of the next removal. A little inefficient.
                  if(!effectorPlacementVIDs.empty())
                    removalPath.insert(removalPath.begin(),
                                       effectorPlacementVIDs.begin(),
                                       effectorPlacementVIDs.end());
                }
                else {
                  // Increment the polygon to start from the next loop state,
                  // then retry the placement.
                  ++placementInfo.polygonIndex;
                  removalPath = PlaceEffectorAlongRemoval(movedParts,
                                 removalCfgs, m_effectorRoadmap, placementInfo);

                  if(this->m_debug) {
                    if(!removalPath.empty()) {
                      std::cout << "Found new removal path after effector "
                                   "placement LP failed" << std::endl;
                    }
                    else {
                      WriteAllOutputs(path, pathIndex);
                      throw RunTimeException(WHERE) << "Couldn't find new "
                           "removal path for failed end effector placement LP.";
                    }
                  }
                }
              } // End while !connected and validPart.

              // Check whether we were successful or not.
              if(placementInfo.partIndex >= movedParts.size()) {
                WriteAllOutputs(path, pathIndex);
                throw RunTimeException(WHERE) << "Couldn't successfully local "
                                    "plan for the next end effector placement.";
              }
            }
          }

          // Set this after doing the effector placing, since we were connecting
          // the last removal to the start of the one represented in removalPath
          lastCfgVID = removalPath.back();

          // Set aside the part just removed, if the removal is now completed:
          if(motionSuccess and lastMotion) {
            Formation activeRobots = movedParts;
            activeRobots.push_back(effectorRobotIndex);
            // We need to create an edge so that the part "teleports" after removal.
            GroupLPOutput<MPTraits> lpOutput(m_effectorRoadmap);
            lpOutput.SetEdgeWeights(1);
            lpOutput.SetSkipEdge();
            lpOutput.SetActiveRobots(activeRobots);
            lpOutput.SetIndividualEdges(activeRobots);

            const GroupCfgType& setAsideCfg = groupMap->GetVertex(setAsideVID);
            // Set the dofs from the original roadmap cfg, and then set the
            // 0-cfg for the effector.
            GroupCfgType setAsideWithEffector(m_effectorRoadmap);
            setAsideWithEffector.OverwriteDofsForRobots(setAsideCfg,
                                                        setAsideCfg.GetRobots());

            // Put the end effector where it was when the part was marked as
            // removed.
            const GroupCfgType& lastEffectorPosition =
                               m_effectorRoadmap->GetVertex(removalPath.back());
            setAsideWithEffector.OverwriteDofsForRobots(lastEffectorPosition,
                                               Formation({effectorRobotIndex}));

            const VID setAsideNewVID =
                             m_effectorRoadmap->AddVertex(setAsideWithEffector);
            if(this->m_debug)
              std::cout << "Added set aside vid " << setAsideNewVID
                        << " to roadmap" << std::endl;

            if(this->m_debug)
              std::cout << "Adding set aside edge (" << lastCfgVID << ", "
                        << setAsideNewVID << ")" << std::endl;
            m_effectorRoadmap->AddEdge(lastCfgVID, setAsideNewVID,
                                       lpOutput.m_edge);
            removalPath.push_back(setAsideNewVID);

            lastCfgVID = setAsideNewVID;

            removalSuccess = true;
          }

          if(this->m_debug and !path.VIDs().empty()) {
            // Check that all the edges are there correctly.
            VID fromVID = path.VIDs().back();
            for(const VID toVID : removalPath) {
              if(!m_effectorRoadmap->IsEdge(fromVID, toVID)) {
                throw RunTimeException(WHERE) << "Edge (" << fromVID << ", "
                    << toVID << ") was not found in the effector's roadmap "
                    << "before adding removalPath to Path" << std::endl;
              }
              fromVID = toVID;
            }
          }

          // TODO I'm pretty sure this will be right, all possibilities for
          // failure should have occurred/been handled before this point.
          path += removalPath;

        } // End for all Motion Infos

      } // End while removal success not achieved

      if(!removalSuccess) {
        // Failure condition for now (replanning is needed). Output all
        // the data we've gathered so far.
        WriteAllOutputs(path, pathIndex);
        this->Finalize(); // Output all disassembly info.
        throw RunTimeException(WHERE) << "No valid effector placement could be "
            << "found for removing formation "
            << currNode->formationMotions.back().movedFormation
            << std::endl << "This case is not yet properly handled, as some"
            " amount of replanning is necessary. Wrote all outputs.";
      }
    }

    WriteAllOutputs(path, pathIndex);
  }
}


template <typename MPTraits>
void
DisassemblyPreemptiveDFSManipulator<MPTraits>::
WriteAllOutputs(const GroupPath<MPTraits>& _path, const size_t _pathIndex) {
  m_effectorRoadmap->Write(this->GetBaseFilename() +
                           "Effector.map", this->GetEnvironment());

  // Also print the failed effector placements if debugging.
  if(this->m_debug)
    m_failedEffectorPlacementsRoadmap->Write(
                        "FailedEffectorPlacements.map", this->GetEnvironment());

  // Generate the intermediates, and output the cfgs to a path file:
  const string filename = this->GetBaseFilename() + "Effector." +
                          std::to_string(_pathIndex) + ".path";

  // Note, we are forcing sl to be the LP because it will try to use the RRT LP
  // again which won't show us the intermediates.
  const std::vector<GroupCfgType> fullCfgs =
                                     _path.FullCfgs(this->GetMPLibrary(), "sl");
  WritePath(filename, fullCfgs);
}


template <typename MPTraits>
typename DisassemblyPreemptiveDFSManipulator<MPTraits>::VIDPath
DisassemblyPreemptiveDFSManipulator<MPTraits>::
PlaceEffectorAlongRemoval(const Formation& _removedFormation,
                          const std::vector<GroupCfgType>& _removalCfgs,
                          GroupRoadmapType* const _effectorMap,
                          EffectorPlacementInfo& _placementInfo) {
  // This function will find an effector placement, which will be fixed
  // for all configurations provided in _removalCfgs. The first cfg provided
  // have its polygons iterated over in order to find a placement, then each
  // other cfg will try that placement. If a placement is not possible for a
  // later cfg, the first one is returned to in order to find a new one. If the
  // first cfg runs out of polygons, failure is declared and an empty vector is
  // returned.

  auto const lp = this->GetLocalPlanner(this->m_lpLabel);
  auto const env = this->GetEnvironment();

  const size_t effectorRobotIndex = _effectorMap->GetNumRobots() - 1;
  const std::vector<double> effectorZero({0., 0., 0., 0., 0., 0.});

  GroupCfgType lastCfg;
  VID lastCfgVID = std::numeric_limits<VID>::max();

  // Note: Parallel removal is NOT happening! (This is also enforced
  // and required elsewhere in this strategy). This means that
  // if removedParts.size() is > 1, then we definitely have a subassembly.
  // Effectively, this just means that for each node we only need one
  // valid contact point.
  // Note that if we increment the partIndex, we reset the bodyIndex and polygonIndex.
  for(size_t& partIndex = _placementInfo.partIndex;
          partIndex < _removedFormation.size(); ++partIndex,
          _placementInfo.bodyIndex = 0, _placementInfo.polygonIndex = 0) {
    const size_t part = _removedFormation[partIndex];
    MultiBody* const mb = _effectorMap->GetGroup()->GetRobot(part)->GetMultiBody();

    // Note that if we increment the bodyNum, we reset the polygonIndex
    for(size_t& bodyNum = _placementInfo.bodyIndex; bodyNum < mb->GetNumBodies();
            ++bodyNum, _placementInfo.polygonIndex = 0) {
      Body* const body = mb->GetBody(bodyNum);

      for(size_t& currentPolygon = _placementInfo.polygonIndex;
          currentPolygon < m_numContactCandidates; ++currentPolygon) {

        VIDPath removalPath;

        bool returnToStartOfRemoval = false;
        bool isFirst = true; // To say if it's first in the removal path.
        for(const GroupCfgType& currCfg : _removalCfgs) {
          if(returnToStartOfRemoval)
            break;

          // Set the dofs from the original roadmap cfg, and then set the 0-cfg
          // for the effector.
          GroupCfgType effectorGroupCfg(_effectorMap);
          effectorGroupCfg.OverwriteDofsForRobots(currCfg, currCfg.GetRobots());
          effectorGroupCfg.OverwriteDofsForRobots(effectorZero,
                                                  {effectorRobotIndex});
          effectorGroupCfg.ConfigureRobot(); // So the bodies are set up.

          if(this->m_debug)
            std::cout << "Attempting effector placement on part " << part
                      << " with assembly cfg = " << currCfg.PrettyPrint(4)
                      << std::endl;

          // Finally, attempt the end effector placement.
          const size_t polygonFound = AttemptEffectorPlacement(
                                 body, effectorGroupCfg, currentPolygon, false);

          if(polygonFound != std::numeric_limits<size_t>::max()) {
            if(this->m_debug)
              std::cout << "Placed the effector successfully for body "
                        << bodyNum << " for removed part " << part
                        << " of removed formation " << _removedFormation
                        << " of the removal path." << std::endl
                        << "Effector group cfg = "
                        << effectorGroupCfg.PrettyPrint(4) << std::endl;
          }
          else { // Must return to start and try next part/body/polygon
            returnToStartOfRemoval = true;
            break;
          }

          // Add to roadmap and do some local planning for the end effector.
          const VID newVID = m_effectorRoadmap->AddVertex(effectorGroupCfg);
          if(this->m_debug)
            std::cout << "Added vid " << newVID << " to roadmap" << std::endl;


          // It seems like this is possible when there are many motion
          // infos for one node that also uses a subassembly, going between
          // them might have a same cfg between the end of one and the start
          // of the next. This is reasonable.
          if(newVID == lastCfgVID) {
            std::cout << "Warning! Last vid added was not new! This should "
                         "not occur very often. Continuing." << std::endl;
            continue;
          }

          // Do a local plan as long as we have a previous cfg (so as long as
          // it's not the very first one we've placed the effector for).
          if(!isFirst) { // If the first, there's only one cfg so far.
            // Create the active robot formation which includes the end effector (assumed
            // to be the last robot in the effector group).
            Formation activeRobots = _removedFormation;
            activeRobots.push_back(_effectorMap->GetNumRobots() - 1);
            GroupLPOutput<MPTraits> lpOutput(m_effectorRoadmap);
            GroupCfgType col;
            if(this->m_debug)
              std::cout << "Starting local plan with effector" << std::endl;
            const bool successfulLP =
                lp->IsConnected(lastCfg, effectorGroupCfg, col, &lpOutput,
                    env->GetPositionRes(), env->GetOrientationRes(),
                    true, true, activeRobots);
            if(this->m_debug) {
              if(successfulLP)
                std::cout << "Successful local plan with effector" << std::endl;
              else
                std::cout << "Failed local plan with effector" << std::endl;
            }
            if(successfulLP) {
              removalPath.push_back(newVID);
              // The moving robots are the removed parts (already set) along with
              // the manipulator.
              lpOutput.SetLPLabel(this->m_lpLabel);
              lpOutput.SetActiveRobots(activeRobots);
              lpOutput.SetIndividualEdges(activeRobots);
              m_effectorRoadmap->AddEdge(lastCfgVID, newVID, lpOutput.m_edge);
              if(this->m_debug)
                std::cout << "Added edge (" << lastCfgVID << ", " << newVID
                          << ") to roadmap" << std::endl;

              lastCfg = effectorGroupCfg;
              lastCfgVID = newVID;
            }
            else {
              // Failed local plan, start from next polygon next time.
              returnToStartOfRemoval = true;

              if(this->m_debug) {
                const VID fromVID = m_failedEffectorPlacementsRoadmap->AddVertex(lastCfg);
                const VID colVID = m_failedEffectorPlacementsRoadmap->AddVertex(col);
                const VID toVID = m_failedEffectorPlacementsRoadmap->AddVertex(effectorGroupCfg);
                m_failedEffectorPlacementsRoadmap->Write(
                          "FailedEffectorPlacements.map", this->GetEnvironment());
                std::cout << "Effector Placement Warning: LP failed, returning to"
                    " the first removal path cfg of formation " << activeRobots <<
                    " to try and find a new effector placement." << std::endl
                    << "Added vids (from, col, to) = ("
                    << fromVID << ", " << colVID << ", " << toVID
                    << ") to failed placement roadmap. Writing roadmap."
                    << std::endl;

                break; // Restart the cfg path with new part/body/polygon
              }
            }
          }
          else { // The very first path vid, add to path without a local plan.
            removalPath.push_back(newVID);
            lastCfg = effectorGroupCfg;
            lastCfgVID = newVID;
          }

          isFirst = false; // No longer the first vid of removal.
        } // End for all cfgs
        if(!returnToStartOfRemoval)
          return removalPath; // We made it through all the cfgs successfully.
      } // End for all polygons
    } // End for all bodies
  } // End for all parts

  return VIDPath(); // No polygon could be found.
}


template <typename MPTraits>
size_t
DisassemblyPreemptiveDFSManipulator<MPTraits>::
AttemptEffectorPlacement(Body* const _body, GroupCfgType& _manipulatorGroupCfg,
              const size_t _lastCandidateAttempted, const bool _isFirstInPath) {
  //Note: _lastCandidateAttempted has two different uses. If the flag
  //      _isFirstInPath is true, it will attempt starting at the index
  //      provided, otherwise it ONLY attempts that index, since it's mid-path.

  // TODO: remove the isFirstInPath option, as this function should now ONLY
  // attempt the polygon provided! Might want to just have it feed the polygon
  // directly (not the index) as well.

  // TODO: This function should probably just be in DisassemblyMethod.

  auto const vc = this->GetValidityChecker(this->m_vcLabel);

  // NOTE: this function was designed under the assumption that there will be a
  // full manipulator robot as the last on in the group cfg passed in. Nothing
  // should need to change now that we are using the end effector as its own
  // robot at this stage, but some things in here reflect the original design.


  // Note: this assumes the manipulator is the last robot in the group, and the
  // effector is the last body in the _manipulatorCfg's multibody and sets the
  // dofs for that body only.
  const std::vector<Robot*> manipRobots = _manipulatorGroupCfg.GetRobots();
  const size_t effectorRobotIndex = manipRobots.size() - 1;

  // Get the manipulator from the group task.
  Robot* const effectorRobot = this->GetGroupTask()->GetEndEffectorRobot();
  if(!effectorRobot)
    throw RunTimeException(WHERE, "No end effector robot present!");

  // Get the body of the effector and the info needed for aligning/placing it.
  const Robot::EndEffector effector = effectorRobot->GetEndEffector();

  // If the first one in the path, then attempt all starting from the last one
  // tried. Otherwise, we are mid-path and ONLY want to try the current.
  const auto& polys = _body->GetWorldPolyhedron().GetPolygonList();
  std::vector<size_t> candsToAttempt;
  if(_isFirstInPath) {
    for(size_t index = _lastCandidateAttempted; index < m_numContactCandidates;
        ++index)
      candsToAttempt.push_back(index);
  }
  else
    candsToAttempt = {_lastCandidateAttempted};

  for(const size_t candInd : candsToAttempt) {
    const GMSPolygon& cand = polys[candInd];
    const mathtool::Vector3d& contactPoint = cand.FindCenter();
    // Place the effector body.
    // Step 1: Align the two vectors (face normal and effector orientation
    // direction) using a rotation.
    // Get the reversed face normal unit vector (reversed so aligning works out)
    const mathtool::Vector3d faceNormal = cand.GetNormal();

    // Get the effector's alignment vector. Flip it because it points TO the tip
    // and we need the vectors aligned in the opposite manner.
    const mathtool::Vector3d effectorAlignmentVec =
                                       -effector.centerToContactDir.normalize();

    mathtool::Matrix3x3 identity;
    mathtool::identity(identity);
    Orientation orientationAlignment(identity);
    const double c = faceNormal * effectorAlignmentVec;
    const double justUnderOne = 1.0 -
                              (10.e13 * std::numeric_limits<double>::epsilon());
    if(std::abs(c) > justUnderOne) { // Then we have too similar of vectors.
      if(c < 0.) // If they are not the same direction, flip the end effector.
        orientationAlignment = -orientationAlignment;
    }
    else {
      // For reference, I am following the instructions for aligning two
      // vectors from:
      // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
      // Get dot and cross products for calculations.
      const mathtool::Vector3d v = effectorAlignmentVec % faceNormal;

      mathtool::Matrix3x3 rotMat;
      mathtool::getMatrix3x3(rotMat, 0.0, -v[2], v[1],
                                     v[2], 0.0, -v[0],
                                    -v[1], v[0], 0.0);
      orientationAlignment = Orientation(identity + rotMat +
                                        (rotMat * rotMat * (1 / (1 + c))));
    }

    // TODO: remove after fixing orientation stuff in coax!
    const double alignmentQuality =
                     (orientationAlignment * effectorAlignmentVec) * faceNormal;
    if(alignmentQuality < justUnderOne)
      std::cout << "Warning! Effector not properly aligned!" << std::endl;

    // Step 2: Set the effector pos dofs to be in the position that is needed.
    // The offset is to get the effector tip on the contact point, since we are
    // really moving around the effector wrt its center/centroid.
    // Adding in epsilon to avoid collisions.
    const double effectorOffsetDist = effector.centerToContactDir.norm() +
                                  10.e10*std::numeric_limits<double>::epsilon();

    const mathtool::Vector3d centerPlacementOffset = cand.GetNormal() *
                                                     effectorOffsetDist;
    const mathtool::Vector3d translationAlignment = contactPoint +
                                                    centerPlacementOffset;

    // Get the dofs that should align the end effector
    const mathtool::Transformation aligningTransform(translationAlignment,
                                                     orientationAlignment);
    std::vector<double> effectorDofs = aligningTransform.GetCfg();

    // Now place the effector's dofs into the cfg for the manipulator.
    const size_t effectorDofCount = 6; // Just to be explicit about hardcoding.
    CfgType manipCfg = _manipulatorGroupCfg.GetRobotCfg(effectorRobotIndex);
    std::vector<double> effectorGroupDofs = manipCfg.GetData();

    // Overwrite the last 6 dofs (for the effector) in the effector group.
    for(size_t manipInd = effectorGroupDofs.size() - effectorDofCount;
            manipInd < effectorGroupDofs.size(); ++manipInd) {
      effectorGroupDofs[manipInd] = effectorDofs[manipInd];
    }
    // Now overwrite the manipulator's dofs in the group cfg.
    _manipulatorGroupCfg.OverwriteDofsForRobots(effectorGroupDofs, {effectorRobotIndex});

    // Perform validity check and handle both cases:
    CDInfo cdInfo;
    const std::string label = this->GetNameAndLabel() + "::AttemptEffectorPlacement()";

    // Make the formation list be just the effector body, since we can assume
    // all other bodies have been co-validated for this cfg, if debug then just
    // do a fully exhaustive validity check (by providing empty formation list).
    Formation effectorGroup;
    if(!this->m_debug)
      effectorGroup = {_manipulatorGroupCfg.GetNumRobots() - 1};
    const bool isValid = vc->IsValid(_manipulatorGroupCfg, cdInfo, label,
                                     effectorGroup);
    // If it's valid, then we declare success and return.
    if(isValid) {
      if(this->m_debug)
        std::cout << "Successful effector placement for polygon " << candInd
                  << " with centroid " << contactPoint << std::endl;
      return candInd; // Return success for this polygon.
    }
    else if(this->m_debug) {
      std::cout << "Could not place effector on polygon " << candInd
                << " with centroid " << contactPoint << std::endl
                << "Adding to failed effector placement roadmap...";
      m_failedEffectorPlacementsRoadmap->AddVertex(_manipulatorGroupCfg);
    }
  }

  // Return invalid candidate index if everything failed.
  return std::numeric_limits<size_t>::max();
}


#endif
