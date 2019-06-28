#ifndef STRAIGHT_LINE_H_
#define STRAIGHT_LINE_H_

#include "ConfigurationSpace/GroupRoadmap.h"

#include "LocalPlannerMethod.h"
#include "GroupLPOutput.h"
#include "LPOutput.h"
#include "MPProblem/IsClosedChain.h"
#include "Transformation.h"
#include "Vector.h"
#include "nonstd.h"

#include <deque>

#include <boost/utility/enable_if.hpp>


////////////////////////////////////////////////////////////////////////////////
/// Check a straight-line path in c-space for valididty.
///
/// This local planner validates straight line paths which is a direct linear
/// interpolation between two configurations in @cspace.
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class StraightLine : public LocalPlannerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::WeightType       WeightType;

    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename GroupCfgType::Formation    Formation;

    ///@}
    ///@name Construction
    ///@{

    StraightLine(const std::string& _vcLabel = "", bool _binary = false,
        bool _saveIntermediates = false);

    StraightLine(XMLNode& _node);

    virtual ~StraightLine() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name LocalPlannerMethod Overrides
    ///@{

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false) override;

    virtual bool IsConnected(
        const GroupCfgType& _c1, const GroupCfgType& _c2, GroupCfgType& _col,
        GroupLPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false,
        const Formation& _robotIndexes = Formation()) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Default for non closed chains
    bool IsConnectedFunc(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    /// Check if two Cfgs could be connected by straight line.
    /// This method implements straight line connection local planner
    /// by checking collision of each Cfg along the line.
    /// If the is any Cfg causes Robot collides with any obstacle,
    /// false will be returned.
    virtual bool IsConnectedSLSequential(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    /// Check if two Cfgs could be connected by straight line
    /// This method uses binary search to check clearances of Cfgs between _c1
    /// and _c2.
    virtual bool IsConnectedSLBinary(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_dmLabel;          ///< The metric for measuring edge length.
    std::string m_vcLabel;          ///< The validity checker.
    bool m_binaryEvaluation{false}; ///< Use binary search?

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
StraightLine<MPTraits>::
StraightLine(const std::string& _vcLabel, bool _binary, bool _saveIntermediates)
  : LocalPlannerMethod<MPTraits>(_saveIntermediates),
    m_vcLabel(_vcLabel), m_binaryEvaluation(_binary) {
  this->SetName("StraightLine");
}


template <typename MPTraits>
StraightLine<MPTraits>::
StraightLine(XMLNode& _node) : LocalPlannerMethod<MPTraits>(_node) {
  this->SetName("StraightLine");

  m_binaryEvaluation = _node.Read("binaryEvaluation", false, m_binaryEvaluation,
      "Use binary search to evaluate the edge, or linear scan if false.");

  m_dmLabel = _node.Read("dmLabel", false, "euclidean",
      "The distance metric for computing edge length.");

  m_vcLabel = _node.Read("vcLabel", true, "", "The validity checker to use.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
StraightLine<MPTraits>::
Print(std::ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  _os << "\tbinary evaluation = " << (m_binaryEvaluation ? "true" : "false")
      << "\n\tdmLabel = " << m_dmLabel
      << "\n\tvcLabel = " << m_vcLabel
      << std::endl;
}

/*----------------------- LocalPlannerMethod Overrides -----------------------*/

template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  if(this->m_debug)
    std::cout << this->GetName() << "::IsConnected"
              << "\n\tChecking line from " << _c1.PrettyPrint()
              << " to " << _c2.PrettyPrint()
              << "\n\tUsing " << (m_binaryEvaluation ? "binary" : "sequential")
              << " evaluation."
              << std::endl;

  _lpOutput->Clear();
  const bool connected = IsConnectedFunc(_c1, _c2,
      _col, _lpOutput, _positionRes, _orientationRes, _checkCollision,
      _savePath);
  /// @todo We should be setting the LP label either way, need to test before
  ///       removing this check.

  if(this->m_debug)
    std::cout << "\n\tLocal Plan is "
              << (connected ? "valid" : "invalid at " + _col.PrettyPrint())
              << std::endl;
  return connected;
}


template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnected(const GroupCfgType& _c1, const GroupCfgType& _c2, GroupCfgType& _col,
    GroupLPOutput<MPTraits>* _lpOutput, double _positionRes,
    double _orientationRes, bool _checkCollision, bool _savePath,
    const Formation& _robotIndexes) {
  if(this->m_debug) {
    std::cout << this->GetName() << "::IsConnected"
              << "\n\tChecking line from " << _c1.PrettyPrint()
              << " to " << _c2.PrettyPrint()
              << std::endl;
    if(!_robotIndexes.empty())
      std::cout << "\tUsing formation: " << _robotIndexes << std::endl;
  }
  _lpOutput->Clear();

  auto stats = this->GetStatClass();
  stats->IncLPAttempts(this->GetNameAndLabel());

  bool connected = true;

  auto env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto groupMap = _c1.GetGroupRoadmap();


  const std::string callee = this->GetNameAndLabel() + "::IsConnectedFunc";

  // Determine whether multiple robots are moving and whether this is a
  // formation rotation (rotation about some leader robot).
  const bool multipleParts = _robotIndexes.size() > 1;
  const bool isRotational = _c1.OriDOF() > 0;
  const bool formationRotation = multipleParts && isRotational;
  const size_t leaderRobotIndex = _robotIndexes.empty() ? size_t(-1)
                                                        : _robotIndexes[0];

  // Will find all the straight-line increments for each robot independently.
  // (Though the nTicks calculation is coupled with all moving robots).
  int nTicks;
  GroupCfgType incr(groupMap);
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);

  const GroupCfgType originalIncr = incr;

  // Set up incr for all translating bodies, should there be more than one.
  if(multipleParts) {
    // Remove the rotational bits, as incr should only do the translation
    //  and then RotateFormationAboutLeader() will handle all rotations:
    incr = GroupCfgType(groupMap, true);

    // Overwrite all positional dofs from the leader's cfg for all active robots
    incr.OverwriteDofsForRobots(
        originalIncr.GetRobotCfg(leaderRobotIndex).GetLinearPosition(),
        _robotIndexes);
  }

  int cdCounter = 0;
  int nIter = 0;
  GroupCfgType tick(_c1),
               leaderTick(_c1),
               previous(groupMap);
  for(int i = 1; i < nTicks; ++i, ++nIter) {
    previous = tick;
    tick += incr;

    // Handle rotation of a formation. We will determine the rotation applied to
    // the leader robot and cause the others to rotate about it, maintaining
    // their realtive formation
    if(formationRotation) {
      /// @todo This can likely be optimized. For one, only one Configure call
      ///       should be necessary here. Also a lot of the group Cfgs here
      ///       could be made individual if using the leader, then using
      ///       Configure on that.

      // Advance the leader tick by the original increment (we will only use
      // data which is set in the leader body).
      leaderTick += originalIncr;

      // Find the previous transformation of the leader robot's base.
      previous.ConfigureRobot();
      mathtool::Transformation initialTransform =
          previous.GetRobot(leaderRobotIndex)->GetMultiBody()->GetBase()->
          GetWorldTransformation();

      // Find the new transformation of the leader robot's base.
      leaderTick.ConfigureRobot();
      mathtool::Transformation finalTransform =
          leaderTick.GetRobot(leaderRobotIndex)->GetMultiBody()->GetBase()->
          GetWorldTransformation();

      // Find the relative transformation of the leader robot's base. This holds
      // the rotation to be applied to tick, which only increments position in
      // this case.
      mathtool::Transformation delta = -initialTransform * finalTransform;
      tick.RotateFormationAboutLeader(_robotIndexes, delta.rotation(),
          this->m_debug);
    }

    // Check collision if requested.
    if(_checkCollision) {
      ++cdCounter;
      if(!tick.InBounds(env->GetBoundary()) or !vc->IsValid(tick, callee)) {
        _col = tick;
        connected = false;
        break;
      }
    }

    // Save path if requested.
    if(_savePath)
      _lpOutput->m_path.push_back(tick);
  }

  // Set data in the LPOutput object.
  _lpOutput->m_edge.first.SetWeight(nIter);
  _lpOutput->m_edge.second.SetWeight(nIter);
  _lpOutput->SetIndividualEdges(_robotIndexes);
  _lpOutput->SetActiveRobots(_robotIndexes);
  _lpOutput->SetLPLabel(this->GetLabel());

  if(connected)
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);

  stats->IncLPConnections(this->GetNameAndLabel(), connected);
  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);

  if(this->m_debug)
    std::cout << "\n\tLocal Plan is "
              << (connected ? "valid" : "invalid at " + _col.PrettyPrint())
              << std::endl;
  return connected;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnectedFunc(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {

  StatClass* const stats = this->GetStatClass();

  stats->IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0;

  bool connected;
  if(m_binaryEvaluation)
    connected = IsConnectedSLBinary(_c1, _c2, _col, _lpOutput,
        cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);
  else
    connected = IsConnectedSLSequential(_c1, _c2, _col, _lpOutput,
        cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);

  if(connected)
    stats->IncLPConnections(this->GetNameAndLabel());

  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
}


template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnectedSLSequential(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {

  Environment* env = this->GetEnvironment();
  //auto robot = this->GetTask()->GetRobot();
  auto robot = _c1.GetRobot();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);

  int nTicks;
  CfgType tick(robot), incr(robot), prev(robot);
  tick = _c1;
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);
  std::string callee = this->GetNameAndLabel() + "::IsConnectedSLSequential";

  if(this->m_debug)
    std::cout << "\n\tComputed increment for " << nTicks << " ticks: "
              << incr.PrettyPrint() << std::endl;
  int nIter = 0;
  double distance = 0;
  for(int i = 1; i < nTicks; ++i) { //don't need to check the ends _c1, _c2
    prev = tick;
    tick += incr;
    distance += dm->Distance(prev, tick);
    _cdCounter++;

    if(_checkCollision) {
      if(this->m_debug)
        std::cout << "\n\t\tChecking " << tick.PrettyPrint();

      const bool inBounds = tick.InBounds(env);
      if(!inBounds || !vc->IsValid(tick, callee)) {
        _col = tick;
        auto& edge1 = _lpOutput->m_edge.first,
            & edge2 = _lpOutput->m_edge.second;

        edge1.SetWeight(edge1.GetWeight() + distance);
        edge2.SetWeight(edge2.GetWeight() + distance);
        edge1.SetTimeSteps(edge1.GetTimeSteps() + nIter);
        edge2.SetTimeSteps(edge2.GetTimeSteps() + nIter);

        if(this->m_debug)
          std::cout << " INVALID";
        return false;
      }
      else if(this->m_debug)
        std::cout << " OK";
    }
    if(_savePath)
      _lpOutput->m_path.push_back(tick);
    nIter++;
  }

  // The edge is valid, now add the distance to the final configuration.
  distance += dm->Distance(tick, _c2);

  auto& edge1 = _lpOutput->m_edge.first,
      & edge2 = _lpOutput->m_edge.second;

  edge1.SetWeight(edge1.GetWeight() + distance);
  edge2.SetWeight(edge2.GetWeight() + distance);
  edge1.SetTimeSteps(edge1.GetTimeSteps() + nTicks);
  edge2.SetTimeSteps(edge2.GetTimeSteps() + nTicks);

  return true;
}


template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnectedSLBinary(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {

  Environment* env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);

  if(!_checkCollision)
    return IsConnectedSLSequential(_c1, _c2, _col, _lpOutput,
        _cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);

  std::string callee = this->GetNameAndLabel() + "::IsConnectedSLBinary";

  int nTicks;
  CfgType incr(this->GetTask()->GetRobot());
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);

  if(this->m_debug)
    std::cout << "\n\tComputed increment for " << nTicks << " ticks: "
              << incr.PrettyPrint()
              << std::endl;

  std::deque<std::pair<int,int> > Q;

  //only perform binary evaluation when the nodes are further apart than the
  //resolution
  if(nTicks > 1)
    Q.push_back(make_pair(0, nTicks));

  while(!Q.empty()) {
    std::pair<int,int> p = Q.front();
    int i = p.first;
    int j = p.second;
    Q.pop_front();

    int mid = i + (j - i) / 2;

    CfgType midCfg = incr * mid + _c1;

    if(this->m_debug)
      std::cout << "\n\t\tChecking " << midCfg.PrettyPrint();

    _cdCounter++;

    if(!midCfg.InBounds(env) || !vc->IsValid(midCfg, callee) ) {
      if(midCfg.InBounds(env))
        _col = midCfg;
      if(this->m_debug)
        std::cout << " INVALID";
      return false;
    }
    else {
      if(i + 1 != mid)
        Q.push_back(make_pair(i, mid));
      if(mid + 1 != j)
        Q.push_back(make_pair(mid, j));

      if(this->m_debug)
        std::cout << " OK";
    }
  }

  // Compute the distance and path.
  double distance = 0;
  CfgType tick = _c1, prev;
  for(int n = 1; n < nTicks; ++n) {
    prev = tick;
    tick += incr;
    distance += dm->Distance(prev, tick);
    if(_savePath)
      _lpOutput->m_path.push_back(tick);
  }
  distance += dm->Distance(tick, _c2);

  auto& edge1 = _lpOutput->m_edge.first,
      & edge2 = _lpOutput->m_edge.second;

  edge1.SetWeight(edge1.GetWeight() + distance);
  edge2.SetWeight(edge2.GetWeight() + distance);
  edge1.SetTimeSteps(edge1.GetTimeSteps() + nTicks);
  edge2.SetTimeSteps(edge2.GetTimeSteps() + nTicks);

  return true;
}

/*----------------------------------------------------------------------------*/

#endif
