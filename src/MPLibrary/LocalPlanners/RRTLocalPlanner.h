#ifndef RRT_LOCAL_PLANNER_H_
#define RRT_LOCAL_PLANNER_H_

#include "ConfigurationSpace/GroupRoadmap.h"
#include "MPLibrary/MPStrategies/BasicRRTStrategy.h"

#include "LocalPlannerMethod.h"
#include "GroupLPOutput.h"
#include "LPOutput.h"


////////////////////////////////////////////////////////////////////////////////
/// Attempt an RRT local plan between two configurations.
///
/// @todo Change use of this object to a substrategy. It does not meet the
///       criteria for a local planner because it is non-deterministic. It also
///       does not handle all the ways that MPLibrary can be configured - we
///       probably need to implement substrategy there.
///
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RRTLocalPlanner : public LocalPlannerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPSolution       MPSolution;
    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VID           VID;

    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename GroupRoadmapType::VID      GroupVID;
    typedef typename GroupCfgType::Formation    Formation;

    ///@}
    ///@name Construction
    ///@{

    RRTLocalPlanner(const std::string& _vcLabel = "", bool _binary = false,
        bool _saveIntermediates = false);

    RRTLocalPlanner(XMLNode& _node);

    virtual ~RRTLocalPlanner() = default;

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


    /// GroupCfg version.
    /// NOTE this is only intended to move individual parts! This is because the
    /// BasicRRT method is going to be used, since the DisassemblyRRT doesn't
    /// use the concept of goals (it is designed to be targetless). And BasicRRT
    /// doesn't yet have support for GroupCfgs, so the group method of this will
    /// move a part and conveniently use the GroupCfg class for ease.
    virtual bool IsConnected(
        const GroupCfgType& _c1, const GroupCfgType& _c2, GroupCfgType& _col,
        GroupLPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false,
        const Formation& _robotIndexes = Formation());

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


    ///@}
    ///@name Internal State
    ///@{

    std::string m_rrtStrategyLabel;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
RRTLocalPlanner<MPTraits>::
RRTLocalPlanner(const std::string& _vcLabel, bool _binary, bool _saveIntermediates)
  : LocalPlannerMethod<MPTraits>(_saveIntermediates) {
  this->SetName("RRTLocalPlanner");
}


template <typename MPTraits>
RRTLocalPlanner<MPTraits>::
RRTLocalPlanner(XMLNode& _node) : LocalPlannerMethod<MPTraits>(_node) {
  this->SetName("RRTLocalPlanner");

  m_rrtStrategyLabel = _node.Read("rrtStrategyLabel", true, "",
                                  "Label for the RRT strategy to use.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
RRTLocalPlanner<MPTraits>::
Print(std::ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  _os << "\tRRT Label = " << m_rrtStrategyLabel << std::endl;
}

/*----------------------- LocalPlannerMethod Overrides -----------------------*/

template <typename MPTraits>
bool
RRTLocalPlanner<MPTraits>::
IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  if(this->m_debug)
    std::cout << this->GetName() << "::IsConnected"
              << "\n\tChecking RRT connection from " << _c1.PrettyPrint()
              << " to " << _c2.PrettyPrint()
              << std::endl;

  _lpOutput->Clear();
  const bool connected = IsConnectedFunc(_c1, _c2, _col, _lpOutput,
                     _positionRes, _orientationRes, _checkCollision, _savePath);
  if(connected)
    _lpOutput->SetLPLabel(this->GetLabel());

  if(this->m_debug)
    std::cout << "\n\tLocal Plan is "
              << (connected ? "valid" : "invalid at " + _col.PrettyPrint())
              << std::endl;
  return connected;
}


template <typename MPTraits>
bool
RRTLocalPlanner<MPTraits>::
IsConnected(const GroupCfgType& _c1, const GroupCfgType& _c2, GroupCfgType& _col,
            GroupLPOutput<MPTraits>* _lpOutput,
            double _positionRes, double _orientationRes,
            bool _checkCollision, bool _savePath,
            const Formation& _robotIndexes) {
  if(_robotIndexes.size() != 1)
    throw RunTimeException(WHERE, "Can only have one active robot right now!");

  const size_t movedRobot = _robotIndexes[0];

  if(this->m_debug)
    std::cout << "Performing local plan with active robots " << _robotIndexes
              << std::endl;

  _lpOutput->Clear();

  // Configure the cfg so that everything static is set in place before LP.
  _c1.ConfigureRobot();

  const CfgType& start = _c1.GetRobotCfg(movedRobot);
  const CfgType& end = _c2.GetRobotCfg(movedRobot);
  CfgType col(start.GetRobot());

  LPOutput<MPTraits> robotLPOutput;

  const bool connected = IsConnectedFunc(start, end, col, &robotLPOutput,
                     _positionRes, _orientationRes, _checkCollision, _savePath);

  if(connected) {
    _lpOutput->SetLPLabel(this->GetLabel());
    // Convert all of the CfgType stuff into GroupCfg stuff.
    if(_savePath) {
      for(CfgType& robotCfg : robotLPOutput.m_path) {
        // Copy over the start so that all other robots are in position
        GroupCfgType groupCfg = _c1;
        groupCfg.SetRobotCfg(movedRobot, std::move(robotCfg));
        _lpOutput->m_path.push_back(groupCfg);
      }
    }
  }
  else {
    _col = _c1; // So all other parts match up, then set the moved robot's cfg.
    _col.SetRobotCfg(movedRobot, std::move(col));
  }

  _lpOutput->SetActiveRobots(_robotIndexes);
  _lpOutput->SetIndividualEdges(_robotIndexes);

  return connected;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
RRTLocalPlanner<MPTraits>::
IsConnectedFunc(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {

  StatClass* const stats = this->GetStatClass();

  stats->IncLPAttempts(this->GetNameAndLabel());

  // TODO note that right now we will assume it will be connected at the end of
  // the RRT. This should probably change since we aren't guaranteed this,
  // though it should be possible is just about every case planned.

  auto rrtStrategy = dynamic_pointer_cast< BasicRRTStrategy<MPTraits> >(
                                       this->GetMPStrategy(m_rrtStrategyLabel));
  rrtStrategy->EnableOutputFiles(false);

  // Store the current task and solution.
  MPTask* const originalTask = this->GetMPLibrary()->GetTask();
  MPSolution* const originalSolution = this->GetMPLibrary()->GetMPSolution();

  // Set the task since the robot won't have one in the solution yet, and RRT
  // will access it.
  auto robot = _c1.GetRobot();
  MPTask tempTask(robot);
  MPSolution tempSolution(robot);

  using ConstraintPtr = std::unique_ptr<CSpaceConstraint>;
  tempTask.SetStartConstraint(ConstraintPtr(new CSpaceConstraint(robot, _c1)));
  tempTask.AddGoalConstraint(ConstraintPtr(new CSpaceConstraint(robot, _c2)));

  // Swap the pointer in MPLibrary for the new MPSolution
  this->GetMPLibrary()->SetTask(&tempTask);
  this->GetMPLibrary()->SetMPSolution(&tempSolution);

  // Reset the goal tracker.
  this->GetGoalTracker()->Reset(tempSolution.GetRoadmap(robot), &tempTask);

  // Call operator() on the MPStrategy, which will solve it and place all
  // needed data in the new MPSolution.
  (*rrtStrategy)();

  // Set the path found in lpOutput.
  _lpOutput->m_path = tempSolution.GetPath()->Cfgs();

  // Set the solution back to the original.
  this->GetMPLibrary()->SetMPSolution(originalSolution);
  this->GetMPLibrary()->SetTask(originalTask);
  this->GetGoalTracker()->Reset(
      originalSolution->GetRoadmap(originalTask->GetRobot()), originalTask);

  // This used to use the IsSuccessful member, which has been removed from
  // MPStrategyMethod. I am assuming that finding a path equals success here
  // based on the comments and my understanding of the method.
  return !_lpOutput->m_path.empty();
}


/*----------------------------------------------------------------------------*/

#endif
