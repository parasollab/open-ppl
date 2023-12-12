#include "MPLibrary.h"

#include <algorithm>
#include <atomic>
#include <unordered_map>

/*---------------------------- Construction ----------------------------------*/

MPLibrary::
MPLibrary() {
  m_distanceMetrics = new DistanceMetricSet(this,
      typename MPUniverse::DistanceMetricMethodList(), "DistanceMetrics");
  m_validityCheckers = new ValidityCheckerSet(this,
      typename MPUniverse::ValidityCheckerMethodList(), "ValidityCheckers");
  m_neighborhoodFinders = new NeighborhoodFinderSet(this,
      typename MPUniverse::NeighborhoodFinderMethodList(), "NeighborhoodFinders");
  m_samplers = new SamplerSet(this,
      typename MPUniverse::SamplerMethodList(), "Samplers");
  m_localPlanners = new LocalPlannerSet(this,
      typename MPUniverse::LocalPlannerMethodList(), "LocalPlanners");
  m_extenders = new ExtenderSet(this,
      typename MPUniverse::ExtenderMethodList(), "Extenders");
  m_pathModifiers = new PathModifierSet(this,
      typename MPUniverse::PathModifierMethodList(), "PathModifiers");
  m_edgeValidityCheckers = new EdgeValidityCheckerSet(this,
      typename MPUniverse::EdgeValidityCheckerMethodList(), "EdgeValidityCheckers");
  m_connectors = new ConnectorSet(this,
      typename MPUniverse::ConnectorMethodList(), "Connectors");
  m_metrics = new MetricSet(this,
      typename MPUniverse::MetricMethodList(), "Metrics");
  m_mapEvaluators = new MapEvaluatorSet(this,
      typename MPUniverse::MapEvaluatorMethodList(), "MapEvaluators");
  m_mpStrategies = new MPStrategySet(this,
      typename MPUniverse::MPStrategyMethodList(), "MPStrategies");
  m_mpTools = new MPTools(this);
  m_goalTracker.reset(new GoalTracker(this));
}


MPLibrary::
MPLibrary(const std::string& _filename) : MPLibrary() {
  ReadXMLFile(_filename);
}


MPLibrary::
MPLibrary(XMLNode& planningLibraryNode) : MPLibrary() {
  ProcessXML(planningLibraryNode);
}


MPLibrary::
~MPLibrary() {
  delete m_distanceMetrics;
  delete m_validityCheckers;
  delete m_neighborhoodFinders;
  delete m_samplers;
  delete m_localPlanners;
  delete m_extenders;
  delete m_pathModifiers;
  delete m_edgeValidityCheckers;
  delete m_connectors;
  delete m_metrics;
  delete m_mapEvaluators;
  delete m_mpStrategies;
  delete m_mpTools;
}


void
MPLibrary::
Initialize() {
  MethodTimer mt(this->GetStatClass(), "MPLibrary::Initialize");

  // Set up the goal tracker.
  m_goalTracker->Clear();
  if(this->GetTask())
    m_goalTracker->AddMap(GetRoadmap(), GetTask());
  else if(this->GetGroupTask())
    m_goalTracker->AddMap(GetGroupRoadmap(), GetGroupTask());
  else
    throw RunTimeException(WHERE) << "No current task was set.";

  m_distanceMetrics->Initialize();
  m_validityCheckers->Initialize();
  m_neighborhoodFinders->Initialize();
  m_samplers->Initialize();
  m_localPlanners->Initialize();
  m_extenders->Initialize();
  m_pathModifiers->Initialize();
  m_edgeValidityCheckers->Initialize();
  m_connectors->Initialize();
  m_metrics->Initialize();
  m_mapEvaluators->Initialize();
  m_mpTools->Initialize();

  m_running = true;
}


void
MPLibrary::
Uninitialize() {
  // Clear goal tracker.
  m_goalTracker->Clear();

  // Clear group hooks.
  GroupRoadmapType* const groupMap = this->GetGroupRoadmap();
  if(groupMap)
    groupMap->ClearHooks();

  // Also clear hooks for the individual robot if it exists:
  if(m_solution->GetRobot())
    this->GetRoadmap()->ClearHooks();
}

/*---------------------------- XML Helpers -----------------------------------*/

void
MPLibrary::
ReadXMLFile(const std::string& _filename) {
  // Open the XML and get the root node.
  XMLNode mpNode(_filename, "MotionPlanning");

  // Find the 'MPLibrary' node.
  XMLNode* planningLibrary = nullptr;
  for(auto& child : mpNode)
    if(child.Name() == "Library")
      planningLibrary = &child;

  // Throw exception if we can't find it.
  if(!planningLibrary)
    throw ParseException(WHERE) << "Cannot find MPLibrary node in XML file '"
                                << _filename << "'.";
  ProcessXML(*planningLibrary);
}


void
MPLibrary::
ProcessXML(XMLNode& planningLibraryNode) {

  // Parse the library node to set algorithms and parameters.
  for(auto& child : planningLibraryNode)
    ParseChild(child);

  // Ensure we have at least one solver.
  if(m_solvers.empty())
    throw ParseException(WHERE) << "Cannot find Solver node in XML node '.";
  
  // Set the seed to the first solver
  SetSeed();

  // Print XML details if requested.
  bool print = planningLibraryNode.Read("print", false, false, "Print all XML input");
  if(print)
    Print(cout);

  // Handle XML warnings/errors.
  bool warnings = planningLibraryNode.Read("warnings", false, false, "Report warnings");
  if(warnings) {
    bool warningsAsErrors = planningLibraryNode.Read("warningsAsErrors", false, false,
        "XML warnings considered errors");
    planningLibraryNode.WarnAll(warningsAsErrors);
  }
}


bool
MPLibrary::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "DistanceMetrics") {
    m_distanceMetrics->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "ValidityCheckers") {
    m_validityCheckers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "NeighborhoodFinders") {
    m_neighborhoodFinders->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Samplers") {
    m_samplers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "LocalPlanners") {
    m_localPlanners->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Extenders") {
    m_extenders->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "PathModifiers") {
    m_pathModifiers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "EdgeValidityCheckers") {
    m_edgeValidityCheckers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Connectors") {
    m_connectors->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Metrics") {
    m_metrics->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MapEvaluators") {
    m_mapEvaluators->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MPStrategies") {
    m_mpStrategies->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MPTools") {
    m_mpTools->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Solver") {
    const std::string label = _node.Read("mpStrategyLabel", true, "",
        "The strategy to use.");

    const long seed = _node.Read("seed", true, size_t(1), size_t(0),
        std::numeric_limits<size_t>::max(),
        "The random number generator seed.");

    const std::string baseFilename = _node.Read("baseFilename", true, "",
        "BaseFilename for the solver.") + "." + std::to_string(seed);

    const bool vdOutput = _node.Read("vizmoDebug", false, false,
        "True yields VizmoDebug output for the solver.");

    m_solvers.emplace_back(Solver{label, seed, baseFilename, vdOutput});
    return true;
  }
  else
    return false;
}

/*-------------------------------- Debugging ---------------------------------*/

void
MPLibrary::
Print(ostream& _os) const {
  _os << "MPLibrary" << std::endl;
  m_distanceMetrics->Print(_os);
  m_validityCheckers->Print(_os);
  m_neighborhoodFinders->Print(_os);
  m_samplers->Print(_os);
  m_localPlanners->Print(_os);
  m_extenders->Print(_os);
  m_pathModifiers->Print(_os);
  m_edgeValidityCheckers->Print(_os);
  m_connectors->Print(_os);
  m_metrics->Print(_os);
  m_mapEvaluators->Print(_os);
  m_mpStrategies->Print(_os);
}

/*----------------------------- Input Accessors ------------------------------*/

MPProblem*
MPLibrary::
GetMPProblem() const noexcept {
  return m_problem;
}


void
MPLibrary::
SetMPProblem(MPProblem* const _problem) noexcept
{
  m_problem = _problem;
}


MPTask*
MPLibrary::
GetTask() const noexcept {
  return m_task;
}


void
MPLibrary::
SetTask(MPTask* const _task) noexcept {
  m_task = _task;
}


GroupTask*
MPLibrary::
GetGroupTask() const noexcept {
  return m_groupTask;
}


void
MPLibrary::
SetGroupTask(GroupTask* const _task) noexcept {
  m_groupTask = _task;
}


const std::string&
MPLibrary::
GetBaseFilename() const noexcept {
  return m_problem->GetBaseFilename();
}


void
MPLibrary::
SetBaseFilename(const std::string& _s) noexcept {
  m_problem->SetBaseFilename(_s);
}

/*---------------------------- Solution Accessors ----------------------------*/

typename MPLibrary::MPSolution*
MPLibrary::
GetMPSolution() const noexcept {
  return m_solution;
}


void
MPLibrary::
SetMPSolution(MPSolution* const _sol) noexcept {
  m_solution = _sol;
}


typename MPLibrary::RoadmapType*
MPLibrary::
GetRoadmap(Robot* const _r) const noexcept {
  if(!_r and !GetTask())
    return nullptr;
  return m_solution->GetRoadmap(_r ? _r : GetTask()->GetRobot());
}


typename MPLibrary::GroupRoadmapType*
MPLibrary::
GetGroupRoadmap(RobotGroup* const _g) const noexcept {
  if(!_g and !GetGroupTask())
    return nullptr;
  return m_solution->GetGroupRoadmap(_g ? _g : GetGroupTask()->GetRobotGroup());
}


typename MPLibrary::RoadmapType*
MPLibrary::
GetBlockRoadmap(Robot* const _r) const noexcept {
  if(!_r and !GetTask())
    return nullptr;
  return m_solution->GetBlockRoadmap(_r ? _r : GetTask()->GetRobot());
}


Path*
MPLibrary::
GetPath(Robot* const _r) const noexcept {
  if(!_r and !GetTask())
    return nullptr;
  return m_solution->GetPath(_r ? _r : GetTask()->GetRobot());
}


GroupPath*
MPLibrary::
GetGroupPath(RobotGroup* const _g) const noexcept {
  if(!_g and !GetGroupTask())
    return nullptr;
  return m_solution->GetGroupPath(_g ? _g : GetGroupTask()->GetRobotGroup());
}


LocalObstacleMap*
MPLibrary::
GetLocalObstacleMap(Robot* const _r) const noexcept {
  if(!_r and !GetTask())
    return nullptr;
  return m_solution->GetLocalObstacleMap(_r ? _r : GetTask()->GetRobot());
}


typename MPLibrary::GoalTracker*
MPLibrary::
GetGoalTracker() const noexcept {
  return m_goalTracker.get();
}


StatClass*
MPLibrary::
GetStatClass() const noexcept {
  return m_solution->GetStatClass();
}

/*--------------------------- Edge Reconstruction ----------------------------*/

std::vector<typename MPLibrary::RoadmapType::VP>
MPLibrary::
ReconstructEdge(RoadmapType* const _roadmap, const VID _source,
    const VID _target, const double _posRes,
    const double _oriRes) {
  const auto& start         = _roadmap->GetVertex(_source);
  const auto& end           = _roadmap->GetVertex(_target);
  const auto& edge          = _roadmap->GetEdge(_source, _target);
  const auto& intermediates = edge.GetIntermediates();

  // Construct the set of start, intermediates, and end waypoints as needed.
  auto waypoints = intermediates;
  waypoints.insert(waypoints.begin(), start);
  waypoints.push_back(end);

  // Check for intermediates. If there are any, we will use straight-line to
  // recompute the path. Otherwise use the lp label if available, and fall back
  // to straight-line if not (this will always happen with extenders).
  const std::string lpLabel = !intermediates.size()
                            and !edge.GetLPLabel().empty()
                            ? edge.GetLPLabel()
                            : "sl";

  // Construct a resolution-level path along the recreated edge.
  auto lp = this->GetLocalPlanner(lpLabel);
  return lp->BlindPath(waypoints, _posRes, _oriRes);
}


std::vector<typename MPLibrary::GroupRoadmapType::VP>
MPLibrary::
ReconstructEdge(GroupRoadmapType* const _roadmap, const VID _source,
    const VID _target, const double _posRes,
    const double _oriRes) {
  const auto& start         = _roadmap->GetVertex(_source);
  const auto& end           = _roadmap->GetVertex(_target);
  const auto& edge          = _roadmap->GetEdge(_source, _target);
  const auto& intermediates = edge.GetIntermediates();

  // Construct the set of start, intermediates, and end waypoints as needed.
  auto waypoints = intermediates;
  waypoints.insert(waypoints.begin(), start);
  waypoints.push_back(end);

  // Check for intermediates. If there are any, we will use straight-line to
  // recompute the path. Otherwise use the lp label if available, and fall back
  // to straight-line if not (this will always happen with extenders).
  const std::string lpLabel = !intermediates.size()
                            and !edge.GetLPLabel().empty()
                            ? edge.GetLPLabel()
                            : "sl";

  // Construct a resolution-level path along the recreated edge.
  auto lp = this->GetLocalPlanner(lpLabel);
  return lp->BlindPath(waypoints, _posRes, _oriRes, edge.GetFormation());
}

/*--------------------------- Execution Interface ----------------------------*/

void
MPLibrary::
Halt() {
  m_running = false;
}


bool
MPLibrary::
IsRunning() const noexcept {
  return m_running;
}


void
MPLibrary::
SetSeed() const noexcept {
  if(m_solvers.empty())
    throw RunTimeException(WHERE) << "No solver nodes.";
  SetSeed(m_solvers[0].seed);
}


void
MPLibrary::
SetSeed(const long _seed) const noexcept {
#ifdef _PARALLEL
  SRand(_seed + get_location_id());
#else
  SRand(_seed);
#endif
}


void
MPLibrary::
Solve(MPProblem* _problem, MPTask* _task, MPSolution* _solution) {
  m_problem = _problem;
  m_task = _task;
  m_solution = _solution;

  for(auto& solver : m_solvers)
    RunSolver(solver);
}


void
MPLibrary::
Solve(MPProblem* _problem, MPTask* _task) {
  m_problem = _problem;
  m_task = _task;

  for(auto& solver : m_solvers) {
    // Create storage for the solution.
    m_solution = new MPSolution(m_task->GetRobot());

    RunSolver(solver);

    delete m_solution;
  }

  m_solution = nullptr;
}


void
MPLibrary::
Solve(MPProblem* _problem, GroupTask* _task) {
  m_problem = _problem;
  m_groupTask = _task;

  for(auto& solver : m_solvers) {
    // Create storage for the solution.
    m_solution = new MPSolution(m_groupTask->GetRobotGroup());

    RunSolver(solver);

    delete m_solution;
  }

  m_solution = nullptr;
}


void
MPLibrary::
Solve(MPProblem* _problem, GroupTask* _task, MPSolution* _solution) {
  m_problem = _problem;
  m_groupTask = _task;
  m_solution = _solution;

  for(auto& solver : m_solvers) {
    RunSolver(solver);
  }

}


void
MPLibrary::
Solve(MPProblem* _problem, MPTask* _task, MPSolution* _solution,
    const std::string& _label, const long _seed,
    const std::string& _baseFilename) {
  m_problem = _problem;
  m_task = _task;
  m_solution = _solution;

  Solver s{_label, _seed, _baseFilename, false};
  RunSolver(s);
}


void
MPLibrary::
RunSolver(const Solver& _solver) {
  // Announce the method label and seed.
  std::cout << "\n\nMPLibrary is solving with MPStrategyMethod labeled "
            << _solver.label << " using seed " << _solver.seed << "."
            << std::endl;

  // Save the original prefix for output files (base name).
  const std::string originalBaseFilename = m_problem->GetBaseFilename();

  // Use the solver node to set the base name for output files.
  std::string baseFilename = _solver.baseFilename;

  // If this task has a label, append it to the solver's output file name.
  if(m_groupTask) {
    if(!m_groupTask->GetLabel().empty())
      baseFilename += "." + m_groupTask->GetLabel();
  }
  else if(!m_task->GetLabel().empty())
    baseFilename += "." + m_task->GetLabel();

  // Remove spaces to keep file names nice.
  {
    auto newEnd = std::remove_if(baseFilename.begin(), baseFilename.end(),
        ::isspace);
    baseFilename.erase(newEnd, baseFilename.end());
  }

  // Set the output file locations.
  SetBaseFilename(GetMPProblem()->GetPath(baseFilename));
  GetStatClass()->SetAuxDest(GetBaseFilename());

  // Initialize vizmo debug if requested.
  if(_solver.vizmoDebug) {
    VDInit(GetBaseFilename() + ".vd");
    if(m_groupTask)
      VDTrackRoadmap(this->GetGroupRoadmap());
    else
      VDTrackRoadmap(this->GetRoadmap());
  }

  // Initialize the library's algorithms.
  SetSeed(_solver.seed);
  Initialize();

  // Reset the seed
  SetSeed(_solver.seed);

  GetMPStrategy(_solver.label)->operator()();

  // Close vizmo debug if necessary
  if(_solver.vizmoDebug)
    VDClose();

  this->GetStatClass()->PrintClock("MPLibrary::Initialize", std::cout);
  Uninitialize();
  m_problem->SetBaseFilename(originalBaseFilename);
}
