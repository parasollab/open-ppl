#include "LowLevelSearch.h"

#include "Behaviors/Agents/HandoffAgent.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "Traits/CfgTraits.h"

LowLevelSearch::
LowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel, bool _debug) : 
								m_tmpLibrary(_tmpLibrary), m_sgLabel(_sgLabel), m_vcLabel(_vcLabel), m_debug(_debug){

	//TODO::Update this to a parameter and make an initialize function
	std::string m_queryLabel = "MegaQuery";
	
	auto query = dynamic_cast<QueryMethod<MPTraits<Cfg,DefaultWeight<Cfg>>>*>(
									m_tmpLibrary->GetMPLibrary()->GetMapEvaluator(m_queryLabel).get());

	auto query2 = dynamic_cast<QueryMethod<MPTraits<Cfg,DefaultWeight<Cfg>>>*>(
									m_tmpLibrary->GetMPLibrary()->GetMapEvaluator("TwoVariableQuery").get());

	if(!query) {
		throw RunTimeException(WHERE) << "Query method " << m_queryLabel
																	<< " is not of type QueryMethod."
																	<< std::endl;
	}

	if(!query2) {
		throw RunTimeException(WHERE) << "Query method " << "TwoVariableQuery"
																	<< " is not of type QueryMethod."
																	<< std::endl;
	}
	// Set the query method's path weight function.
	query->SetPathWeightFunction(
		[this](typename Roadmap::adj_edge_iterator& _ei,
					 const double _sourceDistance,
					 const double _targetDistance) { 
			return this->MultiRobotPathWeight(_ei, _sourceDistance, _targetDistance);
		}
	);

	query2->SetPathWeightFunction(
		[this](typename Roadmap::adj_edge_iterator& _ei,
					 const double _sourceDistance,
					 const double _targetDistance) { 
			return this->MultiRobotPathWeight(_ei, _sourceDistance, _targetDistance);
		}
	);
}
bool
LowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) {
	return true;
}


std::pair<double,std::pair<std::vector<size_t>,size_t>>
LowLevelSearch::
MotionPlan(Cfg _start, Cfg _goal, double _startTime, double _minEndTime, SemanticTask* _currentTask) {

	if(_currentTask)
		m_currentTask = _currentTask;

	if(!m_currentTask)
		throw RunTimeException(WHERE, "LowLevelSearch must have the current task set.");


	m_currentRobot = _start.GetRobot();
	auto agent = m_currentRobot->GetAgent();
	m_currentMotionConstraints = &(m_motionConstraintMap[agent]);


  if(_start.GetRobot()->GetCapability() != _goal.GetRobot()->GetCapability()){
    throw RunTimeException(WHERE, "start and goal are of mismatched robot types.");
  }

	//Setup Query info
	//auto query = dynamic_cast<QueryMethod<MPTraits<Cfg,DefaultWeight<Cfg>>>*>(
	//								m_tmpLibrary->GetMPLibrary()->GetMapEvaluator(m_queryLabel).get());

	auto query2 = dynamic_cast<QueryMethod<MPTraits<Cfg,DefaultWeight<Cfg>>>*>(
									m_tmpLibrary->GetMPLibrary()->GetMapEvaluator("TwoVariableQuery").get());

  //auto upper = m_currentMotionConstraints->upper_bound(MAX_DBL);
	//double lastConstraint = upper->first;
	double lastConstraint = 0;
	for(auto iter = m_currentMotionConstraints->begin(); iter != m_currentMotionConstraints->end(); iter++) {
		if(iter->first > lastConstraint)
			lastConstraint = iter->first;
	}

	//query->SetStartTime(_startTime);
	//query->SetEndTime(_minEndTime);
	//query->SetLastConstraintTime(lastConstraint);
	//auto timeRes = m_tmpLibrary->GetMPProblem()->GetEnvironment()->GetTimeRes();
	query2->SetStartTime(_startTime);///timeRes);
	query2->SetEndTime(_minEndTime);///timeRes);
	query2->SetLastConstraintTime(lastConstraint);

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
/*
	// adjust constraints to account for starting time
	MotionConstraintSet* unadjustedConstraints;
	MotionConstraintSet constraints;
	for(auto kv : *m_currentMotionConstraints){
		constraints.insert(std::make_pair((kv.first-_startTime),kv.second));
	}
	m_currentMotionConstraints = &constraints;
*/
  //save current library task
  auto oldTask = m_tmpLibrary->GetMPLibrary()->GetTask();

	auto originalRobot = _start.GetRobot();

  auto dummyAgent = m_tmpLibrary->GetTaskPlan()->GetCapabilityAgent(_start.GetRobot()->GetCapability());
  auto robot = dummyAgent->GetRobot();

  _start.SetRobot(robot);
  _goal.SetRobot(robot);

  std::shared_ptr<MPTask> task = std::shared_ptr<MPTask>(new MPTask(robot));

  std::unique_ptr<CSpaceConstraint> startConstraint(new CSpaceConstraint(robot, _start));
  std::unique_ptr<CSpaceConstraint> goalConstraint(new CSpaceConstraint(robot, _goal));

  task->SetStartConstraint(std::move(startConstraint));
  task->AddGoalConstraint(std::move(goalConstraint));

  m_tmpLibrary->GetMPLibrary()->SetTask(task.get());

  auto solution = new MPSolution(dummyAgent->GetRobot());

	auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(robot->GetAgent()));

  solution->SetRoadmap(dummyAgent->GetRobot(),roadmap.get());

	size_t goalLastConstraint = 0;

	_goal.SetRobot(originalRobot);
	auto goalVID = roadmap->GetVID(_goal);
  for(auto iter = m_currentMotionConstraints->begin(); iter != m_currentMotionConstraints->end(); ++iter) {
    const size_t timestep = iter->first;
		auto cfg = iter->second.first;
		bool valid = m_validVertices[_currentTask][originalRobot->GetAgent()][goalVID].count(
															std::make_pair(timestep,cfg));
		if(valid) {
			continue;
		}
		bool invalid = m_invalidVertices[_currentTask][originalRobot->GetAgent()][goalVID].count(
														std::make_pair(timestep,cfg));
		if(invalid and timestep > goalLastConstraint and timestep <= _minEndTime) {
			goalLastConstraint = timestep;
		}
	}

	query2->SetLastGoalConstraintTime(goalLastConstraint);
	//Try to solve it without considering time first
  //m_tmpLibrary->GetMPLibrary()->Solve(m_tmpLibrary->GetMPLibrary()->GetMPProblem(), 
	//		task.get(), solution, "EvaluateMapStrategy", LRand(), "LowerLevelGraphWeight");

	//If no solution is found, use the two varibale state search
  //if(m_tmpLibrary->GetMPLibrary()->GetPath()->Cfgs().empty())
  	m_tmpLibrary->GetMPLibrary()->Solve(m_tmpLibrary->GetMPLibrary()->GetMPProblem(), 
				task.get(), solution, "TwoVariableStrategy", LRand(), "LowerLevelGraphWeight");
		

  if(m_tmpLibrary->GetMPLibrary()->GetPath()->Cfgs().empty())
    return {};

  //restore library task
  m_tmpLibrary->GetMPLibrary()->SetTask(oldTask);
	//m_currentMotionConstraints = unadjustedConstraints;
	

	m_currentRobot = nullptr;
	m_currentMotionConstraints = nullptr;


	return std::make_pair(double(solution->GetPath()->TimeSteps())+_startTime,
												std::make_pair(solution->GetPath(robot)->VIDs(),
																			solution->GetPath()->GetFinalWaitTimeSteps()));
}


/*------------------------------- Helper Functions ---------------------------------*/

double
LowLevelSearch::
MultiRobotPathWeight(typename Roadmap::adj_edge_iterator& _ei,
    const double _startTime, const double _bestEndTime) {
	
  // Compute the time when we will end this edge.
  const size_t startTime = static_cast<size_t>(std::llround(_startTime)),
               endTime   = startTime + _ei->property().GetTimeSteps();

  // If this end time isn't better than the current best, we won't use it. Return
  // without checking conflicts to save computation.
  if(endTime >= static_cast<size_t>(std::llround(_bestEndTime)))
    return endTime;

  // If there are no current conflicts, there is nothing to check.
  if(!m_currentMotionConstraints)
    return endTime;

  // There is at least one conflict. Find the set which occurs between this
  // edge's start and end time.
  /*auto lower = m_currentMotionConstraints->lower_bound(startTime),
       //upper = m_currentMotionConstraints->upper_bound(endTime);
       upper = m_currentMotionConstraints->upper_bound(startTime);

  // If all of the conflicts happen before or after now, there is nothing to
  // check.
  const bool beforeNow = lower == m_currentMotionConstraints->end();
  if(beforeNow)
    return endTime;

  const bool afterNow = upper->first > endTime;
//upper == m_currentMotionConstraints->begin();
  if(afterNow)
    return endTime;
*/
  // Check the conflict set to see if this edge hits any of them.
  //for(auto iter = lower; iter != upper; ++iter) {
  for(auto iter = m_currentMotionConstraints->begin(); iter != m_currentMotionConstraints->end(); ++iter) {
    // Unpack the conflict data.
    const size_t timestep = iter->first;

		if(timestep < startTime or timestep+iter->second.second > endTime)
			continue;

    Cfg cfg    = iter->second.first;

    // Assert that the conflict occurs during this edge transition (remove this
    // later once we're sure it works right).
    const bool rightTime = startTime <= timestep and timestep <= endTime;
    if(!rightTime)
      throw RunTimeException(WHERE) << "The conflict set should only include "
                                    << "conflicts that occur during this range.";

		//Try to cache these computations
		auto p = std::make_pair(_ei->source(),_ei->target());

		//auto invalids = m_invalidEdges[m_currentTask][m_currentRobot->GetAgent()][cfg][p.first];
		auto invalids = m_invalidEdges[m_currentTask][m_currentRobot->GetAgent()][p.first][p.second];
		bool invalid = invalids.count(cfg);

		bool valid = m_validEdges[m_currentTask][m_currentRobot->GetAgent()][p.first][p.second].count(cfg);
		if(valid) {
			if(invalid)
				throw RunTimeException(WHERE,"Something is seriously wrong.");
			continue;
		}
    // Check if the conflict cfg hits this edge.
    const bool hitsEdge = invalid
								or !IsEdgeSafe(_ei->source(), _ei->target(), cfg);
    if(!hitsEdge) {
			m_validEdges[m_currentTask][m_currentRobot->GetAgent()][p.first][p.second].insert(cfg);
      continue;
		}
		m_invalidEdges[m_currentTask][m_currentRobot->GetAgent()][p.first][p.second].insert(cfg);

    //if(this->m_debug)
    if(false)
      std::cout << "\t\t\t\tEdge (" << _ei->source() << ","
                << _ei->target() << ") collides against robot "
                << cfg.GetRobot()->GetLabel()
                << " at " << cfg.PrettyPrint()
								<< " at time " << _startTime
                << std::endl;

    // The conflict blocks this edge.
    return std::numeric_limits<double>::infinity();
  }

  // There is no conflict and the end time is better!
  return endTime;
	return 0;
}


bool
LowLevelSearch::
IsEdgeSafe(const size_t _source, const size_t _target, const Cfg& _conflictCfg)
    const {

  auto robot = m_currentRobot;

	if(!m_currentRobot or _conflictCfg.GetRobot() == robot) 
		throw RunTimeException(WHERE,"Robot being searched for is not properly set.");

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(robot->GetAgent())).get();

  // Reconstruct the edge path at resolution-level.
  std::vector<Cfg> path;
  path.push_back(roadmap->GetVertex(_source));
  std::vector<Cfg> edge = m_tmpLibrary->GetMPLibrary()->ReconstructEdge(
      roadmap, _source, _target);
  path.insert(path.end(), edge.begin(), edge.end());
  path.push_back(roadmap->GetVertex(_target));

  // Get the valididty checker and make sure it has type
  // CollisionDetectionValidity.
  /// @TODO Figure out how to avoid needing this downcast so that we can
  ///       leverage more efficient compose checks (like checking the bounding
  ///       spheres first).
  auto basevc = m_tmpLibrary->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits<Cfg,DefaultWeight<Cfg>>>*>(basevc.get());

  // Configure the other robot at _conflictCfg.
  auto otherMultiBody = _conflictCfg.GetRobot()->GetMultiBody();
  otherMultiBody->Configure(_conflictCfg);

  // Check each configuration in the resolution-level path for collision with
  // _conflictCfg.
  CDInfo cdInfo;
  auto thisMultiBody = robot->GetMultiBody();
  for(const Cfg& cfg : path) {
    thisMultiBody->Configure(cfg);
    if(vc->IsMultiBodyCollision(cdInfo, thisMultiBody, otherMultiBody,
        "Edge validaiton in General CBS low level search."))
      return false;
  }

  // If we haven't detected a collision, the edge is safe.
  return true;
}

/*----------------------------------------------------------------------------*/
