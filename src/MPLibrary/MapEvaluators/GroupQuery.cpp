#include "GroupQuery.h"

#include "MPLibrary/MPLibrary.h"
#include "MPProblem/DynamicObstacle.h"
#include "Utilities/MetricUtils.h"

/*----------------------------- Construction ---------------------------------*/

GroupQuery::
GroupQuery() : MapEvaluatorMethod() {
  this->SetName("GroupQuery");
}


GroupQuery::
GroupQuery(XMLNode& _node) : MapEvaluatorMethod(_node) {
  this->SetName("GroupQuery");

  m_vcLabel = _node.Read("vcLabel",false,"",
                         "Collision Detection method to use for dynamic obstacles.");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

void
GroupQuery::
Initialize() {
  // Clear previous state.
  Reset(nullptr);

  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::FoundPath", 0);
}

/*-------------------------- MapEvaluator Overrides --------------------------*/

bool
GroupQuery::
operator()() {
  auto goalTracker = this->GetMPLibrary()->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();
  auto task = this->GetGroupTask();
  const size_t numGoals = task->GetNumGoals();

  // Record the path length history when this function exits.
  nonstd::call_on_destruct trackHistory([this](){
      this->GetStatClass()->AddToHistory("pathlength",
          this->GetGroupPath()->Length());
  });

  // If no goals remain, then this must be a refinement step (as in optimal
  // planning). In this case or the roadmap has changed, reinitialize and
  // rebuild the whole path.
  auto r = this->GetGroupRoadmap();
  if(unreachedGoals.empty() or r != m_roadmap)
    Reset(r);

  if(this->m_debug)
    std::cout << "Querying group roadmap for a path satisfying task '"
              << task->GetLabel()
              << "', " << unreachedGoals.size() << " / " << numGoals
              << " goals not reached."
              << "\n\tTrying to connect goal: " << m_goalIndex
              << "\n\tUnreached goals: " << unreachedGoals
              << std::endl;

  // Search for a sequential path through each task constraint in order.
  auto path = this->GetGroupPath();
  for(; m_goalIndex < numGoals; ++m_goalIndex) {
    // If this goal constraint is unreached, the query cannot succeed.
    auto iter = std::find(unreachedGoals.begin(), unreachedGoals.end(),
        m_goalIndex);
    const bool unreached = iter != unreachedGoals.end();
    if(unreached) {
      if(this->m_debug)
        std::cout << "\tGoal " << m_goalIndex << " has no satisfying VIDs."
                  << std::endl;
      return false;
    }

    // Get the start VID for this subquery.
    VID start = SIZE_MAX;
    if(!path->Empty()) {
      path->VIDs().back();
    }
    else {
      for(auto vid : goalTracker->GetStartVIDs()) {
        //if(SameFormations(r->GetVertex(vid).GetFormations(),r->GetActiveFormations())) {
          start = vid;
          break;
        //}
      }
    }

    if(start == SIZE_MAX)
      throw RunTimeException(WHERE) << "No VIDs located for start.";

    // Get the goal VIDs for this subquery.
    const VIDSet& goals = goalTracker->GetGoalVIDs(m_goalIndex);
    if(goals.empty())
      throw RunTimeException(WHERE) << "No VIDs located for reached goal "
                                    << m_goalIndex << ".";

    if(this->m_debug)
      std::cout << "\tEvaluating sub-query from " << start << " to " << goals
                << "." << std::endl;

    // Warn users if multiple goals are found.
    if(goals.size() > 1 and numGoals > 1)
      std::cerr << "\tWarning: subquery has " << goals.size() << " possible VIDs "
                << "for goal " << m_goalIndex << "/" << numGoals
                << ". The algorithm will try its best but isn't complete for "
                << "this case." << std::endl;

    // Perform this subquery. If it fails, there is no path.
    const bool success = PerformSubQuery(start, goals);
    if(!success)
      return false;
  }

  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::FoundPath", 1);

  if(this->m_debug)
    std::cout << "\tConnected all goals!" << std::endl;

  return true;
}

/*--------------------------- Query Interface --------------------------------*/

std::vector<typename GroupQuery::VID>
GroupQuery::
GeneratePath(const VID _start, const VIDSet& _goals) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "GroupQuery::GeneratePath");

  if(this->m_debug)
    std::cout << "Generating path from " << _start << " to " << _goals << "."
              << std::endl;

  // Check for trivial path.
  if(_goals.count(_start))
    return {_start};

  stats->IncStat("Graph Search");

  // Set up the termination criterion to quit early if we find the _end node.
  SSSPTerminationCriterion<GroupRoadmapType> termination(
      [_goals](typename GroupRoadmapType::vertex_iterator& _vi,
             const SSSPOutput<GroupRoadmapType>& _sssp) {
        return _goals.count(_vi->descriptor()) ? SSSPTermination::EndSearch
                                               : SSSPTermination::Continue;
      }
  );

  // Set up the path weight function depending on whether we have any dynamic
  // obstacles.
  SSSPPathWeightFunction<GroupRoadmapType> weight;
  if(m_weightFunction) {
    weight = m_weightFunction;
  }
  else if(!this->GetMPProblem()->GetDynamicObstacles().empty()) {
    weight = [this](typename GroupRoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->DynamicPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }
  else {
    weight = [this](typename GroupRoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->StaticPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }

  // Run dijkstra's algorithm to find the path, if it exists.
  auto g = this->GetGroupRoadmap();
  const SSSPOutput<GroupRoadmapType> sssp = DijkstraSSSP(g, {_start}, weight,
      termination);

  // Find the last discovered node, which should be a goal if there is a valid
  // path.
  const VID last = sssp.ordering.back();
  if(!_goals.count(last))
    return {};

  // Extract the path.
  std::vector<VID> path;
  path.push_back(last);

  VID current = last;
  do {
    current = sssp.parent.at(current);
    path.push_back(current);
  } while(current != _start);
  std::reverse(path.begin(), path.end());

  return path;
}


void
GroupQuery::
SetPathWeightFunction(SSSPPathWeightFunction<GroupRoadmapType> _f) {
  m_weightFunction = _f;
}

/*--------------------------------- Helpers ----------------------------------*/

void
GroupQuery::
Reset(GroupRoadmapType* const _r) {
  // Set the roadmap.
  m_roadmap = _r;

  // Reset the goal index.
  m_goalIndex = 0;

  // Reset the path.
  auto path = this->GetGroupPath();
  if(path)
    path->Clear();
}


bool
GroupQuery::
PerformSubQuery(const VID _start, const VIDSet& _goals) {
  // Try to generate a path from _start to _goal.
  auto path = this->GeneratePath(_start, _goals);

  // If the path isn't empty, we succeeded.
  if(!path.empty()) {
    *this->GetGroupPath() += path;
    const VID goalVID = path.back();

    if(this->m_debug)
      std::cout << "\tSuccess: reached goal node " << goalVID << "."
                << std::endl;

    return true;
  }
  else if(this->m_debug)
    std::cout << "\tFailed: could not find a path." << std::endl;

  return false;
}


double
GroupQuery::
StaticPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  const double edgeWeight  = _ei->property().GetWeight(),
               newDistance = _sourceDistance + edgeWeight;
  return newDistance;
}


double
GroupQuery::
DynamicPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {

  // Get edge variables
  auto edge = _ei->property();
  auto source = _ei->source();
  auto target = _ei->target();

  // Assuming that _sourceDistance is in timesteps
  size_t start = size_t(_sourceDistance);
  size_t timesteps = size_t(edge.GetWeight());

  auto cfgs = edge.GetIntermediates().empty() ? std::vector<GroupCfgType>()
                                              : edge.GetIntermediates();

  if(cfgs.empty()) {
    auto rm = this->GetGroupRoadmap();
    auto lib = this->GetMPLibrary();
    cfgs = lib->ReconstructEdge(rm,source,target);
  }

  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto cd = dynamic_cast<CollisionDetectionValidity*>(vc);
  auto prob = this->GetMPProblem();
  auto dynamicObstacles = prob->GetDynamicObstacles();

  for(size_t i = 0; i < timesteps; i++) {
    auto cfg1 = cfgs[i];
    cfg1.ConfigureRobot();

    for(auto& obs : dynamicObstacles) {
      if(start < obs.GetStartTime())
        continue;

      const auto& path = obs.GetPath();
      const size_t index = start - obs.GetStartTime() + i;

      const Cfg& cfg2 = index < path.size() ? path[index]
                                            : path.back();
      cfg2.ConfigureRobot();
      auto mb2 = cfg2.GetMultiBody();

      for(auto robot : cfg1.GetRobots()) {
        auto mb1 = robot->GetMultiBody();
       
        CDInfo cdInfo; 
        if(cd->IsMultiBodyCollision(cdInfo,mb1,mb2,this->GetNameAndLabel())) {
          if(this->m_debug) {
            std::cout << "Found collision between " 
                      << robot->GetLabel()
                      << " and dynamic obstacle "
                      << cfg2.GetRobot()->GetLabel()
                      << " at timestep "
                      << start + i
                      << "."
                      << std::endl;
          }
  
          return std::numeric_limits<double>::infinity();
        }
      }
    }
  }

  const double edgeWeight  = _ei->property().GetWeight(),
               newDistance = _sourceDistance + edgeWeight;
  return newDistance;
}

/*
bool
GroupQuery::
SameFormations(std::unordered_set<Formation*> _set1, std::unordered_set<Formation*> _set2) {
  
  bool matched = true;

  for(auto f1 : _set1) {
    matched = false;
    for(auto f2 : _set2) {
      if(*f1 == *f2) {
        matched = true;
        break;
      }
    }

    if(!matched)
      break;
  }

  if(!matched)
    return std::numeric_limits<double>::infinity();

  for(auto f2 : _set2) {
    matched = false;
    for(auto f1 : _set1) {
      if(*f2 == *f1) {
        matched = true;
        break;
      }
    }

    if(!matched)
      break;
  }

  if(!matched)
    return false;

  return true;
}*/

/*----------------------------------------------------------------------------*/
