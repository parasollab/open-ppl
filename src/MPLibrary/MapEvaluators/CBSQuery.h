#ifndef PMPL_CBS_QUERY_H_
#define PMPL_CBS_QUERY_H_

#include "MapEvaluatorMethod.h"
#include "QueryMethod.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "Utilities/CBS.h"

#include "ConfigurationSpace/Cfg.h"

#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>

template<typename MPTraits>
class CBSQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::Path          Path;
    typedef typename RoadmapType::VID        VID;
    typedef typename RoadmapType::EdgeID     EdgeID;


  private:

    struct Conflict {
      CfgType cfg1;     ///< The first robot's configuration.
      CfgType cfg2;     ///< The second robot's configuration.
      size_t  timestep; ///< The timestep when the collision occurred.

      /// @todo This is to support old gcc v4.x, replace with
      ///       default-constructed class members after we upgrade.
      Conflict(const CfgType& _cfg1 = CfgType(nullptr),
          const CfgType& _cfg2 = CfgType(nullptr),
          const size_t _timestep = 0) :
          cfg1(_cfg1),
          cfg2(_cfg2),
          timestep(_timestep)
      {}

      /// @return True if this is an empty conflict.
      bool Empty() const noexcept {
        return !cfg1.GetRobot();
      }
    };

    typedef std::pair<size_t, CfgType> Constraint;

    typedef std::set<Constraint> ConstraintSet;

    typedef std::unordered_map<MPTask*, ConstraintSet> ConstraintMap;

    typedef std::unordered_map<MPTask*, std::shared_ptr<Path>> SolutionMap;

    typedef CBSNode<MPTask*, Constraint, std::shared_ptr<Path>> CBSNodeType;


  public:

    ///@name Construction
    ///@{

    CBSQuery();

    CBSQuery(XMLNode& _node);

    virtual ~CBSQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    virtual bool operator()() override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    void ConstructInitialSolution(CBSTree& _tree);

    void CreateChildNode(Robot* const _robot, CfgType&& _cfg,
        const size_t _timestep, const CBSNodeType& _parent, CBSTree& _tree);

    double ComputeCost(const Solution& _solution);

    std::shared_ptr<Path> SolveIndividualTask(MPTask* const _task,
        const ConstraintMap& _constraintMap = {});

    void SetSolution(Solution&& _solution);

    std::pair<std::pair<MPTask*, MPTask*>, Conflict> FindConflict(const SolutionMap& _solution);

    double MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceTimestep, const double _bestTimestep) const;

    bool IsEdgeSafe(const VID _source, const VID _target,
        const CfgType& _conflictCfg) const;

    GroupTask* m_groupTask;    ///< The group task we're working on.

    std::string m_queryLabel;  ///< Query method for making individual plans.
    std::string m_vcLabel;     ///< Validity checker for conflict detection.
    std::string m_costLabel = "SOC";

    const ConstraintSet* m_currentConstraints{nullptr};
    std::set<ConstraintMap> m_conflictCache;


};

template <typename MPTraits>
CBSQuery<MPTraits>::
CBSQuery() {
  this->SetName("CBSQuery");
}


template <typename MPTraits>
CBSQuery<MPTraits>::
CBSQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("CBSQuery");

  m_queryLabel = _node.Read("queryLabel", true, "",
      "The individual query method. Must be derived from QueryMethod and "
      "should not be used by any other object.");

  m_nodeLimit = _node.Read("nodeLimit", false, m_nodeLimit,
      size_t(1), std::numeric_limits<size_t>::max(),
      "Maximum number of CBS nodes to expand before declaring failure.");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "The validity checker for conflict detection. Must be a CD type.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
CBSQuery<MPTraits>::
Initialize() {
  // Assert that the validity checker is an instance of collision detection
  // validity.
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(
      this->GetValidityChecker(m_vcLabel).get()
  );
  if(!vc)
    throw RunTimeException(WHERE) << "Validity checker " << m_vcLabel
                                  << " is not of type "
                                  << "CollisionDetectionValidity.";

  // Assert that the query evaluator is an instance of query method.
  auto query = dynamic_cast<QueryMethod<MPTraits>*>(
      this->GetMapEvaluator(m_queryLabel).get()
  );
  if(!query)
    throw RunTimeException(WHERE) << "Query method " << m_queryLabel
                                  << " is not of type QueryMethod."
                                  << std::endl;

  // Set the query method's path weight function.
  query->SetPathWeightFunction(
      [this](typename RoadmapType::adj_edge_iterator& _ei,
             const double _sourceDistance,
             const double _targetDistance) {
        return this->MultiRobotPathWeight(_ei, _sourceDistance, _targetDistance);
      }
  );
}

template <typename MPTraits>
bool
CBSQuery<MPTraits>::
operator()() {
  CBSLowLevelPlanner<MPTask, Constraint, std::shared_ptr<typename MPTraits::Path>>(
      [this](CBSNodeType& _node, MPTask* _task) {
        auto path = this->SolveIndividualTask(_task,_node.constraintMap);
        if(!path)
          return false;
        _node.solutionMap[_task] = path;
        return true;
      });
  CBSValidationFunction<MPTask, Constraint, std::shared_ptr<typename MPTraits::Path>>(
      [this](CBSNodeType& _node) {
        auto taskConflict = this->FindConflict(_node.solutionMap);
        std::vector<std::pair<MPTask*, Constraint>> constraints;
        if (taskConflict.first.first != nullptr) {
          constraints = {std::make_pair(taskConflict.first.first, std::make_pair(c.timestep, c.cfg2)), std::make_pair(taskConflict.first.second, std::make_pair(c.timestep, c.cfg1))};
        }
        return constraints;
      });

  CBSSplitNodeFunction<MPTask, Constraint, std::shared_ptr<typename MPTraits::Path>>(
        [this](CBSNodeType& _node, std::vector<std::pair<MPTask*, Constraint>> _constraints,
          CBSLowLevelPlanner<MPTask, Constraint, std::shared_ptr<typename MPTraits::Path>>_lowlevel,
          CBSCostFunction<MPTask, Constraint, std::shared_ptr<typename MPTraits::Path>> _cost) {

            std::vector<CBSNodeType> children;
            for (auto constraintPair : _constraints) {
              auto task = constraintPair.first;
              auto constraint = constraintPair.second;
              CBSNodeType child = _parent;
              if(child.constraintMap[task].find(constraint) == child.constraintMap[task].end()){
                if(this->m_debug)
                  std::cout << "\t\t\tConflict double-assigned to robot "
                            << _robot->GetLabel() << ", skipping."
                            << std::endl;
                continue;
              }
              child.constraintMap[task].insert(constraint);
              if(m_conflictCache.count(child.constraintMap)) {
                if (this->m_debug)
                  std::cout << "\t\t\tThis conflict set was already attempted, skipping."
                            << std::endl;
                continue;
              }
              m_conflictCache.insert(child.constraintMap);
              if (!_lowlevel(child, task))
                continue;

              child.cost = _cost(child, this->m_costLabel);
              children.push_back(child);

              if(this->m_debug)
                std::cout << "\t\t\tChild node created." << std::endl;
            }

            return children;

      });
  CBSCostFunction<MPTask, Constraint, std::sharedptr<typename MPTraits::Path>>(
      [](CBSNodeType& _node, std::string _costLabel) {
        double cost = 0;
        if (_costLabel == "SOC") {
          for (const auto& ts : solutionMap) {
            cost += ts.second->length()
          }
        } else {
          for (const auto& ts : solutionMap) {
            if (ts.second->length() > cost)
              cost = ts.second->length();
          }
        }
        return cost;
      });
}

template <typename MPTraits>
std::shared_ptr<typename MPTraits::Path>
GroupCBSQuery<MPTraits>::
SolveIndividualTask(MPTask* const _task, const ConstraintMap& _constraintMap) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "SolveIndividualTask");
  Robot* robot = _task->GetRobot();


  if(this->m_debug)
    std::cout << (_constraintMap.empty() ? "\t" : "\t\t\t")
              << "Solving task '" << _task->GetLabel() << "' (" << _task
              << ") for robot '" << robot->GetLabel() << "' (" << robot
              << ")."
              << std::endl;

  // Set the conflicts to avoid.
  if (!_constraintMap.empty())
    m_currentConstraints = &_constraintMap.at(robot);

  // Generate a path for this robot individually while avoiding the conflicts.
  this->GetMPLibrary()->SetTask(_task);
  auto query = this->GetMapEvaluator(m_queryLabel);
  const bool success = (*query)();
  this->GetMPLibrary()->SetTask(nullptr);

  // Clear the conflicts.
  m_currentConstraints = nullptr;

  if(this->m_debug)
    std::cout << (_constraintMap.empty() ? "\t\t" : "\t\t\t\t")
              << "Path for robot " << robot->GetLabel() << " was "
              << (success ? "" : "not ") << "found."
              << std::endl;

  // If we failed to find a path, return an empty pointer.
  if(!success)
    return {};

  // Otherwise, return a copy of the robot's path from the solution object.
  Path* path = this->GetPath(robot);
  auto out = std::shared_ptr<Path>(new Path(std::move(*path)));
  path->Clear();

  return out;
}

template <typename MPTraits>
std::pair<std::pair<MPTask*, MPTask*>, typename CBSQuery<MPTraits>::Conflict>
GroupCBSQuery<MPTraits>::
FindConflict(const SolutionMap& _solution) {
  // Recreate each path at resolution level.
  std::map<MPTask*, std::vector<CfgType>> cfgPaths;
  for(const auto& pair : _solution) {
    MPTask* const task = pair.first;
    const auto& path   = pair.second;
    cfgPaths[robot] = path->FullCfgs(this->GetMPLibrary());
  }

  // Find the latest timestep in which a robot is still moving.
  size_t lastTimestep = 0;
  for(auto& path : cfgPaths)
    lastTimestep = std::max(lastTimestep, path.second.size());

  auto vc = static_cast<CollisionDetectionValidity<MPTraits>*>(
      this->GetValidityChecker(m_vcLabel).get()
  );

  // Step through each timestep.
  for(size_t t = 0; t < lastTimestep; ++t) {
    // Collision check each robot path against all others at this timestep.
    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end();) {
      // Configure the first robot at the approriate configuration.
      auto task1         = iter1->first;
      const auto& path1  = iter1->second;
      const size_t step1 = std::min(t, path1.size() - 1);
      const auto& cfg1   = path1[step1];
      auto multibody1    = robot1->GetMultiBody();
      multibody1->Configure(cfg1);

      // Compare to all remaining robots.
      for(auto iter2 = ++iter1; iter2 != cfgPaths.end(); ++iter2) {
        // Configure the second robot at the appropriate configuration.
        auto task2         = iter2->first;
        const auto& path2  = iter2->second;
        const size_t step2 = std::min(t, path2.size() - 1);
        const auto& cfg2   = path2[step2];
        auto multibody2    = robot2->GetMultiBody();
        multibody2->Configure(cfg2);

        // Check for collision. If none, move on.
        CDInfo cdInfo;
        const bool collision = vc->IsMultiBodyCollision(cdInfo,
            multibody1, multibody2, this->GetNameAndLabel());
        if(!collision)
          continue;

        if(this->m_debug)
          std::cout << "\t\tConflict detected at timestemp " << t
                    << " (time " << this->GetEnvironment()->GetTimeRes() * t
                    << ")."
                    << "\n\t\t\tRobot " << task1->GetRobot()->GetLabel() << ": "
                    << cfg1.PrettyPrint()
                    << "\n\t\t\tRobot " << task2->GetRobot()->GetLabel() << ": "
                    << cfg2.PrettyPrint()
                    << std::endl;

				//Old block pre-merge with master, remove if compiles
				//TODO::Was getting some weird complication bug about explicit construction
        Conflict newConflict(cfg1,cfg2,t);
        std::pair<MPTask*, MPTask*> taskPair = std::make_pair(task1, task2);
        //newConflict.cfg1     = cfg1;
        //newConflict.cfg2     = cfg2;
        //newConflict.timestep = t;

        //Conflict newConflict{cfg1, cfg2, t};

        return std::make_pair(taskPair, newConflict);
      }
    }
  }

  if(this->m_debug)
    std::cout << "\t\tNo conflict detected." << std::endl;

  // We didn't find a conflict, return an empty one.
  // Again weird compilation - I'll figure these out later
  Conflict c;
  return std::make_pair(std::make_pair(nullptr, nullptr), c);
}


template <typename MPTraits>
CBSNodeType
GroupCBSQuery<MPTraits>::
CreateChildNode(Robot* const _robot, CfgType&& _cfg, const size_t _timestep,
    const CBSNodeType& _parent) {
  if(this->m_debug)
    std::cout << "\t\tAttempting to create CBS node with conflict on robot "
              << _robot->GetLabel() << " at timestep "
              << _timestep << " colliding against robot "
              << _cfg.GetRobot()->GetLabel()
              << std::endl;

  // Initialize the child node by copying the parent.
  CBSNodeType child = _parent;

  // Assert that we aren't adding a duplicate conflict, which should be
  // impossible with a correct implementation.

  auto bounds = child.conflicts[_robot].equal_range(_timestep);
  for(auto iter = bounds.first; iter != bounds.second; ++iter)
    if(iter->second == _cfg) {
      if(this->m_debug)
        std::cout << "\t\t\tConflict double-assigned to robot "
                  << _robot->GetLabel() << ", skipping."
                  << std::endl;
      return;
    }

  child.conflicts[_robot].emplace(_timestep, std::move(_cfg));

#if 0
  for(auto conflictSet : child.conflicts) {
    for(auto conflict : conflictSet.second) {
      std::cout << "\t\t\t\t\tConflict on robot " << conflictSet.first->GetLabel()
        << " at time " << conflict.first << " against robot "
        << conflict.second.GetRobot()->GetLabel() << " at cfg "
        << conflict.second.PrettyPrint() << std::endl;
    }
  }
#endif

  // If we've already seen this set of conflicts, don't check them again.
  if(m_conflictCache.count(child.conflicts)) {
    if(this->m_debug)
      std::cout << "\t\t\tThis conflict set was already attempted, skipping."
                << std::endl;
    return;
  }
  m_conflictCache.insert(child.conflicts);

  // Find a path for this robot. If it fails, discard the new node.
  auto path = SolveIndividualTask(_robot, child.conflicts);
  if(!path)
    return;

  child.solution[_robot] = path;
  child.cost = ComputeCost(child.solution);

  if(this->m_debug)
    std::cout << "\t\t\tChild node created." << std::endl;

  return child;
}

template <typename MPTraits>
double
GroupCBSQuery<MPTraits>::
MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _startTime, const double _bestEndTime) const {
  // Compute the time when we will end this edge.
  const size_t startTime = static_cast<size_t>(std::llround(_startTime)),
               endTime   = startTime + _ei->property().GetTimeSteps();

  // If this end time isn't better than the current best, we won't use it. Return
  // without checking conflicts to save computation.
  if(endTime >= static_cast<size_t>(std::llround(_bestEndTime)))
    return endTime;

  // If there are no current conflicts, there is nothing to check.
  if(!m_currentConstraints)
    return endTime;

  // There is at least one conflict. Find the set which occurs between this
  // edge's start and end time.
  auto lower = m_currentConstraints->lower_bound(startTime),
       upper = m_currentConstraints->upper_bound(endTime);

  // If all of the conflicts happen before or after now, there is nothing to
  // check.
  const bool beforeNow = lower == m_currentConstraints->end();
  if(beforeNow)
    return endTime;

  const bool afterNow = upper == m_currentConstraints->begin();
  if(afterNow)
    return endTime;

  // Check the conflict set to see if this edge hits any of them.
  for(auto iter = lower; iter != upper; ++iter) {
    // Unpack the conflict data.
    const size_t timestep = iter->first;
    const CfgType& cfg    = iter->second;

    // Assert that the conflict occurs during this edge transition (remove this
    // later once we're sure it works right).
    const bool rightTime = startTime <= timestep and timestep <= endTime;
    if(!rightTime)
      throw RunTimeException(WHERE) << "The conflict set should only include "
                                    << "conflicts that occur during this range.";

    // Check if the conflict cfg hits this edge.
    const bool hitsEdge = !IsEdgeSafe(_ei->source(), _ei->target(), cfg);
    if(!hitsEdge)
      continue;

    if(this->m_debug)
      std::cout << "\t\t\t\t\tEdge (" << _ei->source() << ","
                << _ei->target() << ") collides against robot "
                << cfg.GetRobot()->GetLabel()
                << " at " << cfg.PrettyPrint()
                << std::endl;

    // The conflict blocks this edge.
    return std::numeric_limits<double>::infinity();
  }

  // There is no conflict and the end time is better!
  return endTime;
}


template <typename MPTraits>
bool
GroupCBSQuery<MPTraits>::
IsEdgeSafe(const VID _source, const VID _target, const CfgType& _conflictCfg)
    const {
  auto robot = this->GetTask()->GetRobot();
  auto roadmap = this->GetRoadmap(robot);

  // Reconstruct the edge path at resolution-level.
  std::vector<CfgType> path;
  path.push_back(roadmap->GetVertex(_source));
  std::vector<CfgType> edge = this->GetMPLibrary()->ReconstructEdge(
      roadmap, _source, _target);
  path.insert(path.end(), edge.begin(), edge.end());
  path.push_back(roadmap->GetVertex(_target));

  // Get the valididty checker and make sure it has type
  // CollisionDetectionValidity.
  /// @TODO Figure out how to avoid needing this downcast so that we can
  ///       leverage more efficient compose checks (like checking the bounding
  ///       spheres first).
  auto basevc = this->GetValidityChecker(m_vcLabel);
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc.get());

  // Configure the other robot at _conflictCfg.
  auto otherMultiBody = _conflictCfg.GetRobot()->GetMultiBody();
  otherMultiBody->Configure(_conflictCfg);

  // Check each configuration in the resolution-level path for collision with
  // _conflictCfg.
  CDInfo cdInfo;
  auto thisMultiBody = robot->GetMultiBody();
  for(const CfgType& cfg : path) {
    thisMultiBody->Configure(cfg);
    if(vc->IsMultiBodyCollision(cdInfo, thisMultiBody, otherMultiBody,
        this->GetNameAndLabel()))
      return false;
  }

  // If we haven't detected a collision, the edge is safe.
  return true;
}

