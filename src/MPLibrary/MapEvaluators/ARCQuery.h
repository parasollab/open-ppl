#ifndef PMPL_ARC_QUERY_H_
#define PMPL_ARC_QUERY_H_

#include "MapEvaluatorMethod.h"
#include "QueryMethod.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
// #include "MPLibrary/MPStrategies/ARCStrategy.h"
#include "ConfigurationSpace/Cfg.h"

#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// This method is a refactoring of the Local Repair code. Now it should be more
/// organized and structured.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ARCQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType        RoadmapType;
    typedef typename MPTraits::GroupRoadmapType   GroupRoadmapType;
    typedef typename MPTraits::CfgType            CfgType;
    typedef typename MPTraits::Path               Path;
    typedef typename RoadmapType::VID             VID;
    typedef typename RoadmapType::EdgeID          EdgeID;

    ///@}

  //private:
  public:
    ///@name Internal Types
    ///@{

    /// A Conflict decribes an inter-robot collision between two or different
    /// robots at a given time.
    struct Conflict {
        std::vector<CfgType> cfgs; ///< The configurations of all robots.
        size_t  timestep; ///< The timestep when the collision occurred.

        // /// @todo This is to support old gcc v4.x, replace with
        // ///       default-constructed class members after we upgrade.
        Conflict() : 
          cfgs(std::vector<CfgType>()), timestep(0) {}

        Conflict(std::vector<CfgType> _cfgs, size_t _timestep) :
          cfgs(_cfgs), timestep(_timestep) {}

        /// @return True if this is an empty conflict.
        bool Empty() const noexcept {
          return cfgs.empty();
        }

        /// Order by timestep.
        bool operator<(const Conflict& _other) const noexcept {
          return timestep < _other.timestep;
        }

        bool operator==(const Conflict& _other) const noexcept {
          return cfgs == _other.cfgs &&
                timestep == _other.timestep;
        }
    };

    /// A solution to the query is a path for each robot. These are implemented
    /// with shared_ptr because we may have many copies of a particular path at
    /// once.
    typedef std::map<Robot*, std::shared_ptr<Path>> Solution;

    ///@}

  //public:

    ///@name Construction
    ///@{

    ARCQuery();

    ARCQuery(XMLNode& _node);

    virtual ~ARCQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    virtual bool operator()() override;

    ///@}
    ///@name Accessors
    ///@{

    Conflict GetCurrentConflict();

    Conflict GetLastConflict();

    RobotGroup* GetRobotGroup();

    /// Store a valid solution in the MPLibrary's solution object.
    /// @param _solution The solution paths to store.
    // void SetSolution(Solution&& _solution);

    void SetRepairFailed();

    ///}

  protected:

    ///@name Helpers
    ///@{

    /// Compute the intial set of individual paths
    /// @return The initial solution.
    Solution ConstructInitialSolution();

    /// Compute the total cost of the paths in a solution.
    /// @return The total cost of the solution.
    double ComputeCost(const Solution& _solution);

    /// Compute a path for an individual robot which avoids a set of conflicts.
    /// @param _robot     The individual robot.
    /// @param _conflicts The conflicts to avoid.
    /// @return A path if found, or empty pointer if not.
    std::shared_ptr<Path> SolveIndividualTask(Robot* const _robot);

    /// Check for inter-robot collisions in a solution.
    /// @param _solution The solution to check.
    /// @return A discovered conflict, or an empty one if none was found.
    Conflict FindConflict(size_t latestTimestep);

    std::map<Robot*,Path*> CollectPaths(Solution _solution);

    ///@}
    ///@name Internal State
    ///@{

    // GroupTask* m_groupTask;    ///< The global group task we're working on.

    // RobotGroup* m_group;       ///< The robot grup of the global task.

    // GroupRoadmapType* m_groupRoadmap;  /// The roadmap group of the global task.

    std::string m_queryLabel;  ///< Query method for making individual plans.

    std::string m_vcLabel;     ///< Validity checker for conflict detection.

    std::string m_arcStrategyLabel;

    /// A map from robot to task.
    std::unordered_map<Robot*, MPTask*> m_taskMap;

    /// The repairing strategy we are using
    // std::string m_arcStrategyLabel;

    /// Keeping track of all inter-robot conflict we've seen
    std::vector<Conflict> m_conflictCache;

    std::set<pair<Robot*,Robot*>> m_conflictingRobots;

    /// Keeping track of the current conflict
    Conflict m_currentConflict;

    size_t m_latestTimestep{0};

    Solution m_solution; 

    /// Whether we failed at either finding local or global solution
    bool m_repairFailed{false};   

    /// Keep a record of the robot group
    RobotGroup* m_group;


    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ARCQuery<MPTraits>::
ARCQuery() {
  this->SetName("ARCQuery");
}


template <typename MPTraits>
ARCQuery<MPTraits>::
ARCQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("ARCQuery");
  std::cout << "\033[0;1;32m" "ARCQuery Constructor called for " << this->GetLabel() << "\033[0m" << std::endl;

  m_arcStrategyLabel = _node.Read("arcStrategyLabel", true, "",
      "The arc strategy method.");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "The validity checker for conflict detection. Must be a CD type.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
ARCQuery<MPTraits>::
Initialize() {
  if(this->m_debug) {
    std::cout << "\033[0;1;32m" "ARCQuery Initialize() Called for "
      << this->GetLabel() << "\033[0m" << std::endl;
  }

  // Assert that the validity checker is an instance of collision detection
  // validity.
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(
      this->GetValidityChecker(m_vcLabel)
  );
  if(!vc)
    throw RunTimeException(WHERE) << "Validity checker " << m_vcLabel
                                  << " is not of type "
                                  << "CollisionDetectionValidity.";

  m_group = this->GetGroupTask()->GetRobotGroup();

}

/*-------------------------- MapEvaluator Overrides --------------------------*/

template <typename MPTraits>
bool
ARCQuery<MPTraits>::
operator()() {

  m_currentConflict = FindConflict(m_latestTimestep);
  if(!m_currentConflict.Empty()) {
    m_conflictCache.push_back(m_currentConflict);
    return false; 
  }

  this->GetStatClass()->IncStat(this->GetNameAndLabel()+"::CollisionFound");

  // Check if we have seen this conflict, if so then we declare repair as failed
  // and resample
  for(auto c : m_conflictCache) {
    if(c == m_currentConflict) {
      if(this->m_debug) {
        std::cout << "\033[0;1;31m" "Repair failed! Due to duplicate conflicts!" "\033[0m" << std::endl;
      }
      m_conflictCache.clear();
      return false;
    }
  }  

  // Check if repair failed due to overpassing window or any other reasons
  if(m_repairFailed) {
    if(this->m_debug) {
      std::cout << "\033[0;1;31m" "Repair failed! Due to over passing window!" "\033[0m" << std::endl;
    }
    return false;
  }


  
  return true;


}




/*--------------------------------- Accessors ---------------------------------*/
template <typename MPTraits>
typename ARCQuery<MPTraits>::Conflict
ARCQuery<MPTraits>::
GetCurrentConflict() {
  return m_currentConflict;
}

template <typename MPTraits>
RobotGroup*
ARCQuery<MPTraits>::
GetRobotGroup() {
  return m_group;
}

template <typename MPTraits>
typename ARCQuery<MPTraits>::Conflict
ARCQuery<MPTraits>::
GetLastConflict() {
  if(m_conflictCache.empty() || m_conflictCache.size() == 1) return Conflict();
  return m_conflictCache[m_conflictCache.size() - 2];
}


// template <typename MPTraits>
// void
// ARCQuery<MPTraits>::
// SetSolution(Solution&& _solution) {
//   for(auto& pair : _solution) {
//     auto robot = pair.first;
//     auto& path = pair.second;

//     // Move the solution path into the MPSolution object.
//     auto currentPath = this->GetPath(robot);
//     *currentPath = std::move(*path);
//   }
// }

template <typename MPTraits>
void
ARCQuery<MPTraits>::
SetRepairFailed() {
  m_repairFailed = true;
}




/*--------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
std::map<Robot*,typename MPTraits::Path*>
ARCQuery<MPTraits>::
CollectPaths(ARCQuery<MPTraits>::Solution _solution) {
  std::map<Robot*, Path*> globalPaths;
  for(auto pairIter : _solution) {
    auto robot = pairIter.first;
    auto pathPtr = pairIter.second;
    globalPaths[robot] = pathPtr.get();
  }
  return globalPaths;
}



template <typename MPTraits>
double
ARCQuery<MPTraits>::
ComputeCost(const Solution& _solution) {
  double cost = 0;
  for(const auto& pair : _solution)
    cost += pair.second->Length();
  return cost;
}



// We wanto have a safe copy of the original FindConflict() function
template <typename MPTraits>
typename ARCQuery<MPTraits>::Conflict
ARCQuery<MPTraits>::
FindConflict(size_t latestTimestep) {

  // if(_solution.empty()) 
  //   throw RunTimeException(WHERE) << "Current solution is empty."
  //                                 << " Need to find a solution first"
  //                                 << " before checking for conflicts.";


  MethodTimer mt(this->GetStatClass(),
    this->GetNameAndLabel() + "::ValidationFunction");

  // Recreate each path at resolution level.
  std::map<Robot*, std::vector<CfgType>> cfgPaths;
  for(Robot* const robot : *m_group) {
    // Robot* const robot = pair.first;
    auto path = this->GetPath(robot);
    cfgPaths[robot] = path->FullCfgs(this->GetMPLibrary());
  }

  // Find the latest timestep in which a robot is still moving.
  size_t lastTimestep = 0;
  for(auto& path : cfgPaths)
    lastTimestep = std::max(lastTimestep, path.second.size());

  auto vc = static_cast<CollisionDetectionValidity<MPTraits>*>(
      this->GetValidityChecker(m_vcLabel)
  );

  // Step through each timestep.
  for(size_t t = latestTimestep; t < lastTimestep; ++t) {
    // Collision check each robot path against all others at this timestep.
    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end();) {
      // Configure the first robot at the approriate configuration.
      auto robot1        = iter1->first;
      const auto& path1  = iter1->second;
      const size_t step1 = std::min(t, path1.size() - 1);
      const auto& cfg1   = path1[step1];
      auto multibody1    = robot1->GetMultiBody();
      multibody1->Configure(cfg1);

      // Compare to all remaining robots.
      for(auto iter2 = ++iter1; iter2 != cfgPaths.end(); ++iter2) {
        // Configure the second robot at the appropriate configuration.
        auto robot2        = iter2->first;
        const auto& path2  = iter2->second;
        const size_t step2 = std::min(t, path2.size() - 1);
        const auto& cfg2   = path2[step2];
        auto multibody2    = robot2->GetMultiBody();
        multibody2->Configure(cfg2);

        // Check for collision. If none, move on.
        CDInfo cdInfo;

        this->GetStatClass()->IncStat("Number of multibody collision calls");
        const bool collision = vc->IsMultiBodyCollision(cdInfo,
            multibody1, multibody2, this->GetNameAndLabel());

        if(!collision)
          continue;

        if(this->m_debug) {
          std::cout << "\t\tConflict detected at timestemp " << t
                    << " (time " << this->GetEnvironment()->GetTimeRes() * t
                    << ")."
                    << "\n\t\t\tRobot " << robot1->GetLabel() << ": "
                    << cfg1.PrettyPrint()
                    << "\n\t\t\tRobot " << robot2->GetLabel() << ": "
                    << cfg2.PrettyPrint()
                    << std::endl;
        }

        Conflict newConflict{{cfg1, cfg2}, t};

        return newConflict;
      }
    }
  }

  if(this->m_debug) {
    std::cout << "\t\tNo conflict detected." << std::endl;
  }

  // We didn't find a conflict, return an empty one.
  return {};
}




/*----------------------------------------------------------------------------*/

#endif
