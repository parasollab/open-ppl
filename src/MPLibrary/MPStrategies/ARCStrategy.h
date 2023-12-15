#ifndef PMPL_ARC_STRATEGY_H_
#define PMPL_ARC_STRATEGY_H_

#include "MPLibrary/MapEvaluators/ARCQuery.h"
#include "GroupDecoupledStrategy.h"
#include "MPProblem/MPTask.h"
#include "ConfigurationSpace/GenericStateGraph.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Utilities/MPUtils.h"
#include "Utilities/XMLNode.h"
#include "MPLibrary/MPStrategies/GroupPRM.h"
#include "MPLibrary/MapEvaluators/IterationCountEvaluator.h"


#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>
#include <cstddef>
#include <iostream>
#include <string>
#include <memory>
#include <unordered_map>

//////////////////////////////////////////////////////////////////////////////
/// Reworked ARC Strategy
///
//////////////////////////////////////////////////////////////////////////////

template<typename MPTraits>
class ARCStrategy : public GroupDecoupledStrategy<MPTraits> {

    public:

        
        ///@name Motion Planning Types
        ///@{

        
        typedef typename MPTraits::RoadmapType          RoadmapType;
        typedef typename MPTraits::GroupRoadmapType     GroupRoadmapType;
        typedef typename MPTraits::Path                 Path;
        typedef typename MPTraits::CfgType              CfgType;
        typedef typename RoadmapType::VID               VID;
        typedef typename RoadmapType::EdgeID            EdgeID;    

        ///@}
        ///@name Internal Types
        ///@{

        typedef typename ARCQuery<MPTraits>::Conflict      Conflict;

        /// A search element in the priority queue used for storing the conflict
        /// resolution history.
        struct element {
            std::set<Robot*> robots; ///< The robots involved in the conflict.
            Range<size_t> interval; ///< The conflict resolution interval.

            element(std::set<Robot*> _robots, Range<size_t> _interval)
                : robots(_robots), interval(_interval) {}

            /// Total ordering by increasing distance.
            bool operator>(const element& _other) const noexcept {
                return interval.min > _other.interval.min;
            }
        };


        ///@}
        ///@name Local Types
        ///@{

        /// A subquery composed of start cfg and goal cfg.
        typedef std::pair<CfgType,CfgType> QueryCfg;

        /// A pair of subqueries, one per each robot.
        typedef std::vector<QueryCfg> QueriesCfg;

        /// A vertex pair.
        typedef std::pair<VID,VID> PairVID;

        /// A vector of vertex pair.
        typedef std::vector<PairVID> PairVIDVector;

        // A pair of Paths.
        typedef std::pair<Path,Path> PathPair;

        /// A vector of pairs of Paths
        typedef std::vector<PathPair> PathPairVector;

        /// For mapping a robot to its local problem 
        typedef std::pair<QueriesCfg, std::map<Robot*, Boundary*>> LocalProblem;

        /// A solution to the query is a path for each robot. These are implemented
        /// with shared_ptr because we may have many copies of a particular path at
        /// once.
        typedef std::map<Robot*, std::shared_ptr<Path>> Solution;    

        ///@}
        ///@name Construction
        ///@{

        
        ARCStrategy();

        ARCStrategy(XMLNode& _node);

        virtual ~ARCStrategy() = default;


        ///@}
        ///@name MPStrategyMethod Overrides
        ///@{

        bool operator()();

        virtual void Initialize() override;

        virtual void Iterate() override;

        virtual void Finalize() override {GroupDecoupledStrategy<MPTraits>::Finalize();};

        ///@}
        ///@name Accessors
        ///@{

        /// Setting the current cbs iteration, we need this for evaluation purposes
        void CurrentRepairIteration(size_t _current);

        size_t GetAdjustedTimeWindow();
        size_t GetMexLocalPathLength();

        std::set<Robot*> GetCurrentConflictingRobots();

        Solution GetSolution();

        
        ///@}
        ///@name Repair
        ///@{

        bool Repair(Conflict& _conflict);

        ///@}



    private:

        ///@}
        ///@name Initial Global Solution Construction
        ///@{

        /// Compute the intial set of individual paths
        /// @return The initial solution.
        void ConstructInitialSolution();

        /// Compute a path for an individual robot which avoids a set of conflicts.
        /// @param _robot     The individual robot.
        /// @param _conflicts The conflicts to avoid.
        /// @return A path if found, or empty pointer if not.
        void SolveIndividualTask(Robot* const _robot, MPTask* const _task);

        /// Define a function for computing path weights w.r.t. multi-robot
        /// problems. Here the metric is the number of timesteps.
        /// For now, this function doeas the same as "StaticPathWeight".
        /// @param _ei             An iterator to the edge we are checking.
        /// @param _sourceTimestep The shortest time to the source node.
        /// @param _bestTimestep   The best known time to the target node.
        /// @return The time to the target node via this edge, or infinity if taking
        ///         this edge would result in a collision with dynamic obstacles.
        double MultiRobotPathWeightRepair(typename RoadmapType::adj_edge_iterator& _ei,
            const double _sourceTimestep, const double _bestTimestep) const;



        ///@}
        ///@name Local Problem Preprocessing
        ///@{

        // Runs Preprocessing
        QueriesCfg PreProcessing(Conflict _c, size_t _window);

        bool OverpassingWindow(typename ARCStrategy<MPTraits>::QueriesCfg _queries);

        // Runs ComputeQueryCfg() in all robots.
        std::pair<typename ARCStrategy<MPTraits>::QueriesCfg,std::vector<std::pair<size_t,size_t>>>
        ComputeQueriesCfg(Conflict _c, size_t _window);

        // Detects the corresponding cfgs for the local start and goal points.
        QueryCfg ComputeQueryCfg(CfgType& _cfg, size_t _timestep,  size_t _min, size_t _max);

        // Checks if queries are in collision
        bool ValidQueries(QueriesCfg& _queries);

        // Checks if a single pair of endpoints (start or goal) are in collision
        bool ValidEndpoints(CfgType& _cfg1, CfgType& _cfg2);

        // Check if the cfgs change when added to the roadmap
        /// NOTE: for some reason some start/goal cfgs change when added to the roadmap
        /// in some seeds run in manipulator experiments using decoupled and cbs as the
        /// local strategy (see MPStrategyMethod.h:467)
        bool ValidConstraints(QueriesCfg& _queries);

        // Check if each cfg changes when added to the roadmap
        bool ValidConstraint(CfgType& _cfg);

        // Runs AddingQueryCfg() in all robots.
        std::vector<std::pair<size_t,size_t>> AddingQueriesCfg(QueriesCfg _queries,std::vector<std::pair<size_t,size_t>> _windowTimesteps);

        // Adds the query to its corresponding roadmap. Returns boolean values if the
        // corresponding VIDs already exist in the roadmap.
        std::pair<size_t,size_t> AddingQueryCfg(QueryCfg _query, std::pair<size_t,size_t> _windowTimesteps);

        void AddNewEdge(RoadmapType* _rm, size_t _source, size_t _target);

        // Runs ComputeSubPathPair() in all robots.
        PathPairVector ComputeSubPathPairs(QueriesCfg _queries, std::vector<std::pair<size_t,size_t>> _windowTimesteps,
                                            PairVIDVector _pairsVID);

        // Computes the safe subpaths globalStart-timestepBefore and
        // timestepAfter-globalGoal.
        PathPair ComputeSubPathPair(QueryCfg _query, std::pair<size_t,size_t> _windowTimesteps,
                                    PairVID _pairVID);

        // Add zero wait timesteps to _path if _path has no waiting timesteps
        void AddZeroWaitTimes(Path* _path);

        void AddZeroWaitTimes(Path& _path);


        ///@}
        ///@name Local Problem Construction
        ///@{

        QueriesCfg LocalProblemInitialization(Conflict _c);

        std::vector<MPTask> CreateLocalTaskSet(QueriesCfg _queries);

        std::vector<Robot*> GetRobots(QueriesCfg _queries);

        std::vector<Robot*> GetRobots(GroupTask* _task);



        ///@}
        ///@name Local Problem Update
        ///@{

        void UpdateLocalProblem(std::string _expander);

        // Returns true if we did not violate another other intervals
        // Violations comes from asymmetric increment of max side of window
        bool UpdateConflictMap(Conflict& _conflict);

        // If we find another robots that's involved in the conflict we would update the conflict
        void MergeConflict(Conflict& _conflict);


        ///@}
        ///@name Local Solution Construction
        ///@{
        
        // Performs a resolution given the queries 
        bool SingleResolution(QueriesCfg& _queries, std::string _localStrategy);


        ///@}
        ///@name Local Solution Checking
        ///@{

        // Check for if the paths are empty; if so we generate local solutions, else we go to the next strategy
        bool IsValidSolution(RobotGroup* _robotGroup, std::vector<Robot*> _robots, std::string _localStrategy);

        // Checks for conflicts local solution against external paths. Returns true if it is
        // conflict free.
        bool PostValidation(Conflict& _conflict);

        std::vector<Robot*> SingleRobotPostValidation(Conflict& _c, Robot* _robot, std::map<Robot*,
            std::vector<CfgType>> _localPaths, std::map<Robot*,std::vector<CfgType>> _globalPaths);



        ///@}
        ///@name Local Solution Finalization
        ///@{

        void SetLocalSolution(Solution _localSolution);

        void GenerateDecoupled(RobotGroup* _group);

        void GenerateCoupled(RobotGroup* _group);

        ///@{
        ///@name Global Solution Update
        ///@}

        // Once the local path is computed, this function reassembles the sub paths
        // with the local repaired path.
        void ReassemblePaths(QueriesCfg& _queries, std::map<Robot*,std::shared_ptr<Path>> _localPaths);

        void ReassemblePath(Robot* _robot, Path* _localPath, std::pair<Path,Path> _safeSubPaths);

        ///@}
        ///@name Utilities
        ///@{

        /// For debugging/understanding purposes, print out the local
        /// roadmaps/paths

        void PrintLocalPath(Robot* _robot);

        void PrintLocalRoadmap(Robot* _robot);


        ///@}
        ///@name Internal State
        ///@{

        /// adaptive or hierarchical
        std::string m_behavior;

        /// local validity checker
        std::string m_vcLabel;

        /// Tthe query label for the initial global solution
        std::string m_queryLabel;

        /// local strategy
        std::string m_strategyLabel;

        /// the initial local problem time window size
        size_t m_initialWindowSize;

        /// local problem time window size
        size_t m_windowSize;

        /// scale factor of local problem c space size
        double m_radius;

        /// local region radius
        double m_localRadius;

        /// if we want to include the robot expansion action
        bool m_ifExpandRobot;

        /// if we want local repair to be hierachical
        bool m_ifHierarchical;

        /// increment for time window
        size_t m_queriesIncrement;

        /// increment for c space size
        double m_boundariesIncrement;

        /// maps a robot to it local path
        Solution m_localSolution;

        /// maps a robot to its local boundary
        std::map<Robot*,Boundary*> m_boundaries;

        /// maps a robot to its local boundingSphere
        /// NOTE: not sure what the best practice here is
        /// since we could have other types of cspace boundaries
        std::map<Robot*, CSpaceBoundingSphere*> m_boundingSpheres;

        //<strategy,<probability,weight>>
        std::vector<std::string> m_localStrategies;

        /// number of free space configurations sampled from the local strategy
        size_t m_validSamples;

        /// number of obstacle space configurations sampled from the local strategy
        size_t m_invalidSamples;

        /// cache for storing the last conflict
        Conflict m_lastConflict;

        /// cache for storing the current conflict, we want this for the LocalRepairQuery::FindConflict() function
        Conflict m_currentConflict;

        /// number of times we expanded
        size_t numExp{0};

        /// number of times to run the local decoupled strategy before terminating it
        /// defaulted to 6 for now
        size_t m_terminationCount{6};

        /// the current number of times the local decoupled strategy is being run
        size_t m_currentTerminationCount{0};

        /// if set to true, print out the local roadmap and local path for each robot
        /// for each conflict
        bool m_printLocal;

        double m_minRadius{1.0}; // TODO::Change this to be a factor of the bounding sphere
        // radius for the robot

        /// the iterationEval for local strategy
        std::string m_iterationEval;

        std::map<Robot*,
        std::priority_queue<element,std::vector<element>,std::greater<element>>>
        m_conflictIntervalMap;
        
        size_t m_increment;

        size_t m_conflictTimestep;

        size_t m_adjustedWindowSize{0};

        // we will try using the max lenght (in timesteps) to have a better knowledge
        // of the latest conflict free global timestep, to imporve find conflict
        // function.
        size_t m_maxLocalPathLength{0};

        size_t m_currentRepairIteration{0}; // The current cbs iteration.

        bool m_ifGlobal{false};

        PathPairVector m_safeSubPaths;

        size_t m_windowExpansions{0};

        // Solution m_solution;

        bool m_repairFailed{false};

        /// Whether we are finding the initial solution in this iteration
        size_t m_initial{0};

        /// The counter for sampling iterations
        size_t m_samplingCounter{0};

        /// The counter for repair iterations
        size_t m_repairCounter{0};

        /// Keep a record of the group task
        GroupTask* m_groupTask;

        ///@}



};




/*-------------------------------- Construction -------------------------------------*/

template<typename MPTraits>
ARCStrategy<MPTraits>::
ARCStrategy() {
    this->SetName("ARCStrategy");
}

template<typename MPTraits>
ARCStrategy<MPTraits>::
ARCStrategy(XMLNode& _node) : GroupDecoupledStrategy<MPTraits>(_node) {
  this->SetName("ARCStrategy");

  m_printLocal = _node.Read("printLocal", false, false,
      "If true, print out the local roadmap and path for each robot for each conflict");

  m_terminationCount = _node.Read("terminationCount",false, m_terminationCount,size_t(0), size_t(MAX_INT),
      "Number of times to run the local decoupled strategy before terminating it");

  m_ifExpandRobot = _node.Read("ifExpandRobot", false, false,
      "If we want to expand the set of robot or not");

  m_ifHierarchical = _node.Read("ifHierarchical", false, false,
      "If hierarchical, we want to run a sequence of local strategies read from Local Strategy; else we run the strategy read from strategyLabel");

  m_behavior = _node.Read("behavior", true, "",
      "The behavior to implmement expansion actions. Default behiavor is hierarchical");

  m_vcLabel = _node.Read("vcLabel", true, "",
    "The local VC method.");

  m_strategyLabel = _node.Read("strategyLabel", true, "",
    "The local mp strategy method.");

  m_radius = _node.Read("radius", true, m_radius,
    double(1), std::numeric_limits<double>::max(),
    "The radius scale factor of c-space bounding sphere");

  m_initialWindowSize = _node.Read("windowSize", true, m_windowSize,
    size_t(1), std::numeric_limits<size_t>::max(),
    "The window size");

  m_windowSize = m_initialWindowSize;

  m_increment = _node.Read("increment", true, m_increment,
    size_t(0), std::numeric_limits<size_t>::max(),
    "The increment for getting valid queries");

  m_minRadius = _node.Read("minRadius",false,m_minRadius,0.01,MAX_DBL,
    "Minimum radius for the local subproblem.");

  m_iterationEval = _node.Read("iterationEval", true, "",
    "The iterationEval for local strategies");

  m_queryLabel = _node.Read("queryLabel", true, "",
      "The individual query method. Must be derived from QueryMethod and "
      "should not be used by any other object.");

  size_t numExpanders = 0;
  for(auto& child : _node)
    if(child.Name() == "Expander")
      ++numExpanders;
  for(auto& child : _node)
    if(child.Name() == "Expander") {
      auto method = child.Read("label", true, "", "Expander Method");
      if(method == "queries")
        m_queriesIncrement = child.Read("increment",true, m_queriesIncrement, size_t(0), std::numeric_limits<size_t>::max(), "The increment");
      else if(method == "boundaries")
        m_boundariesIncrement = child.Read("increment",true, m_boundariesIncrement, double(0), std::numeric_limits<double>::max(), "The increment");
    }

  for(auto& child : _node)
    if(child.Name() == "LocalStrategy") {
      auto method = child.Read("label", true, "", "Local Strategy");
      m_localStrategies.push_back(method);
    }
}



/*--------------------- GroupDecoupledStrategy Overrides ----------------------*/

template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
operator()() {

  return true;
}

template<typename MPTraits>
void
ARCStrategy<MPTraits>::
Initialize() {
  
  this->GetStatClass()->SetStat(this->GetNameAndLabel()+"::CollisionFound", 0);

  GroupDecoupledStrategy<MPTraits>::Initialize();
  m_conflictIntervalMap.clear();

  m_groupTask = this->GetGroupTask();

}

template<typename MPTraits>
void
ARCStrategy<MPTraits>::
Iterate() {

  if(MPStrategyMethod<MPTraits>::m_meLabels.size() > 1)
    throw RunTimeException(WHERE) << "ARC currently doesn't support"
                                  << " multiple map evaluators.";

  auto arcQuery = dynamic_cast<ARCQuery<MPTraits>*>
    (this->GetMapEvaluator(MPStrategyMethod<MPTraits>::m_meLabels.front()));

  // typename ARCStrategy<MPTraits>::Solution globalSolution;

  // We will resample the roadmap if either it's the initial solution or if repair fails
  if(!m_initial || m_repairFailed) {
    if(!m_initial) m_initial++;
    GroupDecoupledStrategy<MPTraits>::Iterate();
    ConstructInitialSolution();

    ++m_samplingCounter;
    this->GetStatClass()->IncStat("GlobalSamplingIterations"); 
    
    // // Construct the intial global solution
    // for(Robot* const robot : *(arcQuery->GetRobotGroup())) {
    //   globalSolution[robot] = std::shared_ptr<Path>(this->GetPath(robot));
    // }
       
  }

  m_currentConflict = arcQuery->GetCurrentConflict();
  m_lastConflict = arcQuery->GetLastConflict();



  // We will repair if there is a conflict 
  if(!m_currentConflict.Empty()) {

    ++m_repairCounter;
    if(this->m_debug) {
      std::cout << "\tStarting Repair iteration " << m_repairCounter << ","
                << " Sampling iteration " << m_samplingCounter << "."
                << std::endl;
    }

    m_repairFailed = !Repair(m_currentConflict);
    if(m_repairFailed) arcQuery->SetRepairFailed();



  }






    // // Set the task we're working on.
    // m_groupTask = this->GetGroupTask();
    // m_group = m_groupTask->GetRobotGroup();
    // m_groupRoadmap = this->GetMPLibrary()->GetGroupRoadmap(m_group);

    // if(this->m_debug) {
    //   std::cout << "Running Local Repair query for robot group '"
    //             << m_groupTask->GetRobotGroup()->GetLabel() << "' ("
    //             << m_groupTask->GetRobotGroup() << "), "
    //             << "task '" << m_groupTask->GetLabel() << "' ("
    //             << m_groupTask << ")."
    //             << std::endl;
    // }

    // auto arcStrategy = dynamic_cast<ARCStrategy<MPTraits>*>(this->GetMPStrategy(m_arcStrategyLabel));
    // arcStrategy->Initialize();
    // Solution solution = arcStrategy->GetSolution();

    // this->GetMPLibrary()->SetTask(nullptr);
    // this->GetMPLibrary()->SetGroupTask(m_groupTask);
    // if(solution.empty()) {
    //   return false;
    // }
    // // Check the solution for the first conflict.
    // Conflict conflict;
    // conflict = FindConflict(solution, 0);

    // // Collect initial paths.
    // auto globalPaths = CollectPaths(solution);

    // bool success = true;



    // // We'll keep iterating until we solve all the conflicts.
    // while(!conflict.Empty()) {

    //   // Increment iteration count.
      
      


    //   // Now run the repair and see if it can compute the conflict-free local paths
    //   arcStrategy->CurrentRepairIteration(repairCounter);
    //   bool successRepair = false;
    //   {
    //   // add timer here
    //   MethodTimer mt(this->GetStatClass(),
    //       this->GetNameAndLabel() + "::Time spent on repair");
    //   successRepair = arcStrategy->Repair(conflict.cfg1, conflict.cfg2, conflict.timestep, globalPaths);
    //   }
    //   if(!successRepair) {
    //     if(this->m_debug) {
    //       std::cout << "\033[0;1;31m" "Repair failed!!!!!!" "\033[0m" << std::endl;
    //     }
    //     success = false;
    //     break;
    //   }


    //   // latest timestep to search from when looking for another conflict
    //   size_t latestTimestep = 0;
    //   if(conflict.timestep > (arcStrategy->GetAdjustedTimeWindow() +
    //         arcStrategy->GetMexLocalPathLength()))
    //     latestTimestep = conflict.timestep - (arcStrategy->GetAdjustedTimeWindow() +
    //         arcStrategy->GetMexLocalPathLength());


    //   // Check if there is another conflict. If the new conflict we found is the same
    //   // as the last one we declare repair as failed.

    //   conflict = FindConflict(solution, latestTimestep);

    //   // check if we have already seen and solved this conflict
    //   for(auto c : m_conflictCache) {
    //     if(c == conflict) {
    //       if(this->m_debug) {
    //         std::cout << "\033[0;1;31m" "Repair failed!" "\033[0m" << std::endl;
    //       }
    //       success = false;
    //       break;
    //     }
    //   }

    //   if(!success)m_group
    //     break;

    //   m_conflictCache.insert(conflict);
    // }

    // m_conflictCache.clear();

    // // If there is no conflict, we solved the problem successfully.
    // if(success) {
    //   double currentCost = ComputeCost(solution);
    //   SetSolution(std::move(solution));
    //   if(this->m_debug) {
    //     std::cout << "\tSolution found with cost " << currentCost << "." << std::endl;
    //     for(Robot* const robot : *m_groupTask->GetRobotGroup())
    //       std::cout << "\t\tVID Path for robot " << robot->GetLabel()
    //                 << ": " << this->GetPath(robot)->VIDs() << std::endl;
    //   }
    // } else {
    //     if(this->m_debug) {
    //       std::cout << "\tNo solution found." << std::endl;
    //     }
    // }

    // // Restore the group task.
    // if(this->m_debug) {
    //   std::cout << "Setting back group task with " << m_groupRoadmap->GetNumRobots() << "robot on roadmap" << std::endl;
    //   std::cout << "Setting back group task with " << m_group->Size() << "robot on robot group" << std::endl;
    // }
    // this->GetMPLibrary()->SetGroupTask(m_groupTask);

    // // m_conflictCache.clear();

    // if(success)
    //   this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::FoundPath", 1);
    // return success;
    // }
}


/*------------------------------- Accesors -----------------------------*/
template<typename MPTraits>
void
ARCStrategy<MPTraits>::
CurrentRepairIteration(size_t _current){
  m_currentRepairIteration = _current;
}

template<typename MPTraits>
size_t
ARCStrategy<MPTraits>::
GetAdjustedTimeWindow() {
  return m_adjustedWindowSize;
}


template<typename MPTraits>
size_t
ARCStrategy<MPTraits>::
GetMexLocalPathLength() {
  return m_maxLocalPathLength;
}

template<typename MPTraits>
std::set<Robot*>
ARCStrategy<MPTraits>::
GetCurrentConflictingRobots() {
  std::set<Robot*> robots;
  for(auto cfg : m_currentConflict.cfgs)
    robots.insert(cfg.GetRobot());
  return robots;
}

// template<typename MPTraits>
// typename ARCStrategy<MPTraits>::Solution
// ARCStrategy<MPTraits>::
// GetSolution() {
//   return m_solution;
// }



/*--------------------------- Repair -------------------------*/

template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
Repair(Conflict& _conflict) {

  // we want to make sure each conflict start with the initial time window
  // otherwise if it's the same as the last conflict we keep the current time window
  if(!m_lastConflict.cfgs.empty()) {
    if(_conflict == m_lastConflict) {
      m_windowSize = m_windowSize;
    } else {
      m_windowSize = m_initialWindowSize;
    }
  }
  // std::vector<CfgType> newVec{_cfg1, _cfg2};

  // // storing the conflict
  // m_lastConflict.cfgs = newVec;
  // m_lastConflict.timestep = _timestep;

  // repair
  // Conflict conflict{{_cfg1,_cfg2},_timestep};


  if(m_ifExpandRobot) {
    MergeConflict(_conflict);
  }

  auto strategy = m_strategyLabel;
  auto localProblem = LocalProblemInitialization(_conflict);
  // Set global to false to use the hierarchy at least once
  m_ifGlobal = false;
  auto success = SingleResolution(localProblem, strategy);

  /// HERE WE UPDATE THE CONFLICT MAP
  if(success && m_ifExpandRobot)
    success = UpdateConflictMap(_conflict);
  //bool isLocalSolutionConflictFree = false;
  //isLocalSolutionConflictFree = PostValidation(conflict);


  while(!success) {// or !isLocalSolutionConflictFree) {

    //if(m_ifExpandRobot) {
    //  isLocalSolutionConflictFree = PostValidation(conflict);
    //}

    //if(!isLocalSolutionConflictFree)
    //  continue;

    // NOTE: currently the probability function for deciding expansions
    // is not included
    UpdateLocalProblem("queries");
    if(m_ifExpandRobot) {
      MergeConflict(_conflict);
    }

    // Initialize the local problem and attempt to solve it
    localProblem = LocalProblemInitialization(_conflict);
    success = SingleResolution(localProblem, strategy);
    /// HERE WE UPDATE THE CONFLICT MAP
    if(success && m_ifExpandRobot)
      success = UpdateConflictMap(_conflict);


    if(this->OverpassingWindow(localProblem))
      break;

  }

  if(success) {
    m_currentConflict = _conflict;
    this->GetStatClass()->SetStat("final local region radius", m_localRadius);
    this->GetStatClass()->SetStat("final local region window size", m_adjustedWindowSize);
    this->ReassemblePaths(localProblem, m_localSolution);
    // if(m_printLocal) {
    //   for(auto pair : _solution) {
    //     PrintLocalPath(pair.first);
    //     // PrintLocalRoadmap(pair.first);
    //   }
    // }
  }

  m_boundaries.clear();
  m_boundingSpheres.clear();

  return success;

}




/*--------------------------- Initial Global Solution Construction -----------------------*/
template <typename MPTraits>
void
ARCStrategy<MPTraits>::
ConstructInitialSolution() {

  // Assert that the query evaluator is an instance of query method.
  auto query = dynamic_cast<QueryMethod<MPTraits>*>(
      this->GetMapEvaluator(m_queryLabel));
  if(!query)
    throw RunTimeException(WHERE) << "Query method " << m_queryLabel
                                  << " is not of type QueryMethod."
                                  << std::endl;

  // Set the query method's path weight function.
  query->SetPathWeightFunction(
      [this](typename RoadmapType::adj_edge_iterator& _ei,
             const double _sourceDistance,
             const double _targetDistance) {
        return this->MultiRobotPathWeightRepair(_ei, _sourceDistance, _targetDistance);
      }
  );


  auto groupTask = this->GetGroupTask();

  // Clear the group task for creating the individual plans.
  this->GetMPLibrary()->SetGroupTask(nullptr);

  // Construct an initial solution with all robots ignoring each other.
  // Solution initialSolution;
  for(auto& task : *groupTask) {
    // Ensure a maximum of one task per robot.
    auto robot = task.GetRobot();

    // Construct a path for this robot. If none exists, we can't solve the query
    // yet.
    SolveIndividualTask(robot, &task);
    // Solution emptySolution;
    
    // if(!this->GetPath(robot)) return false;
    // initialSolution[robot] = path;
  }

  // Set group task back
  this->GetMPLibrary()->SetGroupTask(groupTask);

  // m_solution = initialSolution;
  // return true;
}


template <typename MPTraits>
void
ARCStrategy<MPTraits>::
SolveIndividualTask(Robot* const _robot, MPTask* const _task) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SolveIndividualTask");

  // MPTask* const task = m_taskMap.at(_robot);
  if(this->m_debug) {
    std::cout << "Solving task '" << _task->GetLabel() << "' (" << _task
              << ") for robot '" << _robot->GetLabel() << "' (" << _robot
              << ")."
              << std::endl;
  }

  // Generate a path for this robot individually while avoiding the conflicts.

  this->GetMPLibrary()->SetTask(_task);
  // auto query = this->GetMapEvaluator(m_queryLabel);
  auto query = dynamic_cast<QueryMethod<MPTraits>*>(
      this->GetMapEvaluator(m_queryLabel)
  );
  query->SetPathWeightFunction(
      [this](typename RoadmapType::adj_edge_iterator& _ei,
             const double _sourceDistance,
             const double _targetDistance) {
        return this->MultiRobotPathWeightRepair(_ei, _sourceDistance, _targetDistance);
      }
  );
  const bool success = (*query)();
  this->GetMPLibrary()->SetTask(nullptr);

  if(this->m_debug) {
    std::cout << "Path for robot " << _robot->GetLabel() << " was "
              << (success ? "" : "not ") << "found."
              << std::endl;
  }

  // Return immediately if we fail to find a path.
  if(!success) return;

  // // Otherwise, return a copy of the robot's path from the solution object.
  // Path* path = this->GetPath(_robot);
  // auto out = std::shared_ptr<Path>(new Path(std::move(*path)));

  const std::string base = this->GetBaseFilename();
  ::WritePath(base +"."+ _robot->GetLabel()  + ".localRepair.initialGlobal.path",
        this->GetPath(_robot)->FullCfgs(this->GetMPLibrary()));
  // this->GetPath(_robot)->Clear();

}


template <typename MPTraits>
double
ARCStrategy<MPTraits>::
MultiRobotPathWeightRepair(typename RoadmapType::adj_edge_iterator& _ei,
    const double _startTime, const double _bestEndTime) const {

  // Compute the time when we will end this edge.
  const size_t startTime = static_cast<size_t>(std::llround(_startTime)),
               endTime   = startTime + _ei->property().GetTimeSteps();
  return endTime;
}






/*---------------------- Local Problem Preprocessing ---------------------------*/
template<typename MPTraits>
typename ARCStrategy<MPTraits>::QueriesCfg
ARCStrategy<MPTraits>::
PreProcessing(Conflict _c,size_t  _window) {
//PreProcessing(Conflict _c,size_t  _window) {
  // First, we compute valid local queries
  auto queryInfo = ComputeQueriesCfg(_c, _window);
  auto queries = queryInfo.first;
  auto timesteps = queryInfo.second;

  // Then we add them to their corresponding roadmaps
  auto existingQueriesVID = AddingQueriesCfg(queries,timesteps);
  // Then we add safe edges for storing the conflict-free path segments
  //auto safePairsVID = AddingSafeEdges(queries, existingQueriesVID, _c.timestep);
  auto safePairsVID = existingQueriesVID;
  // We finally compute the conflict-free path segments
  m_safeSubPaths = ComputeSubPathPairs(queries, timesteps, safePairsVID);

  m_conflictTimestep = _c.timestep;

  // Check if we've reached the global problem
  m_ifGlobal = true;
  for(size_t i = 0; i < queries.size(); i++) {

    auto ts = timesteps[i];
    if(ts.first != 0) {
      m_ifGlobal = false;
      break;
    }

    auto q = queries[i];
    auto robot = q.first.GetRobot();
    auto path = this->GetPath(robot);
    auto last = path->TimeSteps();

    if(ts.second < last) {
      m_ifGlobal = false;
      break;
    }
  }

  return queries;
}


template<typename MPTraits>
void
ARCStrategy<MPTraits>::
ReassemblePaths(typename ARCStrategy<MPTraits>::QueriesCfg& _queries, std::map<Robot*,std::shared_ptr<Path>> _localPaths) {
  std::vector<Path> reassembledPaths;
  for(size_t i = 0 ; i < _queries.size() ; ++i) {
    auto robot = _queries[i].first.GetRobot();
    auto localPath = _localPaths[robot].get();
    m_maxLocalPathLength = std::max(m_maxLocalPathLength, localPath->TimeSteps());
    ReassemblePath(robot, localPath, m_safeSubPaths[i]);
  }
}


template<typename MPTraits>
void
ARCStrategy<MPTraits>::
ReassemblePath(Robot* _robot, Path* _localPath, std::pair<Path,Path> _safeSubPaths) {
  Path out(this->GetRoadmap(_robot));

  // if vids are not empty this means we are running decoupled
  // and we want to have wait times
  //if(!_localPath->VIDs().empty()) {
  AddZeroWaitTimes(_safeSubPaths.first);
  AddZeroWaitTimes(_localPath);
  AddZeroWaitTimes(_safeSubPaths.second);
  //}

  auto localStart = _localPath->StartVID();
  auto globalGoal = this->GetPath(_robot)->GoalVID();
  auto robotAtGoal = localStart == globalGoal;

  auto zeroCost = _localPath->TimeSteps() == 0;

  //auto pathTimesteps =
  //  _safeSubPaths.first.FullCfgs(this->GetMPLibrary()).size() +
  //  _localPath->FullCfgs(this->GetMPLibrary()).size() +
  //  _safeSubPaths.second.FullCfgs(this->GetMPLibrary()).size();

  auto pathTimesteps = _safeSubPaths.first.TimeSteps() +
                       _localPath->TimeSteps() +
                       _safeSubPaths.second.TimeSteps();

  auto lateConflict = m_conflictTimestep > pathTimesteps;

  out += _safeSubPaths.first;
  if(robotAtGoal and zeroCost and lateConflict) {
    //std::vector<size_t> goalVID{globalGoal},
    //                    waitingTimesteps{m_conflictTimestep-pathTimesteps};
    //out += std::make_pair(goalVID,waitingTimesteps);
    auto waiting = out.GetWaitTimes();
    waiting[waiting.size()-1] = m_conflictTimestep-pathTimesteps;
    out.SetWaitTimes(waiting);
  }
  out += *_localPath;
  out += _safeSubPaths.second;
  *(this->GetPath(_robot)) = out;


}

template<typename MPTraits>
std::pair<typename ARCStrategy<MPTraits>::QueriesCfg,std::vector<std::pair<size_t,size_t>>>
ARCStrategy<MPTraits>::
ComputeQueriesCfg(Conflict _c, size_t _window) {

  std::vector<std::pair<size_t,size_t>> windowTimesteps;

  // Assigning info from conflict
  auto cfgs = _c.cfgs;
  auto timestep = _c.timestep;
  // Computing initial local queries
  m_adjustedWindowSize = _window;
  QueriesCfg queries;
  for(auto& cfg : cfgs) {
   auto min = timestep > _window ? timestep - _window : 0;
   auto max = this->GetPath(cfg.GetRobot())->TimeSteps();
   max = std::min(max,timestep + _window);
   auto query = ComputeQueryCfg(cfg, timestep, min, max);
   queries.push_back(query);
   windowTimesteps.push_back(std::make_pair(min,max));
  }
  // Validating queries, if they are still in collision we extend the window
  auto increment = m_increment;
  while(!ValidQueries(queries)) {
    queries.clear();
    windowTimesteps.clear();

    for(auto& cfg : cfgs) {
      // This is hack to compute windows of different sizes
      auto randomWindow = _window + increment;
      if(DRand()> 0.5)
        randomWindow = _window;

      // These are poorly named and implemented - fix
      auto preBuffer = _window;
      auto postBuffer = randomWindow;

      auto min = timestep < preBuffer ? 0 : timestep - preBuffer;
      auto max = this->GetPath(cfg.GetRobot())->TimeSteps();
      max = std::min(max,timestep + postBuffer);
      auto query = ComputeQueryCfg(cfg, timestep, min, max);
      queries.push_back(query);
      windowTimesteps.push_back(std::make_pair(min,max));
    }
    m_adjustedWindowSize = _window + increment;
    if(this->m_debug) {
      std::cout << "Adjusted window size : " << m_adjustedWindowSize  << "." << std::endl;
    }
    increment += m_increment;
  }
  if(this->m_debug) {
    std::cout << "Adjusted window size : " << m_adjustedWindowSize  << "." << std::endl;
  }
  if(m_windowExpansions == 0)
    this->GetStatClass()->SetStat("initial local region window size", m_adjustedWindowSize);
  m_windowExpansions++;

  return std::make_pair(queries,windowTimesteps);
}


template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
OverpassingWindow(typename ARCStrategy<MPTraits>::QueriesCfg _queries) {

  auto robot1 = _queries[0].first.GetRobot();
  auto roadmap1 = this->GetRoadmap(robot1);
  auto startVID1 = roadmap1->GetVID(_queries[0].first);
  auto goalVID1 = roadmap1->GetVID(_queries[0].second);
  auto path1 = this->GetPath(robot1)->VIDs();

  auto robot2 = _queries[1].first.GetRobot();
  auto roadmap2 = this->GetRoadmap(robot2);
  auto startVID2 = roadmap2->GetVID(_queries[1].first);
  auto goalVID2 = roadmap2->GetVID(_queries[1].second);
  auto path2 = this->GetPath(robot2)->VIDs();

  if(path1.empty() or path2.empty())
    return false;

  auto timesteps1 = this->GetPath(robot1)->TimeSteps();
  auto timesteps2 = this->GetPath(robot1)->TimeSteps();

  auto maxTimesteps = std::max(timesteps1,timesteps2);

  if(m_adjustedWindowSize > maxTimesteps/2)
    return false;


  return startVID1 == path1.front() and goalVID1 == path1.back()
     and startVID2 == path2.front() and goalVID2 == path2.back();
}

template<typename MPTraits>
typename ARCStrategy<MPTraits>::QueryCfg
ARCStrategy<MPTraits>::
ComputeQueryCfg(CfgType& _cfg, size_t _timestep, size_t _min, size_t _max) {

  auto robot = _cfg.GetRobot();
  auto path = this->GetPath(robot);
  auto cfgPath = path->FullCfgs(this->GetMPLibrary());
  //const std::string base = this->GetBaseFilename();
  //::WritePath(base +"."+ robot->GetLabel()  + "."
  //         + std::to_string(this->m_currentCBSIteration) + ".pre.global.path"
  //        , path->FullCfgs(this->GetMPLibrary()));

  CfgType beforeConflict(robot),
          afterConflict(robot);
  auto timestepBefore = std::min(_min, cfgPath.size()-1);
  //if(_timestep < _preBuffer)
  //  timestepBefore = 0;
  auto timestepAfter  = std::min(_max, cfgPath.size()-1);

  if(this->m_debug) {
    std::cout << "Robot : " << robot->GetLabel() << " with timestepBefore " << timestepBefore
        << " and timestepAfter " << timestepAfter << "." << std::endl;
  }

  beforeConflict = cfgPath[timestepBefore];
  afterConflict = cfgPath[timestepAfter];
  return std::make_pair(beforeConflict,afterConflict);
}


template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
ValidQueries(typename ARCStrategy<MPTraits>::QueriesCfg& _queries) {
  bool valid;
  // We first check start positions
  /*
  for(size_t i = 0 ; i < _queries.size()-1 ; ++i) {
    for(size_t j = i ; j < _queries.size() ; ++j) {
      if(i==j)
        continue;
      valid = ValidEndpoints(_queries[i].first, _queries[j].first);
      if(!valid)
        return false;
    }
  }*/
  //Commenting this, given shared-roadmap triggers invalid subgoals pretty often.

  // We then check goal positions
  for(size_t i = 0 ; i < _queries.size()-1 ; ++i) {
    for(size_t j = i ; j < _queries.size() ; ++j) {
      if(i==j)
        continue;
      valid = ValidEndpoints(_queries[i].second, _queries[j].second);
      if(!valid)
        return false;
    }
  }

  if(!ValidConstraints(_queries))
    return false;

  return true;
}


template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
ValidEndpoints(CfgType& _cfg1, CfgType& _cfg2) {

  auto vc = static_cast<CollisionDetectionValidity<MPTraits>*>(
      this->GetValidityChecker("pqp_solid"));
  CDInfo cdInfo;

  // Configure the first robot at the approriate configuration.
  // we first need to check if the cfgs are in free space
  const std::string callee = this->GetNameAndLabel();
  if(!vc->IsValid(_cfg1, callee) || !vc->IsValid(_cfg2, callee))
    return false;

  auto robot1        = _cfg1.GetRobot();
  auto multibody1    = robot1->GetMultiBody();
  multibody1->Configure(_cfg1);


  // Configure the second robot at the appropriate configuration.
  auto robot2        = _cfg2.GetRobot();
  auto multibody2    = robot2->GetMultiBody();
  multibody2->Configure(_cfg2);


  // Check for collision. If none, move on.
  const bool collision = vc->IsMultiBodyCollision(cdInfo,
      multibody1, multibody2, this->GetNameAndLabel());

  return !collision;
}

template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
ValidConstraints(typename ARCStrategy<MPTraits>::QueriesCfg& _queries) {
  // validate each cfg as a start/goal constraint
  for(size_t i = 0 ; i < _queries.size() ; ++i) {
    auto start = _queries[i].first;
    auto goal  = _queries[i].second;

    if(!ValidConstraint(start) || !ValidConstraint(goal))
      return false;
  }
  return true;
}

template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
ValidConstraint(CfgType& _cfg) {
    auto roadmap = this->GetRoadmap(_cfg.GetRobot());
    auto size1 = roadmap->get_num_vertices();
    auto vid = roadmap->AddVertex(_cfg);
    auto size2 = roadmap->get_num_vertices();

    auto boundary = new CSpaceBoundingBox(_cfg.DOF());
    for(size_t i = 0; i < boundary->GetDimension(); i++) {
      boundary->SetRange(i, _cfg[i], _cfg[i]);
    }
    if(size1 != size2) {
      if(!boundary->InBoundary(roadmap->GetVertex(vid))) {
        roadmap->DeleteVertex(vid);
        return false;
      } else {
        roadmap->DeleteVertex(vid);
      }
    } else {
      if(!boundary->InBoundary(roadmap->GetVertex(vid))) {
        return false;
      }
    }
  return true;
}


template<typename MPTraits>
std::vector<std::pair<size_t,size_t>>
ARCStrategy<MPTraits>::
AddingQueriesCfg(typename ARCStrategy<MPTraits>::QueriesCfg _queries,std::vector<std::pair<size_t,size_t>> _windowTimesteps) {
  std::vector<std::pair<size_t,size_t>> existingQueriesVID;
  for(size_t i = 0; i < _queries.size(); i++) {
    auto existingQueryVID = AddingQueryCfg(_queries[i],_windowTimesteps[i]);
    existingQueriesVID.push_back(existingQueryVID);
  }
  return existingQueriesVID;
}


template<typename MPTraits>
std::pair<size_t,size_t>
ARCStrategy<MPTraits>::
AddingQueryCfg(typename ARCStrategy<MPTraits>::QueryCfg _query, std::pair<size_t,size_t> _windowTimesteps) {

  auto robot = _query.first.GetRobot();
  auto roadmap = this->GetRoadmap(robot);
  auto path = this->GetPath(robot);

  auto oldVIDs = path->VIDs();
  auto oldWaittimes = path->GetWaitTimes();
  auto vid1 = roadmap->AddVertex(_query.first);
  auto vid2 = roadmap->AddVertex(_query.second);
  // Get original edge at timestep
  auto edge = path->GetEdgeAtTimestep(_windowTimesteps.first);
  auto vids = edge.first;
  // Get original edge at timestep
  auto edge2 = path->GetEdgeAtTimestep(_windowTimesteps.second);
  auto vids2 = edge2.first;

  if(edge == edge2) {
    AddNewEdge(roadmap,vid1,vid2);
  }


  auto durations = path->GetDurations();

  size_t cnt = 0;

  // Collect set of new vids and wait times
  std::vector<size_t> newVIDs;
  std::vector<size_t> newWaittimes;


  // Update path to first edge vid
  size_t i = 0;
  for(; i < oldVIDs.size(); i++) {

    newVIDs.push_back(oldVIDs[i]);

    if(i > 0) {
      auto edge = roadmap->GetEdge(newVIDs[newVIDs.size()-2],newVIDs.back());
      if(durations.empty())
        cnt += edge.GetTimeSteps();
      else if (durations[i] == std::numeric_limits<size_t>::infinity())
        cnt += edge.GetTimeSteps();
      else
        cnt += durations[i];
    }

    if(newVIDs.size() > 2 and !roadmap->IsEdge(newVIDs[newVIDs.size()-2],newVIDs[newVIDs.size()-1]))
      throw RunTimeException(WHERE) << "Adding nonexistent edge to vids sequence.";

    size_t waittime = oldWaittimes.empty() ? 0 : oldWaittimes[i];
    newWaittimes.push_back(waittime);

    cnt += waittime;

    // Check if start of edge matches this VID
    if(oldVIDs[i] == vids.first and cnt >= edge.second.first and cnt <= edge.second.second) {
      break;
    }
  }
  i++;

  if(cnt > _windowTimesteps.first) {
    throw RunTimeException(WHERE) << "Went too far.";
  }

  // Check if the new start vid1 lays along the edge or at a vertex
  if(vids.first != vid1 and vids.second != vid1) {
    // Add edge to roadmap
    AddNewEdge(roadmap,vids.first,vid1);
    AddNewEdge(roadmap,vid1,vids.second);

    newVIDs.push_back(vid1);

    if(newVIDs.size() > 2 and !roadmap->IsEdge(newVIDs[newVIDs.size()-2],newVIDs[newVIDs.size()-1]))
      throw RunTimeException(WHERE) << "Adding nonexistent edge to vids sequence.";

    newWaittimes.push_back(0);

  }


  // Check if vid2 lays along the same edge
  if(oldVIDs[i] != vids2.second) {
    // If not, fill in the path to vid2
    for(; i < oldVIDs.size(); i++) {

      newVIDs.push_back(oldVIDs[i]);

      if(newVIDs.size() > 2 and !roadmap->IsEdge(newVIDs[newVIDs.size()-2],newVIDs[newVIDs.size()-1]))
        throw RunTimeException(WHERE) << "Adding nonexistent edge to vids sequence.";

      size_t waittime = oldWaittimes.empty() ? 0 : oldWaittimes[i];
      newWaittimes.push_back(waittime);

      if(oldVIDs[i] == vids2.first)
        break;
    }
    i++;
  }

  // Connect vid2 to the path
  if(vids2.first != vid2 and vids2.second != vid2) {
    AddNewEdge(roadmap,vids2.first,vid2);
    AddNewEdge(roadmap,vid2,vids2.second);

    newVIDs.push_back(vid2);

    if(newVIDs.size() > 2 and !roadmap->IsEdge(newVIDs[newVIDs.size()-2],newVIDs[newVIDs.size()-1]))
      throw RunTimeException(WHERE) << "Adding nonexistent edge to vids sequence.";

    newWaittimes.push_back(0);

    newVIDs.push_back(vids2.second);

    if(newVIDs.size() > 2 and !roadmap->IsEdge(newVIDs[newVIDs.size()-2],newVIDs[newVIDs.size()-1]))
      throw RunTimeException(WHERE) << "Adding nonexistent edge to vids sequence.";

    size_t waittime = oldWaittimes.empty() ? 0 : oldWaittimes[i];
    newWaittimes.push_back(waittime);
    i++;
  }

  // Fill in the path to the end
  for(; i < oldVIDs.size(); i++) {
    newVIDs.push_back(oldVIDs[i]);

    if(newVIDs.size() > 2 and !roadmap->IsEdge(newVIDs[newVIDs.size()-2],newVIDs[newVIDs.size()-1]))
      throw RunTimeException(WHERE) << "Adding nonexistent edge to vids sequence.";

    size_t waittime = oldWaittimes.empty() ? 0 : oldWaittimes[i];
    newWaittimes.push_back(waittime);
  }

  auto oldTimesteps = path->TimeSteps();

  // Reset path
  path->Clear();
  *path += newVIDs;
  path->SetWaitTimes(newWaittimes);

  if(path->TimeSteps() != oldTimesteps) {
    throw RunTimeException(WHERE) << "Throw off path length.";
  }


  return std::make_pair(vid1,vid2);
}


template<typename MPTraits>
void
ARCStrategy<MPTraits>::
AddNewEdge(RoadmapType* _rm, size_t _source, size_t _target) {
  auto dm = this->GetMPLibrary()->GetDistanceMetric("euclidean");
  auto distance = dm->Distance(_rm->GetVertex(_source),_rm->GetVertex(_target));
  int numSteps;
  CfgType increment(_rm->GetRobot());
  increment.FindIncrement(_rm->GetVertex(_source), _rm->GetVertex(_target),
              &numSteps, this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetPositionRes());

  typename MPTraits::WeightType weight;
  weight.SetWeight(distance);
  weight.SetTimeSteps(size_t(numSteps));

  _rm->AddEdge(_source,_target,weight);
}


template<typename MPTraits>
typename ARCStrategy<MPTraits>::PathPairVector
ARCStrategy<MPTraits>::
ComputeSubPathPairs(typename ARCStrategy<MPTraits>::QueriesCfg _queries,
    std::vector<std::pair<size_t,size_t>> _windowTimesteps,
    typename ARCStrategy<MPTraits>::PairVIDVector _pairsVID ) {
  PathPairVector subPathsVector;
  for(size_t i = 0 ; i < _queries.size() ; ++i) {
    auto subPathPair = ComputeSubPathPair(_queries[i], _windowTimesteps[i], _pairsVID[i]);
    subPathsVector.push_back(subPathPair);
  }
  return subPathsVector;
}


template<typename MPTraits>
typename ARCStrategy<MPTraits>::PathPair
ARCStrategy<MPTraits>::
ComputeSubPathPair(typename ARCStrategy<MPTraits>::QueryCfg _query, std::pair<size_t,size_t> _windowTimesteps,
	typename ARCStrategy<MPTraits>::PairVID _pairVID) {

  auto robot = _query.first.GetRobot();
  auto path = this->GetPath(robot);

  AddZeroWaitTimes(*path);

  std::vector<size_t> beforeWaiting;
  std::vector<size_t> afterWaiting;

  Path outBefore(this->GetRoadmap(robot));
  Path outAfter(this->GetRoadmap(robot));

  auto vids = path->VIDs();
  auto waitingTimesteps = path->GetWaitTimes();

  if(!vids.empty()) {
    std::vector<VID> beforePath;
    std::vector<VID> afterPath;
    size_t i = 0;
    for(; i < vids.size(); i++) {
      auto vid = vids[i];
      auto waiting = waitingTimesteps[i];

      beforePath.push_back(vid);
      beforeWaiting.push_back(waiting);

      if(vid == _pairVID.first) {
        break;
      }
    }

    for(; i < vids.size(); i++) {
      auto vid = vids[i];
      if(vid == _pairVID.second) {
        break;
      }
    }

    for(; i < vids.size(); i++) {
      auto vid = vids[i];
      auto waiting = waitingTimesteps[i];

      afterPath.push_back(vid);
      afterWaiting.push_back(waiting);
    }

    outBefore += beforePath;
    outAfter += afterPath;

    if(this->m_debug)
      std::cout << robot->GetLabel() << " SubPaths:\t beforePath: "
        << beforePath << "\tafterPath: " << afterPath << std::endl;
  }

  if(outBefore.TimeSteps() > _windowTimesteps.first) {
    auto diff = outBefore.TimeSteps() - _windowTimesteps.first;
    auto last = beforeWaiting.back();
    if(diff > last)
      throw RunTimeException(WHERE) << "Path timesteps have been thrown off.";

    beforeWaiting[beforeWaiting.size()-1] = last - diff;
    outBefore.SetWaitTimes(beforeWaiting);
  }

  outBefore.SetWaitTimes(beforeWaiting);
  outAfter.SetWaitTimes(afterWaiting);

  // TODO::Figure out equivalent for the out after path

  return std::make_pair(outBefore,outAfter);
}

template<typename MPTraits>
void
ARCStrategy<MPTraits>::
AddZeroWaitTimes(Path* _path) {
  if(_path->GetWaitTimes().size() == 0) {
    auto numVIDs = _path->VIDs().size();
    if(numVIDs == 0)
      numVIDs = _path->VIDs().size();

    std::vector<size_t> waitTimes(numVIDs, 0);
    _path->SetWaitTimes(waitTimes);
  }
}

template<typename MPTraits>
void
ARCStrategy<MPTraits>::
AddZeroWaitTimes(Path& _path) {
  if(_path.GetWaitTimes().size() == 0) {
    auto numVIDs = _path.VIDs().size();
    if(numVIDs == 0)
      numVIDs = _path.VIDs().size();

    std::vector<size_t> waitTimes(numVIDs, 0);
    _path.SetWaitTimes(waitTimes);
  }
}





/*---------------------- Local Problem Construction ------------------------*/


template<typename MPTraits>
typename ARCStrategy<MPTraits>::QueriesCfg
ARCStrategy<MPTraits>::
LocalProblemInitialization(Conflict _c) {
  auto queries = this->PreProcessing(_c, m_windowSize);
  return queries;
}

template<typename MPTraits>
std::vector<MPTask>
ARCStrategy<MPTraits>::
CreateLocalTaskSet(typename ARCStrategy<MPTraits>::QueriesCfg _queries) {

  std::vector<MPTask> localTaskSet;
  for(auto query : _queries){
    auto robot = query.first.GetRobot();
    MPTask localTask(robot);
    auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,query.first));
    auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,query.second));
    localTask.SetStartConstraint(std::move(startConstraint));
    localTask.AddGoalConstraint(std::move(goalConstraint));
    localTaskSet.push_back(localTask);
  }
  return localTaskSet;
}


/*--------------------------- Local Problem Update -----------------------*/

template<typename MPTraits>
void
ARCStrategy<MPTraits>::
UpdateLocalProblem(std::string _expander) {

  if(_expander == "queries") {
    m_windowSize += m_queriesIncrement;
    std::cout << "\033[0;1;34m" "Expanding time space" "\033[0m" << std::endl;
    this->GetStatClass()->IncStat("time space expansions");
  }
  else if (_expander == "boundaries") {
    m_radius += m_boundariesIncrement;
    std::cout << "\033[0;1;34m" "Expanding c-space" "\033[0m" << std::endl;
    this->GetStatClass()->IncStat("c-space expansions");
  }
}

template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
UpdateConflictMap(Conflict& _conflict) {

  // TODO::Check if local solution extended past window and now intersects
  // following interval and requires further resolutions
  std::set<Robot*> robots;
  for(auto& solution : m_localSolution) {
    robots.insert(solution.first);
  }

  //for(auto& solution : m_localSolution) {
  for(auto robot : robots) {
    auto solution = m_localSolution[robot];
    auto lowerBound = _conflict.timestep - m_windowSize;
    if(_conflict.timestep < m_windowSize)
      lowerBound = 0;
    auto higherBound = _conflict.timestep - m_windowSize +
      solution->TimeSteps();
    Range<size_t> interval(lowerBound,higherBound);
    element newElem(robots,interval);

    //for(auto robot : robots) {
    {
      // Check if element overrides any existing elements
      auto pq = m_conflictIntervalMap[robot];
      m_conflictIntervalMap[robot] = std::priority_queue<element,std::vector<element>,std::greater<element>>();
      while(!pq.empty()) {
        auto elem = pq.top();
        pq.pop();

        bool intersect = false;

        if(newElem.interval.min <= elem.interval.min and newElem.interval.max >= elem.interval.max) {
          intersect = true;
        }
        else if(newElem.interval.min >= elem.interval.min and
            newElem.interval.max <= elem.interval.max) {
          intersect = true;
          auto copy = elem;
          copy.interval.max = newElem.interval.min;
          elem.interval.min = newElem.interval.max;
          m_conflictIntervalMap[robot].push(copy);
          m_conflictIntervalMap[robot].push(elem);
        }
        else if(elem.interval.Contains(newElem.interval.min)) {
          intersect = true;
          elem.interval.max = newElem.interval.min;
          m_conflictIntervalMap[robot].push(elem);
        }
        else if(elem.interval.Contains(newElem.interval.max)) {
          intersect = true;
          elem.interval.min = newElem.interval.max;
          m_conflictIntervalMap[robot].push(elem);
        }
        else {
          m_conflictIntervalMap[robot].push(elem);
        }

        // If there is an intersection make sure all robots are already involved
        if(intersect) {
          for(auto r : elem.robots) {
            if(newElem.robots.count(r))
              continue;

            //throw RunTimeException(WHERE) << "Overlapping interval with robots"
            //  << " not contained in the conflict resolution.";
            auto path = this->GetPath(r);
            auto cfgs = path->FullCfgs(this->GetMPLibrary());
            auto ts = std::min(_conflict.timestep,cfgs.size()-1);
            auto c = cfgs[ts];
            _conflict.cfgs.push_back(c);
            return false;
          }
        }
      }

      // Add new element
      m_conflictIntervalMap[robot].push(newElem);
    }
  }

  return true;
}



template<typename MPTraits>
void
ARCStrategy<MPTraits>::
MergeConflict(Conflict& _conflict) {

  // Check if m_conflictIntervalMap has been initialized
  // If not, initialize
  if(m_conflictIntervalMap.empty()) {
    for(Robot* const& robot : *this->GetGroupTask()->GetRobotGroup()) {
      std::set<Robot*> robots = {robot};
      Range<size_t> interval(0,SIZE_MAX);
      element elem(robots,interval);

      m_conflictIntervalMap[robot] =
        std::priority_queue<element,std::vector<element>,std::greater<element>>();
      m_conflictIntervalMap[robot].push(elem);
    }
  }

  // Otherwise, check if there are any overlaps between the window and prior
  // resolutions
  else {
    //size_t consistentWindow = m_windowSize;
    auto min = _conflict.timestep - m_windowSize;
    if(_conflict.timestep < m_windowSize)
      min = 0;
    auto max = _conflict.timestep + m_windowSize;

    std::queue<Robot*> queue;
    std::set<Robot*> merged;
    for(auto cfg : _conflict.cfgs) {
      auto robot = cfg.GetRobot();
      queue.push(robot);
      merged.insert(robot);
    }

    while(!queue.empty()) {
      auto robot = queue.front();
      queue.pop();

      auto pq = m_conflictIntervalMap[robot];
      //m_conflictIntervalMap[robot] = std::priority_queue<element,std::vector<element>,std::greater<element>>();
      while(!pq.empty()) {
      //for(auto iter = pq.begin(); iter!= pq.end(); iter++) {
        auto elem = pq.top();
        pq.pop();
        //auto elem = *iter;

        bool intersect = false;

        if(elem.interval.Contains(min)) {
          intersect = true;
          //elem.interval.max = min;
          //m_conflictIntervalMap[robot].push(elem);
        }
        else if(elem.interval.Contains(max)) {
          intersect = true;
          //elem.interval.min = max;
          //m_conflictIntervalMap[robot].push(elem);
        }
        else if(min <= elem.interval.min and max >= elem.interval.max) {
          intersect = true;
        }
        else if(min >= elem.interval.min and
            max <= elem.interval.max) {
          intersect = true;
        }

        if(intersect) {
          for(auto r : elem.robots) {
            /*bool contained = false;
            for(auto cfg : _conflict.cfgs) {
              if(cfg.GetRobot() == r) {
                contained = true;
                break;
              }
            }
            if(contained)*/
            if(merged.count(r))
              continue;

            merged.insert(r);
            queue.push(r);

            auto path = this->GetPath(r);
            auto cfgs = path->FullCfgs(this->GetMPLibrary());
            auto ts = std::min(_conflict.timestep,cfgs.size()-1);
            auto c = cfgs[ts];
            _conflict.cfgs.push_back(c);
            //m_windowSize = std::max(m_windowSize,(elem.interval.max-elem.interval.min));
          }
        }
        //else {
          //m_conflictIntervalMap[robot].push(elem);
        //}
      }
    }
  }

  return;

  // to compare every pair of cfg we createa copy of conflict in the form of an array of cfgs that contains
  // both the cfgs from the original conflict and the cfgs from the global paths
  Conflict conflictCopy = _conflict;
  for(Robot* const& robot : *this->GetGroupTask()->GetRobotGroup()) {

    // safe check: we dont want any robot from globalPath that already exists in conflict
    bool existingRobot = false;
    for(auto cfg : _conflict.cfgs) {
      if(robot == cfg.GetRobot()) {
        existingRobot = true;
      }
    }
    if(existingRobot)
      continue;

    auto fullGlobalPath = this->GetPath(robot)->FullCfgs(this->GetMPLibrary());
    const size_t step = std::min(fullGlobalPath.size() - 1, _conflict.timestep);
    const auto& globalCfg = fullGlobalPath[step];
    conflictCopy.cfgs.push_back(globalCfg);

  }

  // compare every pair of cfgs and only add the ones that are in conflict
  for(auto cfg1 : _conflict.cfgs) {
    for(auto cfg2 : conflictCopy.cfgs) {

      // if same cfg skip
      if(cfg1 == cfg2) {
        continue;
      }

      // check for collisions
      auto multibody1 = cfg1.GetRobot()->GetMultiBody();
      auto multibody2 = cfg2.GetRobot()->GetMultiBody();

      multibody1->Configure(cfg1);
      multibody2->Configure(cfg2);

      auto vc = static_cast<CollisionDetectionValidity<MPTraits>*>(
          this->GetValidityChecker("pqp_solid")
      );

      CDInfo cdInfo;
      const bool collision = vc->IsMultiBodyCollision(cdInfo, multibody1,
        multibody2, this->GetNameAndLabel());

      // if collision we add the second cfg
      if(collision) {
        // if conflict cfg is not already added
        if(std::find(_conflict.cfgs.begin(), _conflict.cfgs.end(), cfg2) == _conflict.cfgs.end()) {
          if(this->m_debug) {
            std::cout << "\nMerging " << cfg2.GetRobot()->GetLabel() << " to the conflict!!\n";
          }
          _conflict.cfgs.push_back(cfg2);
        }
      }
    }
  }
}





/*--------------------------- Single Repair -----------------------*/

template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
SingleResolution(typename ARCStrategy<MPTraits>::QueriesCfg& _queries, std::string _localStrategy) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "SingleResolution");

  /// We store the global boundary setting, by time we are ready with local repair,
  /// we set it back and we can return to the global task.
  this->GetEnvironment()->SaveBoundary();

  // Here we are going to try how the algorithm behaves using CSpaceBoundingSpheres.
  auto dm = this->GetMPLibrary()->GetDistanceMetric("euclidean");
  std::map<Robot*,std::unique_ptr<Boundary>> boundariesPtr;

  auto queries = _queries;

  for(auto query : queries) {
    auto start = query.first;
    auto goal = query.second;
    auto robot = start.GetRobot();
    auto centerCfg = (start + goal) / 2;
    auto centerPos = centerCfg.GetPosition();

    //auto radius = dm->Distance(start,goal) * m_radius + 2 * robot->GetMultiBody()->GetBoundingSphereRadius();
    auto radius = (dm->Distance(start,goal) * m_radius);
    // TODO::Change m_minRadius to be a factor of the robot bounding sphere
    // radius
    radius = std::max(radius,m_minRadius);
    m_localRadius = radius;
    // we record only one of the robot's local region radius
    if(numExp == 1)
      this->GetStatClass()->SetStat("initial local region radius", m_localRadius);
    numExp++;
    auto boundary = std::unique_ptr<Boundary>(new CSpaceBoundingSphere(centerCfg.GetData(), radius));
    boundariesPtr[robot] = std::move(boundary);
    m_boundaries[robot] = boundariesPtr[robot].get();

    auto boundingSphere = new CSpaceBoundingSphere(centerCfg.GetData(), radius);
    m_boundingSpheres[robot] = boundingSphere;
  
  }

  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  //vc->SetLocalBoundary(boundary.get());
  vc->SetLocalBoundaries(m_boundaries);

  if(this->m_debug) {
    for(auto & boundary : m_boundaries)
      std::cout << "Resetting robot's " << boundary.first->GetLabel()<< " local boundary: " << *boundary.second << std::endl;
  }

  /// Creating and setting local group task.
  auto robots = GetRobots(queries);
  auto localGroupTaskSet = CreateLocalTaskSet(queries);
  RobotGroup* localRobotGroup = new RobotGroup(this->GetMPLibrary()->GetMPProblem(), robots);
  std::shared_ptr<GroupTask> localGroupTask(new GroupTask(localRobotGroup, localGroupTaskSet));
  this->GetMPLibrary()->SetGroupTask(localGroupTask.get());
  this->GetMPLibrary()->GetMPSolution()->AddRobotGroup(localRobotGroup);
  auto groupRoadmap = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(localRobotGroup);
  this->GetMPLibrary()->GetGoalTracker()->AddMap(groupRoadmap,localGroupTask.get());

  // Setting back to zero some stats
  this->GetStatClass()->SetStat("Free samples", 0);
  this->GetStatClass()->SetStat("Obstacle samples", 0);

  // Initialize the iterationEval map evaluator; if the instance is global we set the flag
  auto me = dynamic_cast<IterationCountEvaluator<MPTraits>*>(this->GetMapEvaluator(m_iterationEval));
  if(me) {
    if(!m_ifGlobal)
      me->Initialize();
    else
      me->InitializeGlobal();
  }

  bool success = true;
  if(!m_ifHierarchical or m_ifGlobal) {
    // Running the single local strategy
    auto s = this->GetMPStrategy(_localStrategy);
    s->SetLocalBoundaries(m_boundaries);
    (*s)();

    success = IsValidSolution(localRobotGroup, robots, _localStrategy);
  } else {
    // Running the sequence of strategies
    for(size_t i = 0; i < m_localStrategies.size(); i++) {
      _localStrategy = m_localStrategies[i];

      auto s = this->GetMPStrategy(_localStrategy);
      s->SetLocalBoundaries(m_boundaries);
      if(me) {
        if(!m_ifGlobal)
          me->Initialize();
        else
          me->InitializeGlobal();
      }        
      {
      MethodTimer mt(this->GetStatClass(),
        this->GetNameAndLabel() + "RunningLocalMPStrategy");
      (*s)();
      }
      // m_currentTerminationCount++;

      success = IsValidSolution(localRobotGroup, robots, _localStrategy);

      if(success) {
        this->GetStatClass()->IncStat("times used " + _localStrategy);
        break;
      }
    }
  }


  if(m_printLocal) {
    for(auto query : _queries) {
      auto robot = query.first.GetRobot();
      auto totalConflicts = this->GetStatClass()->GetStat("TotalConflicts");
      const std::string base = this->GetBaseFilename();
      auto localRoadmap = this->GetRoadmap(robot);
      localRoadmap->Write(base +"."+ robot->GetLabel() + ".conflict" + std::to_string(totalConflicts) + ".local.map",
        this->GetEnvironment());
    }
  }


  // Restoring global settings
  this->GetEnvironment()->RestoreBoundary();
  this->GetMPLibrary()->SetGroupTask(m_groupTask);
  this->GetMPLibrary()->GetGoalTracker()->Clear(groupRoadmap,localGroupTask.get());

  return success;
}


template<typename MPTraits>
std::vector<Robot*>
ARCStrategy<MPTraits>::
GetRobots(typename ARCStrategy<MPTraits>::QueriesCfg _queries) {

  std::vector<Robot*> robots;
  for(auto query : _queries){
    auto robot = query.first.GetRobot();
    robots.push_back(robot);
  }
  return robots;
}

template<typename MPTraits>
std::vector<Robot*>
ARCStrategy<MPTraits>::
GetRobots(GroupTask* _task) {
  std::vector<Robot*> robots;
  for(auto& task : *_task) {
    // Ensure a maximum of one task per robot.
    auto robot = task.GetRobot();
    robots.push_back(robot);
  }
  return robots;
}


template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
IsValidSolution(RobotGroup* _robotGroup, std::vector<Robot*> _robots, std::string _localStrategy) {

  bool success = true;
  auto groupPath = this->GetMPLibrary()->GetGroupPath(_robotGroup);

  bool groupPathEmpty = false;

  // check for group path returned from coupled strategy
  if(groupPath->Empty()) {
    groupPathEmpty = true;
  }

  // check for individual paths returned from decoupled strategy
  for(auto robot : _robots) {
    if(groupPath->GetIndividualPath(robot).Empty() && groupPathEmpty) {
      success = false;
      return success;
    }
  }

  // NOTE: not sure what the best practice here is; seems that decoupledPRM
  // would also return a groupPath so I cant use that as the if condition
  auto s = dynamic_cast<GroupPRM<MPTraits>*>(this->GetMPStrategy(_localStrategy));
  if(s)
    GenerateCoupled(_robotGroup);
  else
    GenerateDecoupled(_robotGroup);

  return success;
}

template<typename MPTraits>
bool
ARCStrategy<MPTraits>::
PostValidation(Conflict& _conflict) {

  // We get local paths (in cfgs) of each agent in the local resolution,
  // and the global paths too.
  std::set<Robot*> allRobots;
  std::queue<Robot*> robotQueue;
  bool conflictFree = true;
  if(_conflict.cfgs.size() == this->GetGroupTask().Size())
    return true;

  std::map<Robot*,std::vector<CfgType>> localPaths;
  for(auto solution : m_localSolution) {
    auto localPath = solution.second.get();
    localPaths[solution.first] = localPath->FullCfgs(this->GetMPLibrary());
    robotQueue.push(solution.first);
  }
  std::map<Robot*,std::vector<CfgType>> globalPaths;
  for(Robot* const& robot : *this->GetGroupTask()->GetRobotGroup()) {
    allRobots.insert(robot);
    auto globalPath = this->GetPath(robot);
    globalPaths[robot] = globalPath->FullCfgs(this->GetMPLibrary());
  }
  // We initially only check the robots in the conflict, if a new robot is
  // disccovered, we analize its path too. Then we add those robots to the
  // conflict.
  while(!robotQueue.empty()) {
    auto robot = robotQueue.front();
    robotQueue.pop();
    auto iter = allRobots.find(robot);
    if(iter == allRobots.end())
      continue;
    else
      allRobots.erase(iter);;
    auto newRobots = SingleRobotPostValidation(_conflict, robot, localPaths, globalPaths);
    if(!newRobots.empty()) {
      conflictFree = false;
      for(auto& newRobot : newRobots) {
        robotQueue.push(newRobot);
        auto globalPath = globalPaths[newRobot];
        auto timestep = std::min(_conflict.timestep,globalPath.size()-1);
        _conflict.cfgs.push_back(globalPath[timestep]);
      }
    }
  }
  return conflictFree;
}


template<typename MPTraits>
std::vector<Robot*>
ARCStrategy<MPTraits>::
SingleRobotPostValidation(Conflict& _c, Robot* _robot, std::map<Robot*, std::vector<CfgType>> _localPaths, 
  std::map<Robot*,std::vector<CfgType>> _globalPaths) {


  std::vector<Robot*> newRobots;
  std::set<Robot*> conflictingRobots;
  for(auto& cfg : _c.cfgs)
    conflictingRobots.insert(cfg.GetRobot());
  bool isLocal = false;

  auto vc = static_cast<CollisionDetectionValidity<MPTraits>*>(
      this->GetValidityChecker("pqp_solid")
  );

  for(auto globalPath : _globalPaths) {
    // We set the first timestep on globalPaths to be time-consistent
    size_t globalTimestep;
    if (_c.timestep <= m_windowSize)
      globalTimestep = 0;
    else
      globalTimestep = _c.timestep - m_windowSize;
    // We don't want to compare the same robots
    if(_robot == globalPath.first)
      continue;
    if(_localPaths.find(_robot) != _localPaths.end())
      isLocal = true;
    for(size_t i = 0 ; i < 2 * m_windowSize ; ++i) {

        auto robot1        = _robot;
        auto robot2        = globalPath.first;
        // if the other robot already belongs to current conflict we don't need
        // to add it again.
        if(conflictingRobots.find(robot2) != conflictingRobots.end())
          break;
        // Configure the first robot (from localPath) at the approriate configuration.
        std::vector<CfgType> path1;
        size_t step1;
        if(isLocal) {
          path1 = _localPaths[_robot];
          step1 = std::min(i, path1.size() - 1);
        } else {
          path1 = _globalPaths[_robot];
          step1 = std::min(globalTimestep + i, path1.size() - 1);
        }
        const auto& cfg1   = path1[step1];
        auto multibody1    = robot1->GetMultiBody();
        multibody1->Configure(cfg1);
        // Configure the second robot (from globalPath) at the appropriate configuration.
        const auto& path2  = globalPath.second;
        const size_t step2 = std::min(globalTimestep + i, path2.size() - 1);
        const auto& cfg2   = path2[step2];
        auto multibody2    = robot2->GetMultiBody();
        multibody2->Configure(cfg2);

        // Check for collision. If none, move on.
        CDInfo cdInfo;
        const bool collision = vc->IsMultiBodyCollision(cdInfo,
            multibody1, multibody2, this->GetNameAndLabel());
        if(collision) {
          std::cout << "\t\t\t\n\nAdding  robot " << robot2->GetLabel() <<
            " to conflict resolution\n\n" << std::endl;
          newRobots.push_back(robot2);
          break;
        }
    }

  }
  return newRobots;
}



/*--------------------- Local Solution Finalization ---------------------*/



template<typename MPTraits>
void
ARCStrategy<MPTraits>::
SetLocalSolution(typename ARCStrategy<MPTraits>::Solution _localSolution) {
  m_localSolution = _localSolution;
}



template<typename MPTraits>
void
ARCStrategy<MPTraits>::
GenerateDecoupled(RobotGroup* _group) {

  auto robots = _group->GetRobots();

  typename ARCStrategy<MPTraits>::Solution localSolution;
  for(auto robot : robots) {
    Path* path = this->GetPath(robot);
    auto out = std::shared_ptr<Path>(new Path(std::move(*path)));
    localSolution[robot] = out;
  }
  SetLocalSolution(localSolution);
}


template<typename MPTraits>
void
ARCStrategy<MPTraits>::
GenerateCoupled(RobotGroup* _group) {

  auto robots = _group->GetRobots();
  auto groupPath = this->GetMPLibrary()->GetGroupPath(_group);
  typename ARCStrategy<MPTraits>::Solution localSolution;

  for(auto robot : robots) {
    auto path = groupPath->GetIndividualPath(robot);
    auto out = std::shared_ptr<Path>(new Path(std::move(path)));
    localSolution[robot] = out;
  }
  SetLocalSolution(localSolution);
}



/*--------------------------  Utilities  -------------------------------*/
template<typename MPTraits>
void
ARCStrategy<MPTraits>::
PrintLocalPath(Robot* _robot) {
  if(m_localSolution.find(_robot) == m_localSolution.end())
    return;

  auto path = m_localSolution[_robot];
  auto totalConflicts = this->GetStatClass()->GetStat("TotalConflicts");
  const std::string base = this->GetBaseFilename();
  ::WritePath(base + "." + _robot->GetLabel() + ".conflict" + std::to_string(totalConflicts) + ".rdmp.local.path", path->Cfgs());
  ::WritePath(base + "." + _robot->GetLabel() + ".conflict" + std::to_string(totalConflicts) + ".local.path",
    path->FullCfgs(this->GetMPLibrary()));
}

template<typename MPTraits>
void
ARCStrategy<MPTraits>::
PrintLocalRoadmap(Robot* _robot) {
  if(m_boundingSpheres.find(_robot) == m_boundingSpheres.end())
    return;

  auto roadmap = this->GetRoadmap(_robot);
  auto boundary = m_boundingSpheres[_robot];
  auto totalConflicts = this->GetStatClass()->GetStat("TotalConflicts");
  const std::string base = this->GetBaseFilename();

  // make a copy of the roadmap since we don't want to mess up
  // with the current one
  auto roadmapCopy = roadmap;

  // getting rid of the nodes and edges that are not in the local boundary
  // and we have a local roadmap
  auto allVIDs = roadmapCopy->GetAllVIDs();

  for(auto vid : allVIDs) {
    if(boundary->InBoundary(roadmapCopy->GetVertex(vid))) {
      // auto children = roadmapCopy->GetChildren(vid);
      if(this->m_debug) {
        std::cout << "vids that are in local boundary: " << "for robot " << _robot->GetLabel() << std::endl;
        std::cout << vid << std::endl;
      }
      roadmapCopy->DeleteVertex(vid);
    }

  }

  roadmapCopy->Write(base +"."+ _robot->GetLabel() + ".conflict" + std::to_string(totalConflicts) + ".local.map",
    this->GetEnvironment());
}



#endif

