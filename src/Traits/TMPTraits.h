#ifndef PMPL_TMP_TRAITS_H_
#define PMPL_TMP_TRAITS_H_

#include "TMPLibrary/TMPLibrary.h"

// TMPStrategyMethods to include

#include "TMPLibrary/TMPStrategies/DummyStrategyMethod.h"
#include "TMPLibrary/TMPStrategies/ITMethod.h"
#include "TMPLibrary/TMPStrategies/MultiAgentMultiTaskPlanner.h"
#include "TMPLibrary/TMPStrategies/SimpleMotionMethod.h"

// PoIPlacementMethods to include

#include "TMPLibrary/PoIPlacementMethods/ITPlacement/DisjointWorkspaces.h"
#include "TMPLibrary/PoIPlacementMethods/ITPlacement/OverlappingWorkspacesDensity.h"
#include "TMPLibrary/PoIPlacementMethods/ITPlacement/WorkspaceGuidance.h"

// TaskEvaluators to include

#include "TMPLibrary/TaskEvaluators/EnforcedHillClimbing.h"
#include "TMPLibrary/TaskEvaluators/OrderedMultiTaskEvaluator.h"
#include "TMPLibrary/TaskEvaluators/SimpleMotionEvaluator.h"
#include "TMPLibrary/TaskEvaluators/TaskCBSPlanner.h"
#include "TMPLibrary/TaskEvaluators/TCBS.h"
#include "TMPLibrary/TaskEvaluators/TMPCBS.h"

// TaskDecomposers to include

#include "TMPLibrary/TaskDecomposers/ITTaskBreakup.h"

// TaskAllocators to include 

#include "TMPLibrary/TaskAllocators/AuctionMethod.h"
#include "TMPLibrary/TaskAllocators/TCBSAllocator.h"

// StateGraphs to include

#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/StateGraphs/DiscreteIntervalGraph.h"
#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup TaskAndMotionPlanningUniverse
/// @brief Defines available methods in the Task and Motion Planning Universe
/// 
///
/// TMPTraits is a type class which defines the task and motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
////////////////////////////////////////////////////////////////////////////////
struct TMPTraits {

  //types of tmp strategy methods available in our world
  typedef boost::mpl::list<
    ITMethod,
		DummyStrategyMethod,
		MultiAgentMultiTaskPlanner,
		SimpleMotionMethod
      > TMPStrategyMethodList;

  //types of points of interest placement methods available in our world
  typedef boost::mpl::list<
    DisjointWorkspaces,
    WorkspaceGuidance,
		OverlappingWorkspacesDensity
      > PoIPlacementMethodList;

  //types of task evaluators available in our world
  typedef boost::mpl::list<
		OrderedMultiTaskEvaluator,
		SimpleMotionEvaluator,
		TaskCBSPlanner,
		TCBS,
		TMPCBS,
    EnforcedHillClimbing
      > TaskEvaluatorMethodList;

  //types of task decomposers available in our world
  typedef boost::mpl::list<
    ITTaskBreakup
      > TaskDecomposerMethodList;

  //types of task allocators available in our world
  typedef boost::mpl::list<
    AuctionMethod,
		TCBSAllocator
      > TaskAllocatorMethodList;

	typedef boost::mpl::list<
		CombinedRoadmap,
		DiscreteIntervalGraph,
		MultiTaskGraph
			> StateGraphList;
};

#endif
