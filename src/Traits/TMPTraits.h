#ifndef PPL_TMP_TRAITS_H_
#define PPL_TMP_TRAITS_H_

#include "TMPLibrary/TMPLibrary.h"

// TMPStrategyMethods to include
#include "TMPLibrary/TMPStrategies/BasicHCR.h"
#include "TMPLibrary/TMPStrategies/NextBestSearch.h"
#include "TMPLibrary/TMPStrategies/SimpleMotionMethod.h"
#include "TMPLibrary/TMPStrategies/BasicTMPStrategyMethod.h"

// PoIPlacementMethods to include

// TaskEvaluators to include
#include "TMPLibrary/TaskEvaluators/HCRQuery.h"
#include "TMPLibrary/TaskEvaluators/SimpleMotionEvaluator.h"
#include "TMPLibrary/TaskEvaluators/SimultaneousMultiArmEvaluator.h"
#include "TMPLibrary/TaskEvaluators/SubmodeQuery.h"

// TaskDecomposers to include

// TaskAllocators to include 

// StateGraphs to include
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/StateGraphs/ModeGraph.h"
#include "TMPLibrary/StateGraphs/ObjectCentricModeGraph.h"

// InteractionStrategyMethods to include
#include "TMPLibrary/InteractionStrategies/IndependentPaths.h"
#include "TMPLibrary/InteractionStrategies/GraspStrategy.h"
#include "TMPLibrary/InteractionStrategies/HandoffStrategy.h"
#include "TMPLibrary/InteractionStrategies/TemplateInteractions.h"

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
    BasicHCR,
    BasicTMPStrategyMethod,
    NextBestSearch,
		SimpleMotionMethod
      > TMPStrategyMethodList;

  //types of points of interest placement methods available in our world
  typedef boost::mpl::list<
      > PoIPlacementMethodList;

  //types of task evaluators available in our world
  typedef boost::mpl::list<
    HCRQuery,
		SimpleMotionEvaluator,
    SimultaneousMultiArmEvaluator,
    SubmodeQuery
      > TaskEvaluatorMethodList;

  //types of task decomposers available in our world
  typedef boost::mpl::list<
      > TaskDecomposerMethodList;

  //types of task allocators available in our world
  typedef boost::mpl::list<
      > TaskAllocatorMethodList;

	typedef boost::mpl::list<
    CombinedRoadmap,
    ModeGraph,
    ObjectCentricModeGraph
			> StateGraphList;

	typedef boost::mpl::list<
		IndependentPaths,
    GraspStrategy,
    HandoffStrategy,
    TemplateInteractions
			> InteractionStrategyMethodList;
};

#endif
