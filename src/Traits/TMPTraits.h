#ifndef PPL_TMP_TRAITS_H_
#define PPL_TMP_TRAITS_H_

#include "TMPLibrary/TMPLibrary.h"

// TMPStrategyMethods to include

#include "TMPLibrary/TMPStrategies/SimpleMotionMethod.h"
#include "TMPLibrary/TMPStrategies/SimpleTaskAllocationMethod.h"

// PoIPlacementMethods to include

// TaskEvaluators to include

#include "TMPLibrary/TaskEvaluators/SimpleMotionEvaluator.h"

// TaskDecomposers to include

// TaskAllocators to include
#include "TMPLibrary/TaskAllocators/GreedyAllocator.h" 
#include "TMPLibrary/TaskAllocators/SmartAllocator.h" 

// StateGraphs to include

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
		SimpleTaskAllocationMethod,
		SimpleMotionMethod
      > TMPStrategyMethodList;

  //types of points of interest placement methods available in our world
  typedef boost::mpl::list<
      > PoIPlacementMethodList;

  //types of task evaluators available in our world
  typedef boost::mpl::list<
		SimpleMotionEvaluator
      > TaskEvaluatorMethodList;

  //types of task decomposers available in our world
  typedef boost::mpl::list<
      > TaskDecomposerMethodList;

  //types of task allocators available in our world
  typedef boost::mpl::list<
    GreedyAllocator,
    SmartAllocator
      > TaskAllocatorMethodList;

	typedef boost::mpl::list<
			> StateGraphList;

};

#endif
