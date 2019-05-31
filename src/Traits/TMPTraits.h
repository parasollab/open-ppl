#ifndef PMPL_TMP_TRAITS_H_
#define PMPL_TMP_TRAITS_H_

#include "TMPLibrary/TMPLibrary.h"

// TMPStrategyMethods to include

#include "TMPLibrary/TMPStrategies/ITMethod.h"

// PoIPlacementMethods to include

#include "TMPLibrary/PoIPlacementMethods/ITPlacementMethods/DisjointWorkspaces.h"
#include "TMPLibrary/PoIPlacementMethods/ITPlacementMethods/WorkspaceGuidance.h"

// TaskEvaluators to include

#include "TMPLibrary/TaskEvaluators/EnforcedHillClimbing.h"

// TaskDecomposers to include

#include "TMPLibrary/TaskDecomposers/InteractionTaskBreakup.h"

// TaskAllocators to include 

#include "TMPLibrary/TaskAllocators/AuctionMethod.h"

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
    ITMethod
      > TMPStrategyMethodList;

  //types of points of interest placement methods available in our world
  typedef boost::mpl::list<
    DisjointWorkspaces,
    WorkspaceGuidance
      > PoIPlacementMethodList;

  //types of task evaluators available in our world
  typedef boost::mpl::list<
    EnforcedHillClimbing
      > TaskEvaluatorMethodList;

  //types of task decomposers available in our world
  typedef boost::mpl::list<
    InteractionTaskBreakup
      > TaskDecomposerMethodList;

  //types of task allocators available in our world
  typedef boost::mpl::list<
    AuctionMethod
      > TaskAllocatorMethodList;
};

#endif
