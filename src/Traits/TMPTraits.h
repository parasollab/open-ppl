#ifndef PMPL_TMP_TRAITS_H_
#define PMPL_TMP_TRAITS_H_

#include "TMPLibrary/TMPLibrary.h"

// TMPStrategyMethods to include
#include "TMPLibrary/TMPStrategies/SimpleMotionMethod.h"

// TaskEvaluators to include
#include "TMPLibrary/TaskEvaluators/SimpleMotionEvaluator.h"

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
		SimpleMotionMethod
      > TMPStrategyMethodList;

  //types of task evaluators available in our world
  typedef boost::mpl::list<
		SimpleMotionEvaluator
      > TaskEvaluatorMethodList;
};

#endif
