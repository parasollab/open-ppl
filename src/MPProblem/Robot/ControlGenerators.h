#ifndef CONTROL_GENERATORS_H_
#define CONTROL_GENERATORS_H_

#include "Control.h"

class Robot;


////////////////////////////////////////////////////////////////////////////////
/// A functor for generating specific sets of controls.
////////////////////////////////////////////////////////////////////////////////
struct ControlGenerator {

  virtual ~ControlGenerator() = 0;

  /// Generate a discrete set of controls for a robot.
  /// @param[in] _r The robot to generate controls for.
  /// @return A discrete set of controls for the given robot.
  virtual ControlSet* GenerateDiscreteSet(Robot* const _r) const;

  /// Generate a continuous control space for each of a robot's actuators.
  /// @param[in] +r The robot to generate controls for.
  /// @return A continuous control space for each actuator.
  virtual ControlSpace* GenerateContinuousSpace(Robot* const _r) const;

};


////////////////////////////////////////////////////////////////////////////////
/// Generate a discrete set of controls for each actuator, including full
/// forward, full reverse, and coast for each DOF (individually). Does not
/// include simultaneous actuation of multiple DOFs.
////////////////////////////////////////////////////////////////////////////////
struct SimpleControlGenerator : public ControlGenerator {

  virtual ~SimpleControlGenerator() = default;

  virtual ControlSet* GenerateDiscreteSet(Robot* const _r) const override;

};

#endif
