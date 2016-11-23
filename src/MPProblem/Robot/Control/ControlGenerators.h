#ifndef CONTROL_GENERATORS_H_
#define CONTROL_GENERATORS_H_

#include <set>

#include "Control.h"

class Robot;


////////////////////////////////////////////////////////////////////////////////
/// A functor for generating specific sets of controls.
////////////////////////////////////////////////////////////////////////////////
struct ControlGenerator {

  virtual ~ControlGenerator() = default;

  /// Generate a discrete set of controls for a robot.
  /// @param[in] _r The robot to generate controls for.
  /// @return A discrete set of controls for the given robot.
  virtual std::set<Control> GenerateDiscreteSet(Robot* const _r) const = 0;

  /// @TODO Add a method for generating a control space instead of a discrete
  ///       control set. This will probably look like a boundary object.

};


////////////////////////////////////////////////////////////////////////////////
/// Generate a discrete set of controls for each actuator, including full
/// forward, full reverse, and coast for each DOF (individually). Does not
/// include simultaneous actuation of multiple DOFs.
////////////////////////////////////////////////////////////////////////////////
struct SimpleControlGenerator : public ControlGenerator {

  virtual ~SimpleControlGenerator() = default;

  virtual std::set<Control> GenerateDiscreteSet(Robot* const _r) const override;

};

#endif
