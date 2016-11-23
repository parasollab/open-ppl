#include "ControlGenerators.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Robot/Robot.h"


std::set<Control>
SimpleControlGenerator::
GenerateDiscreteSet(Robot* const _r) const {
  std::set<Control> controls;

  // Create coast action.
  Control c = {nullptr, Control::Signal(_r->GetMultiBody()->DOF(), 0)};
  controls.insert(c);

  // Create controls for each actuator.
  for(auto& actuator : _r->GetActuators()) {
    auto mask = actuator.ControlMask();
    c.actuator = &actuator;

    // Create a min and max action for each controllable DOF.
    for(size_t i = 0; i < mask.size(); ++i) {
      // Skip DOFs that this actuator can't control.
      if(!mask[i])
        continue;

      // Make a control for the forward signal.
      c.signal[i] = 1;
      controls.insert(c);

      // Make a control for the reverse signal.
      c.signal[i] = -1;
      controls.insert(c);

      c.signal[i] = 0;
    }
  }

  return controls;
}
