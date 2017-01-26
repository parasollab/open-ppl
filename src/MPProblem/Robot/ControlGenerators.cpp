#include "ControlGenerators.h"

#include "Actuator.h"
#include "Robot.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Utilities/PMPLExceptions.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ControlGenerator ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

ControlGenerator::
~ControlGenerator() = default;


ControlSet*
ControlGenerator::
GenerateDiscreteSet(Robot* const _r) const {
  throw RunTimeException(WHERE, "Not implemented.");
  return nullptr;
}


ControlSpace*
ControlGenerator::
GenerateContinuousSpace(Robot* const _r) const {
  throw RunTimeException(WHERE, "Not implemented.");
  return nullptr;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~ SimpleControlGenerator ~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

ControlSet*
SimpleControlGenerator::
GenerateDiscreteSet(Robot* const _r) const {
  ControlSet* controls = new ControlSet;

  // Create coast action.
  Control c = {nullptr, Control::Signal(_r->GetMultiBody()->DOF(), 0)};
  controls->push_back(c);

  // Create forward and reverse controls for each actuator.
  for(auto& actuator : _r->GetActuators()) {
    auto mask = actuator->ControlMask();
    c.actuator = actuator;

    // Create a min and max action for each controllable DOF.
    for(size_t i = 0; i < mask.size(); ++i) {
      // Skip DOFs that this actuator can't control.
      if(!mask[i])
        continue;

      // Make a control for the forward signal.
      c.signal[i] = 1;
      controls->push_back(c);

      // Make a control for the reverse signal.
      c.signal[i] = -1;
      controls->push_back(c);

      c.signal[i] = 0;
    }
  }

  return controls;
}

/*----------------------------------------------------------------------------*/
