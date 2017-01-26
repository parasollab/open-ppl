#ifndef CONTROL_H_
#define CONTROL_H_

#include <cstddef>
#include <iostream>
#include <set>
#include <vector>

class Actuator;
class btMultiBody;
class Robot;


////////////////////////////////////////////////////////////////////////////////
/// Models the instructions that a robot sends to its actuators.
///
/// @details A control is modeled as a 'relative power' signal for each of
///          the robot's DOFs. Controls are sent to a specific actuator. A
///          strength in the range [-1:1] describes how to drive the actuator's
///          mechanism for that DOF, with -1 representing full reverse and 1
///          representing full forward.
////////////////////////////////////////////////////////////////////////////////
struct Control final {

  ///@name Local Types
  ///@{

  typedef std::vector<double> Signal;

  ///@}
  ///@name Public Data
  ///@{

  Actuator* actuator; ///< The actuator for this control.
  Signal signal;      ///< The signal for this control.

  ///@}
  ///@name Planning Interface
  ///@{

  /// Ask the actuator to compute the generalized force for this control.
  std::vector<double> GetForce() const;

  ///@}
  ///@name Simulation Interface
  ///@{

  /// Tell the actuator to execute this control on the simulated robot.
  void Execute() const;

  /// Tell the actuator to execute this control on an arbitrary simulated robot.
  /// @param _model The simulated robot to use.
  void Execute(btMultiBody* const _model) const;

  ///@}
  ///@name Ordering and Equality
  ///@{

  /// Define a weak ordering over controls to allow sorting/sequencing.
  bool operator<(const Control& _rhs) const noexcept;

  /// Define equality operators to check against identical controls.
  bool operator==(const Control& _rhs) const noexcept;
  bool operator!=(const Control& _rhs) const noexcept;

  ///@}

};

/// Display function for debugging controls. Prints the signal.
std::ostream& operator<<(std::ostream&, const Control&);

typedef std::vector<Control> ControlSet;

/// @TODO Add real abstraction for control space. This will probably look like a
///       boundary object for each actuator.
typedef int ControlSpace;

#endif
