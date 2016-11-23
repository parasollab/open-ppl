#ifndef CONTROL_H_
#define CONTROL_H_

#include <cstddef>
#include <set>
#include <vector>

class Actuator;
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
  ///@name Simulation Interface
  ///@{

  /// Ask the actuator to compute the generalized force for this control.
  std::vector<double> GetForce() const;

  /// Tell the actuator to execute this control.
  void Execute() const;

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

#endif
