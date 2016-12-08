#ifndef DYNAMICS_MODEL_H_
#define DYNAMICS_MODEL_H_

#include <vector>

class btMultiBody;
class Robot;


////////////////////////////////////////////////////////////////////////////////
/// Wrapper class for interfacing with our physics engine representation of a
/// robot.
////////////////////////////////////////////////////////////////////////////////
class DynamicsModel final {

  ///@name Internal State
  ///@{

  Robot* const m_robot;       ///< The robot this model represents.
  btMultiBody* const m_model; ///< The physics engine model.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Create a wrapper around a physics engine model for a robot.
    /// @param[in] _r The robot being modeled.
    /// @param[in] _b The physics engine model of the robot.
    DynamicsModel(Robot* const _r, btMultiBody* const _b);

    ///@}
    ///@name Physics Model Accessors
    ///@{

    /// Get the underlying physics model.
    btMultiBody* Get() const noexcept;

    /// Allows implicit access to underlying model pointer.
    operator btMultiBody*() const noexcept;

    ///@}
    ///@name Interface
    ///@{

    /// Get the state of the robot in the simulation.
    std::vector<double> GetSimulatedState() const;

    ///@}

};

#endif
