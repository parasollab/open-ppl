#ifndef ROBOT_H_
#define ROBOT_H_

#include <string>

#include "Geometry/Bodies/ActiveMultiBody.h"

class Boundary;
class Controller;

////////////////////////////////////////////////////////////////////////////////
/// Representation of a robot that has geometry, dynamics, and a controller.
////////////////////////////////////////////////////////////////////////////////
class Robot {

  ///@name Internal State
  ///@{

  ActiveMultiBody m_multibody;       ///< The robot's geometric representation.
  Controller* m_controller{nullptr}; ///< The robot's controller.

  std::string m_label;               ///< The robot's unique label.

  ///@}

  public:

    ///@name Construction
    ///@{

    Robot() = default;
    Robot(XMLNode& _node);
    virtual ~Robot();

    ///@}
    ///@name Geometry Accessors
    ///@{

    /// Set the boundary to determine the DOF ranges.
    /// @param[in] _b The boundary to use.
    void SetBoundary(const Boundary* _b);

    ActiveMultiBody* GetMultiBody();
    const ActiveMultiBody* GetMultiBody() const;

    ///@}
    ///@name Controller Accessors
    ///@{

    /// Get the robot's controller.
    Controller* GetController();

    /// Set the robot's controller.
    /// @param[in] _c The controller to use. The robot will take ownership of
    ///               this controller and destruct it when necessary.
    void SetController(Controller* const _c);

    ///@}
    ///@name Dynamics Accessors
    ///@{


    ///@}
};

#endif
