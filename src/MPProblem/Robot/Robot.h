#ifndef ROBOT_H_
#define ROBOT_H_

#include <string>

#include "Geometry/Bodies/ActiveMultiBody.h"

class Boundary;

////////////////////////////////////////////////////////////////////////////////
/// @brief Representation of a robot that has geometry, dynamics, and a
///        controller.
////////////////////////////////////////////////////////////////////////////////
class Robot {

  ///@name Internal State
  ///@{

  ActiveMultiBody m_multibody;  ///< The robot's geometric representation.

  std::string m_label;          ///< The robot's unique label.

  ///@}

  public:

    ///@name Construction
    ///@{

    Robot() = default;
    Robot(XMLNode& _node);
    virtual ~Robot() = default;

    ///@}
    ///@name Geometry Accessors
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Set the boundary to determine the DOF ranges.
    void SetBoundary(const Boundary* _b);

    ActiveMultiBody* GetMultiBody();
    const ActiveMultiBody* GetMultiBody() const;

    ///@}
    ///@name Controller Accessors
    ///@{


    ///@}
    ///@name Dynamics Accessors
    ///@{


    ///@}
};

/*----------------------------- Inlined Functions ----------------------------*/

/*---------------------------- Geometry Accessors ----------------------------*/

inline
ActiveMultiBody*
Robot::
GetMultiBody() {
  return &m_multibody;
}


inline
const ActiveMultiBody*
Robot::
GetMultiBody() const {
  return &m_multibody;
}

/*----------------------------------------------------------------------------*/

#endif
