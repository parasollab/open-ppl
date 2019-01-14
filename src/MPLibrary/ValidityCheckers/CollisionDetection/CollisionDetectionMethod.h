#ifndef PMPL_COLLISION_DETECTION_METHOD_H_
#define PMPL_COLLISION_DETECTION_METHOD_H_

#include "Vector.h"

#include <iostream>
#include <string>

class Body;
class CDInfo;


////////////////////////////////////////////////////////////////////////////////
/// Base abstraction for \ref CollisionDetection.
///
/// CollisionDetectionMethod is a base class for geometric collision detection
/// methods. Mostly these serve as middleware to interface with external
/// libraries. @cd methods are not directly accessed but have two core
/// functions, @c IsInCollision and @c IsInsideObstacle. @c IsInCollision takes
/// as input two @c Body and determine if they collide. @c IsInsideObstacle
/// takes a point and a @c Body to determine if the point is inside of the body.
///
/// @ingroup CollisionDetection
////////////////////////////////////////////////////////////////////////////////
class CollisionDetectionMethod {

  public:

    ///@name Construction
    ///@{

    /// @param _name The method name.
    CollisionDetectionMethod(const std::string& _name = "CD_USER1");

    virtual ~CollisionDetectionMethod();

    ///@}
    ///@name Accessors
    ///@{

    /// @return Name of CD Method
    const std::string& GetName() const;

    /// Print information to an output stream.
    /// @param _os The output stream.
    virtual void Print(std::ostream& _os) const;

    ///@}
    ///@name CD Interface
    ///@{

    /// Check if two bodies are in collision.
    /// @param _body1  The first Body.
    /// @param _body2  The second Body.
    /// @param _cdInfo Output information from the collision computation.
    /// @return Do the bodies touch?
    virtual bool IsInCollision(const Body* const _body1,
        const Body* const _body2, CDInfo& _cdInfo) = 0;

    /// Check if a point is inside of a body.
    /// @param _pt   The point to check.
    /// @param _body The body to check against.
    /// @return Is the point inside the body?
    virtual bool IsInsideObstacle(const mathtool::Vector3d& _pt,
        const Body* const _body);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::string m_name; ///< Name of the CD method.

    ///@}

};

#endif
