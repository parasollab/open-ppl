#ifndef COLLISION_DETECTION_METHOD_H_
#define COLLISION_DETECTION_METHOD_H_

#include <memory>
#include <string>
using namespace std;

#include "Vector.h"
using namespace mathtool;

class Body;
class CDInfo;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief Base abstraction for \ref CollisionDetection.
///
/// CollisionDetectionMethod is a base class for geometric collision detection
/// methods. Mostly these serve as middleware to interface with external
/// libraries. @cd methods are not directly accessed but have two core
/// functions, @c IsInCollision and @c IsInsideObstacle. @c IsInCollision takes
/// as input two @c Body and determine if they collide. @c IsInsideObstacle
/// takes a point and a @c Body to determine if the point is inside of the body.
////////////////////////////////////////////////////////////////////////////////
class CollisionDetectionMethod {

  public:

    ///@name Construction
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _name The method name.
    CollisionDetectionMethod(const string& _name = "CD_USER1");

    virtual ~CollisionDetectionMethod() = default;

    ///@}
    ///@name Accessors
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Name of CD Method
    const string& GetName() const {return m_name;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Output class information
    /// @param _os Output stream
    virtual void Print(ostream& _os) const;

    ///@}
    ///@name CD Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build a representation of a body for this CD library.
    /// @param _body The body of interest.
    virtual void Build(Body* _body) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check if two bodies are in collision.
    /// @param[in]  _body1  The first Body.
    /// @param[in]  _body2  The second Body.
    /// @param[out] _cdInfo Output information from the collision computation.
    /// @return Do the bodies touch?
    virtual bool IsInCollision(shared_ptr<Body> _body1, shared_ptr<Body> _body2,
        CDInfo& _cdInfo) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check if a point is inside of a body.
    /// @param[in] _pt   The point to check.
    /// @param[in] _body The body to check against.
    /// @return Is the point inside the body?
    virtual bool IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    string m_name; ///< Name of the CD method.

    ///@}
};

#endif
