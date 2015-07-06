#ifndef COLLISION_DETECTION_METHOD_H_
#define COLLISION_DETECTION_METHOD_H_

#include <iostream>
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
/// CollisionDetectionMethod is base class for geometric collision detection
/// methods. Mostly these serve as middleware to interface with external
/// libraries. @cd methods are not directly accessed but have two core
/// functions, @c IsInCollision and @c IsInsideObstacle. @c IsInCollision takes
/// as input two @c Body and determine if they collide. @c IsInsideObstacle
/// takes a point and a @c Body to determine if the point is inside of the body.
////////////////////////////////////////////////////////////////////////////////
class CollisionDetectionMethod {
  public:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Type of collision detection computation
    ////////////////////////////////////////////////////////////////////////////
    enum class CDType {
      Out,  ///< No collision sure; collision unsure.
      In,   ///< No collision unsure; collision sure.
      Exact ///< No collision sure; collision sure.
    };

    ////////////////////////////////////////////////////////////////////////////
    /// @param _name Name of CD Method
    /// @param _type Type of CD computation
    CollisionDetectionMethod(const string& _name = "CD_USER1",
        CDType _type = CDType::Out);
    virtual ~CollisionDetectionMethod();

    ////////////////////////////////////////////////////////////////////////////
    /// @return Name of CD Method
    const string& GetName() const {return m_name;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Type of CD computation
    CDType GetType() const {return m_type;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Output class information
    /// @param _os Output stream
    virtual void Print(ostream& _os) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build representation for CD library
    /// @param _body Body to build representation of
    virtual void Build(Body* _body) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check if two Body are in collision
    /// @param _body1 First Body
    /// @param _body2 Second Body
    /// @param[out] _cdInfo CDInfo for output of collision computation
    /// @return Collision or not
    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check if point is inside of Body
    /// @param _pt Point
    /// @param _body Body
    /// @return Inside or not
    virtual bool IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body);

  protected:
    string m_name; ///< Name of CD method
    CDType m_type; ///< Type of CD computation
};

#endif
