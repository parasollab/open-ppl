#ifndef COLLISION_DETECTION_METHOD_H_
#define COLLISION_DETECTION_METHOD_H_

#include <iostream>
#include <memory>
#include <string>
using namespace std;

#include "Vector.h"
using namespace mathtool;

class Body;
class ActiveMultiBody;
class CDInfo;
class Cfg;
class MultiBody;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief Base abstraction for \ref CollisionDetection.
///
/// CollisionDetectionMethod is base class for geometric collision detection
/// methods. Mostly these serve as middleware to interface with external
/// libraries. @cd methods are not directly accessed but have two core
/// functions, @c IsInCollision and @c IsInsideObstacle. @c IsInCollision takes
/// as input a MultiBody for a robot and an obstacle and returns whether the
/// robot and obstacle collide. @c IsInsideObstacle takes a configuration @c c
/// and determines whether the robot configured at @c c lies entirely within a
/// workspace obstacle.
////////////////////////////////////////////////////////////////////////////////
class CollisionDetectionMethod {
  public:
    //Type Out: no collision sure; collision unsure.
    //Type In: no collision unsure; collision sure.
    //Type Exact: no collision sure; collision sure.
    enum class CDType {Out, In, Exact};

    CollisionDetectionMethod(const string& _name = "CD_USER1",
        CDType _type = CDType::Out);
    virtual ~CollisionDetectionMethod();

    string GetName() const {return m_name;}
    CDType GetType() const {return m_type;}

    virtual void Print(ostream& _os) const;

    virtual void Build(Body* _body) = 0;

    /**
     * Check if robot in given cfg is complete inside or outside obstacle.
     *
     * The precondition is that robot is collision free
     * in this given cfg. (i.e no intersections among boundaries of robot and obs)
     * return true, if robot is completely contained inside any obs.
     * otherwise, false will be returned.
     */
    virtual bool IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body);

    /**Check collision between two bodies
    */
    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo) = 0;

  protected:
    string m_name;
    CDType m_type; ///<Out, In, or Exact. Used to classify CD functions.
};

#endif
