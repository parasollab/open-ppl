#ifndef COLLISIONDETECTIONMETHOD_H
#define COLLISIONDETECTIONMETHOD_H

#include "Utilities/MPUtils.h"

class MultiBody;
class StatClass;
class Cfg;
class CDInfo;

class CollisionDetectionMethod {
  public:
    //Type Out: no collision sure; collision unsure.
    //Type In: no collision unsure; collision sure.
    //Type Exact: no collision sure; collision sure.
    enum CDType {Out, In, Exact};

    CollisionDetectionMethod(string _name = "CD_USER1", CDType _type = Out, cd_predefined _cdType = CD_USER1);
    virtual ~CollisionDetectionMethod();

    string GetName() const {return m_name;}
    CDType GetType() const {return m_type;}
    cd_predefined GetCDType() const {return m_cdType;}

    virtual bool operator==(const CollisionDetectionMethod& _cd) const;

    virtual void PrintOptions(ostream& _os) const;

    /**
     * Check if robot in given cfg is complete inside or outside obstacle.
     *
     * The precondition is that robot is collision free
     * in this given cfg. (i.e no intersections among boundaries of robot and obs)
     * return true, if robot is completely contained inside any obs.
     * otherwise, false will be returned.
     */
    virtual bool IsInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo);

    /**Check collision between MultiBody of robot and obstacle.
    */
    virtual bool IsInCollision(shared_ptr<MultiBody> _rob,
        shared_ptr<MultiBody> _obstacle, StatClass& _Stats, CDInfo& _cdInfo,
        const string& _callName, int _ignoreIAdjacentMultibodies = 1) = 0;

  protected:
    string m_name;
    CDType m_type; ///<Out, In, or Exact. Used to classify CD functions.
    cd_predefined m_cdType;
};

#endif
