#ifndef COLLISIONDETECTIONMETHOD_H
#define COLLISIONDETECTIONMETHOD_H

#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPProblem/Geometry/MultiBody.h"

const int Out = 0;      ///<Type Out: no collision sure; collision unsure.
const int In = 1;       ///<Type In: no collision unsure; collision sure.
const int Exact = 2;    ///<Type Exact: no collision sure; collision sure.

class CollisionDetectionMethod {
 public:
  CollisionDetectionMethod();
  virtual ~CollisionDetectionMethod();

  string GetName() const {return m_name;}
  int GetType();
  cd_predefined GetCDType() const { return m_cdtype; }

  virtual bool operator==(const CollisionDetectionMethod& _cd) const;

  virtual void PrintOptions(ostream& _os) const;

  /**
   * Check if robot in given cfg is complete inside or outside obstacle.
   * @warning The precondition is that robot is collision free
   * in this given cfg. (i.e no intersections among boundaries of robot and obs)
   * @return True, if robot is completely contained inside any obs.
   * otherwise, false will be returned.
   */
  virtual bool IsInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo);
  //@}
  
  /**Check collision between MultiBody of robot and obstacle.
   */
  virtual bool IsInCollision(shared_ptr<MultiBody> _rob, shared_ptr<MultiBody> _obstacle, StatClass& _Stats, CDInfo& _cdInfo, std::string *_callName=NULL, int _ignoreIAdjacentMultibodies=1) = 0;

 protected:
  int m_type; ///<Out, In, or Exact. Used to classify CD functions.
  cd_predefined m_cdtype;
  string m_name;
};

#endif
