#ifndef COLLISIONDETECTIONVALIDITY_H_
#define COLLISIONDETECTIONVALIDITY_H_

#include "ValidityCheckerMethod.hpp"
#include "CollisionDetectionMethod.h"

class CollisionDetectionValidity : public ValidityCheckerMethod 
{
public:
  CollisionDetectionValidity();
  CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod, bool _ignoreSelfCollision = false, int _ignoreIAdjacentLinks = 1);
  CollisionDetectionValidity(XMLNodeReader& _node, MPProblem* _problem);
  virtual ~CollisionDetectionValidity(); 
  
  virtual bool IsValidImpl(Cfg& _cfg, Environment* _env, 
  		       StatClass& _stats, CDInfo& _cdInfo, 
		       std::string *_callName);    
  virtual  bool IsInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo); 
  
  cd_predefined GetCDType() const { return m_cdMethod->GetCDType(); }

private:
  bool IsInCollision(Environment* _env, StatClass& _stats, CDInfo& _cdInfo,
		     shared_ptr<MultiBody> _rob, shared_ptr<MultiBody> _obst, std::string *_callName);
  bool IsInCollision(Environment* _env, StatClass& _stats, CDInfo& _cdInfo, 
		     std::string *_callName);

  CollisionDetectionMethod* m_cdMethod;
  bool m_ignoreSelfCollision;
  int m_ignoreIAdjacentLinks;
};

#endif // #ifndef COLLISIONDETECTIONVALIDITY_H_
