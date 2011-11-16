#ifndef _VALIDITY_CHECKER_METHOD_HPP_
#define _VALIDITY_CHECKER_METHOD_HPP_

#include <string>
#include "MPUtils.h"
#include "CfgTypes.h"


class ValidityCheckerMethod : public MPBaseObject 
{
public:
  ValidityCheckerMethod() { }
  ValidityCheckerMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) : MPBaseObject(in_Node, in_pProblem) { }
  virtual ~ValidityCheckerMethod() { }
  
  virtual bool IsValid(Cfg& _cfg, Environment* env, Stat_Class& Stats, 
		       CDInfo& _cdInfo, bool enablePenetration, std::string *pCallName) = 0; 
  virtual bool isInsideObstacle(const Cfg& cfg, Environment* env, CDInfo& _cdInfo) {
    cerr << "error: isInsideObstacle() not defined." << endl;
		exit(-1);
  };
	
};
#endif // End #ifndef _VALIDITY_CHECKER_METHOD_HPP_
