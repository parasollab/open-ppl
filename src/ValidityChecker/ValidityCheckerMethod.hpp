#ifndef _VALIDITY_CHECKER_METHOD_HPP_
#define _VALIDITY_CHECKER_METHOD_HPP_

#include <string>
#include "LabeledObject.h"
#include "CfgTypes.h"


class ValidityCheckerMethod : public LabeledObject, public MPBaseObject 
{
public:
  ValidityCheckerMethod() { }
  ValidityCheckerMethod(std::string in_strLabel, MPProblem* in_pProblem) : LabeledObject(in_strLabel), 
									   MPBaseObject(in_pProblem) { }
  virtual ~ValidityCheckerMethod() { }
  
  virtual bool IsValid(CfgType& _cfg, Environment* env, Stat_Class& Stats, 
		       CDInfo& _cdInfo, bool enablePenetration, std::string *pCallName) = 0; 
};

#endif //end #ifndef _VALIDITY_CHECKER_METHOD_HPP_
