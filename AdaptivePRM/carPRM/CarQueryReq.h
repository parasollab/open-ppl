///////////////////////////////////////
//
// CarQueryReq.h
//
// derived classs of BasicQueryReq and
// DefaultQueryReqFactory to implement
// query requirements specific for
// car-like robots
//
///////////////////////////////////////
#ifndef _CarQueryReq_h
#define _CarQueryReq_h

#include "QueryRequirements.h"


///////////////////////////////////////////////
//
// CarQueryReqFactory : DefaultQueryReqFactory
//
///////////////////////////////////////////////
class CarQueryReq;
class CarQueryReqFactory : public DefaultQueryReqFactory {
  friend class CarQueryReq;

protected:
  virtual bool Create(IQueryReq ** ppIQueryReq) const;
};


///////////////////////////////////////////////
//
// CarQueryReq : public BasicQueryReq
//
///////////////////////////////////////////////
class CarQueryReq : public BasicQueryReq {

public:
  CarQueryReq();

  virtual void init(char* filename);
  
  virtual void Print(ostream& os);

  bool CheckTurningRadius();
  double GetTurningRadius();

protected:
   // flags:
   bool checkTurningRadius;

   // values:
   double turningRadius;
};

#endif
