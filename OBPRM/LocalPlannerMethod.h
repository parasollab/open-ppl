#ifndef LocalPlannerMethod_h
#define LocalPlannerMethod_h

#include "Parameters.h"

#include "DistanceMetrics.h"
#include "Roadmap.h"
#include "Environment.h"
#include "MultiBody.h"
#include "Input.h"


template <class CFG, class WEIGHT> struct LPOutput;

template <class CFG, class WEIGHT>
class LocalPlannerMethod { 
 public:

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{

  ///Default Constructor.
  LocalPlannerMethod();
  ///Destructor.	
  virtual ~LocalPlannerMethod();

  //@}

  virtual bool operator == (const LocalPlannerMethod<CFG,WEIGHT> &other) const;

  virtual bool SameParameters(const LocalPlannerMethod<CFG,WEIGHT> &other) const = 0;

  //////////////////////
  // Access
  virtual char* GetName() const = 0;
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy() = 0;


  /**Determine whether two cfgs are connected according to method type, abstract.
   *@param LPOutput.
   */
  virtual bool IsConnected(Environment* _env, CollisionDetection* cd, 
			   DistanceMetric *, const CFG &_c1, const CFG &_c2, 
			   LPOutput<CFG, WEIGHT>* lpOutput,
			   double positionRes, double orientationRes,
			   bool checkCollision=true, 
			   bool savePath=false, bool saveFailedPath=false) = 0;


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
 protected:
  //@}

  int lp_id;

 public:
  void SetID(int new_id);
  int GetID() const;
  
  SID* dmsetid;
  SID* cdsetid;
  CDInfo* cdInfo;

};

template <class CFG, class WEIGHT>
bool 
LocalPlannerMethod<CFG, WEIGHT>::operator == (const LocalPlannerMethod<CFG, WEIGHT> &other) const {
  bool result = false;
  if ( strcmp(GetName(), other.GetName()) ) //they are different
    result = false;
  else //they have the same name (they are of the same type)
    result = SameParameters(other); 
  return result;
}

/////////////////////////////////////////////////////////////////////
//
//  definitions for class LocalPlannerMethod declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>::
LocalPlannerMethod() {
  lp_id = -1;
  SetDefault();
}


template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>::
~LocalPlannerMethod() {
}

template <class CFG, class WEIGHT>
void LocalPlannerMethod<CFG, WEIGHT>::
SetDefault() {
}

template <class CFG, class WEIGHT>
void LocalPlannerMethod<CFG, WEIGHT>::
SetID(int new_id) {
  lp_id = new_id;
}

template <class CFG, class WEIGHT>
int LocalPlannerMethod<CFG, WEIGHT>::
GetID() const {
  return lp_id;
}

#endif
