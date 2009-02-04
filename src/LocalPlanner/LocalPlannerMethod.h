#ifndef LocalPlannerMethod_h
#define LocalPlannerMethod_h



#include "DistanceMetrics.h"
#include "Roadmap.h"
#include "Environment.h"
#include "MultiBody.h"
#include "Stat_Class.h"
#include "util.h"
#include "ValidityChecker.hpp"

template <class CFG, class WEIGHT> struct LPOutput;

template <class CFG, class WEIGHT>
class LocalPlannerMethod : public MPBaseObject{ 
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
  LocalPlannerMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  
  ///Destructor.  
  virtual ~LocalPlannerMethod();

  //@}

//  virtual bool operator == (const LocalPlannerMethod<CFG,WEIGHT> &other) const;

  //////////////////////
  // Access
  virtual char* GetName() const = 0;
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  ///Used in new MPProblem framework. \todo remove the "{ }" later
  virtual void PrintOptions(ostream& out_os) { };
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy() = 0;


  /**Determine whether two cfgs are connected according to method type, abstract.
   *@param LPOutput.
   */
  virtual bool IsConnected(Environment* _env, Stat_Class& Stats, 
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
  
  CDInfo* cdInfo;

  

};


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
LocalPlannerMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    MPBaseObject(in_Node,in_pProblem) {
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
