#ifndef ApproxSpheres_h
#define ApproxSpheres_h

#include "LocalPlannerMethod.h"

template <class CFG, class WEIGHT>
class ApproxSpheres: public LocalPlannerMethod<CFG, WEIGHT> {
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
  ApproxSpheres();
  ///Destructor.	
  virtual ~ApproxSpheres();

  //@}
  
  //////////////////////
  // Access
  virtual char* GetName() const;
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();
      /**Roughly check if two Cfgs could be connected using clearance.
        *Algorithm is given here:
        *   -# set clearance1 as clearance for _c1
        *   -# set clearance2 as clearance for _c2
        *   -# set dist as distance from _c1 to c2
        *   -# if clearance1+clearance2 > dist
        *       -# connected
        *   -# else
        *       -# not connected
        *
        *@see Cfg::ApproxCSpaceClearance and Cfg::Clearance
        */
  virtual 
    bool IsConnected(Environment *_env, Stat_Class& Stats,
		     CollisionDetection *cd,
		     shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
		     LPOutput<CFG, WEIGHT>* lpOutput,
		     double positionRes, double orientationRes,
		     bool checkCollision=true, 
		     bool savePath=false, bool saveFailedPath=false);

  //@}
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
 protected:
  int n;///< Number of times Cfg::ApproxCSpaceClearance will try to find clearance.
};

/////////////////////////////////////////////////////////////////////
//
//  definitions for class StraightLine declarations
//
/////////////////////////////////////////////////////////////////////


template <class CFG, class WEIGHT>
ApproxSpheres<CFG, WEIGHT>::
ApproxSpheres() : LocalPlannerMethod<CFG, WEIGHT>(){
  SetDefault();
}


template <class CFG, class WEIGHT>
ApproxSpheres<CFG, WEIGHT>::
~ApproxSpheres() {
}


template <class CFG, class WEIGHT>
char*
ApproxSpheres<CFG, WEIGHT>::
GetName() const {
  return "approx_spheres";
}


template <class CFG, class WEIGHT>
void
ApproxSpheres<CFG, WEIGHT>::
SetDefault() {
  n = 3;
}



template <class CFG, class WEIGHT>
void
ApproxSpheres<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t" << n;
 
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ApproxSpheres<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os  << GetName() << " ";
  _os << "n" << " " << n << " ";
  _os << endl;
}


template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
ApproxSpheres<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new ApproxSpheres<CFG, WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
bool
ApproxSpheres<CFG,WEIGHT>::
IsConnected(Environment *_env, Stat_Class& Stats,
	    CollisionDetection *cd, shared_ptr<DistanceMetricMethod >dm,
	    const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
	    double positionRes, double orientationRes,
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {
  Stats.IncLPAttempts( "ApproxSpheres" );
  int cd_cntr = 0;

  double dist, c1_clearance, c2_clearance;
  
  //calculate the distance between the two cfgs
  dist = dm->Distance(_env,_c1,_c2);
  
  if (_c1.clearance != -1)
    c1_clearance = _c1.clearance;
  else
    c1_clearance = _c1.ApproxCSpaceClearance(_env,Stats,cd,*this->cdInfo,
					     dm,n,false);
  if (_c2.clearance != -1)
    c2_clearance = _c2.clearance;
  else
    c2_clearance = _c2.ApproxCSpaceClearance(_env,Stats,cd,*this->cdInfo,
					     dm,n,false);
  
  Stats.IncLPCollDetCalls("ApproxSpheres", cd_cntr);
  if (c1_clearance + c2_clearance >= dist) {
    Stats.IncLPConnections("ApproxSpheres");
    return true;
  } else {
    return false;
  }  
};

#endif
