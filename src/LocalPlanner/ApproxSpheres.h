#ifndef APPROXSPHERES_H
#define APPROXSPHERES_H

#include "LocalPlannerMethod.h"

template <class CFG, class WEIGHT>
class ApproxSpheres: public LocalPlannerMethod<CFG, WEIGHT> {
  public:

    /////////////////////////////////////////////////////////////
    //
    //
    //    Constructors and Destructor
    //
    //
    /////////////////////////////////////////////////////////////
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
      bool IsConnected(Environment *_env, StatClass& _stats,
          CollisionDetection *_cd,
          shared_ptr<DistanceMetricMethod >_dm, const CFG &_c1, const CFG &_c2, 
          LPOutput<CFG, WEIGHT>* _lpOutput,
          double _positionRes, double _orientationRes,
          bool _checkCollision=true, 
          bool _savePath=false, bool _saveFailedPath=false);

    //@}
    //////////////////////////////////////////////////////////////////
    //
    //
    //    Public Data
    //
    //
    //////////////////////////////////////////////////////////////////
  protected:
    int m_numAttempts;///< Number of times Cfg::ApproxCSpaceClearance will try to find clearance.
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
  return "ApproxSpheres";
}


template <class CFG, class WEIGHT>
void
ApproxSpheres<CFG, WEIGHT>::
SetDefault() {
  m_numAttempts = 3;
}



template <class CFG, class WEIGHT>
void
ApproxSpheres<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";
  _os << "\n\t" << m_numAttempts;

  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ApproxSpheres<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os  << GetName() << " ";
  _os << "m_numAttempts" << " " << m_numAttempts << " ";
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
IsConnected(Environment *_env, StatClass& _stats,
    CollisionDetection *_cd, shared_ptr<DistanceMetricMethod >_dm,
    const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, 
    bool _savePath, bool _saveFailedPath) {
  //clear lpOutput
  _lpOutput->path.clear();
  _lpOutput->edge.first.SetWeight(0);
  _lpOutput->edge.second.SetWeight(0);
  _lpOutput->savedEdge.clear();
  _stats.IncLPAttempts( this->GetNameAndLabel() );
  int cdCounter = 0;

  double dist, c1Clearance, c2Clearance;

  //calculate the distance between the two cfgs
  dist = _dm->Distance(_env,_c1,_c2);

  if (_c1.clearance != -1)
    c1Clearance = _c1.clearance;
  else
    c1Clearance = _c1.ApproxCSpaceClearance(_env,_stats,_cd,*this->cdInfo,
        _dm,m_numAttempts,false);
  if (_c2.clearance != -1)
    c2Clearance = _c2.clearance;
  else
    c2Clearance = _c2.ApproxCSpaceClearance(_env,_stats,_cd,*this->cdInfo,
        _dm,m_numAttempts,false);

  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  if (c1Clearance + c2Clearance >= dist) {
    _stats.IncLPConnections(this->GetNameAndLabel());
    return true;
  } else {
    return false;
  }  
};

#endif
