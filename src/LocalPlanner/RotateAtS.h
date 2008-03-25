#ifndef RotateAtS_h
#define RotateAtS_h

#include "StraightLine.h"

template <class CFG, class WEIGHT>
class RotateAtS: public StraightLine<CFG, WEIGHT> {

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
  RotateAtS(cd_predefined _cdtype);
  RotateAtS(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ///Destructor.  
  virtual ~RotateAtS();

  //@}
  //////////////////////
  // Access
  virtual char* GetName() const;
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual void PrintOptions(ostream& out_os);

  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual bool IsConnected(Environment *env, Stat_Class& Stats,
         CollisionDetection *,
         DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
         LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision=true, 
         bool savePath=false, bool saveFailedPath=false);


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

 protected:
  virtual 
    bool IsConnectedOneWay(Environment *env, Stat_Class& Stats,
         CollisionDetection *cd,
         DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
         LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision=true, 
         bool savePath=false, bool saveFailedPath=false);

  //@{
    double s_value;
    vector<double> s_values;
    bool isSymmetric;
  //@}
};

/////////////////////////////////////////////////////////////////////
//
//  definitions for class RotateAtS declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::
RotateAtS(cd_predefined _cdtype) : StraightLine<CFG, WEIGHT>(_cdtype) {
  SetDefault(); 
}

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::
RotateAtS(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem) : StraightLine<CFG, WEIGHT>(_cdtype, in_Node,in_pProblem) {
  this->cdtype = _cdtype;
  LOG_DEBUG_MSG("RotateAtS::RotateAtS()");

  
  double  nSValue = in_Node.numberXMLParameter(string("s"),true,double(0.5),
                                            double(0.0),double(1.0),string("rotate at s value"));
  s_values.push_back(nSValue);
  
  LOG_DEBUG_MSG("~RotateAtS::RotateAtS()");
}

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::
~RotateAtS() {
}



template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
PrintOptions(ostream& out_os) {
  out_os << "    " << GetName() << "::  ";
  out_os << "line segment length = " << " " << this->lineSegmentLength << " ";
  out_os << "binary search = " << " " << this->binarySearch << " ";
  out_os << "s_value = " << s_values[0];
  out_os << endl;
}



template <class CFG, class WEIGHT>
char*
RotateAtS<CFG, WEIGHT>::
GetName() const {
  return "rotate_at_s";
}

template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
SetDefault() {
  StraightLine<CFG, WEIGHT>::SetDefault();
  s_values.clear();
  //s_values.push_back(sValue); 
  isSymmetric = true;
}



template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t" << this->lineSegmentLength;
  _os << "\n\t" << this->binarySearch;
  _os << "\n\t" << s_values[0];
 
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os << GetName() << " ";
  _os << "ineSegmentLength" << " " << this->lineSegmentLength << " ";
  _os << "binarySearch" << " " << this->binarySearch << " ";
  for(int i=0; i<s_values.size(); i++){
   _os << "s" << " " << s_values[i] << " ";
   }
 _os << endl;
}


template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
RotateAtS<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new RotateAtS<CFG, WEIGHT>(*this);
  return _copy;

}

template <class CFG, class WEIGHT>
bool
RotateAtS<CFG,WEIGHT>::
IsConnected(Environment *_env, Stat_Class& Stats,
      CollisionDetection *cd, DistanceMetric *dm,
      const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
      double positionRes, double orientationRes,
      bool checkCollision, 
      bool savePath, bool saveFailedPath) {  
  bool connected = false;
  connected = IsConnectedOneWay(_env, Stats, cd, dm, _c1, _c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
  if (!connected && !isSymmetric) { //try the other way
    connected = IsConnectedOneWay(_env, Stats, cd, dm, _c2, _c1, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    if (savePath)
      reverse(lpOutput->path.begin(), lpOutput->path.end());
  }
  return connected;

}

template <class CFG, class WEIGHT>
bool
RotateAtS<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, Stat_Class& Stats, CollisionDetection *cd, DistanceMetric *dm,
      const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
      double positionRes, double orientationRes,
      bool checkCollision, 
      bool savePath, bool saveFailedPath) {  
  char RatS[50] = "Rotate_at_s";
  sprintf(RatS,"%s=%3.1f",RatS, s_values[0]);
  for(int i=1; i<s_values.size(); ++i)
    sprintf(RatS,"%s,%3.1f",RatS, s_values[i]);
  Stats.IncLPAttempts( RatS );
  int cd_cntr= 0;
   
  if(this->lineSegmentLength && lineSegmentInCollision(_env, Stats, cd, dm, _c1, _c2, 
								  lpOutput, cd_cntr, positionRes)) {
    Stats.IncLPCollDetCalls( RatS, cd_cntr );
    return false;
  }
  
  vector<Cfg *> sequence;
  _c1.GetMovingSequenceNodes(_c2, s_values, sequence);

  bool connected = true;
  
  //check sequence nodes
  if(checkCollision) {
    std::string Callee(GetName());
    std::string Method("-rotate_at_s::IsConnectedOneWay");
    Callee = Callee + Method;
    for(int i=1; i<sequence.size()-1; ++i) { //_c1 and _c2 not double checked
      cd_cntr++;
      if(!sequence[i]->InBoundingBox(_env) ||
         sequence[i]->isCollision(_env, Stats, cd, *this->cdInfo, true, &(Callee))) {
        connected = false;
        break;
      }
    }
  }
    
  //check intermediate nodes  
  if(connected) {
    for(int i=0; i<sequence.size()-1; ++i) {
      if(this->binarySearch) 
        connected = IsConnectedSLBinary(_env, Stats, cd, dm, *sequence[i], *sequence[i+1], 
				        lpOutput, cd_cntr, positionRes, orientationRes, 
				        checkCollision, savePath, saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, Stats, cd, dm, *sequence[i], *sequence[i+1], 
				            lpOutput, cd_cntr, positionRes, orientationRes, 
				            checkCollision, savePath, saveFailedPath);

      if((savePath || saveFailedPath) && (i+1 != sequence.size()-1)) //don't put _c2 on end
        lpOutput->path.push_back(*sequence[i+1]);

      if(!connected) 
        break;
    }
  }

  if(connected)
    Stats.IncLPConnections( RatS );  
  Stats.IncLPCollDetCalls( RatS, cd_cntr );
  
  // Since we use vector<Cfg*>, we need to delete it
  for(int i=0; i<sequence.size(); ++i) 
    if (sequence[i] != NULL)
      delete sequence[i];
  
  return connected;
};
#endif
