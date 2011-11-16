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
  RotateAtS(cd_predefined _cdtype, double _s_val, vector<double> _s_vals, bool isSym);
  RotateAtS(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml = true);
  ///Destructor.  
  virtual ~RotateAtS();

  //@}
  //////////////////////
  // Access
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual void PrintOptions(ostream& out_os);

  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual bool IsConnected(Environment *env, Stat_Class& Stats,
         shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
         CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput,
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
         shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
         CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision=true, 
         bool savePath=false, bool saveFailedPath=false);

  double s_value;
  vector<double> s_values;
  bool isSymmetric;
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
  this->SetName("rotateAtS");
}

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::
RotateAtS(cd_predefined _cdtype, double _s_val, vector<double> _s_vals, bool isSym) : StraightLine<CFG, WEIGHT>(_cdtype), s_value(_s_val), s_values(_s_vals), isSymmetric(isSym) {
  SetDefault();
  this->SetName("rotateAtS");
};

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::
RotateAtS(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml) : StraightLine<CFG, WEIGHT>(_cdtype, in_Node, in_pProblem, false) 
{
  this->SetName("rotateAtS");
  this->cdtype = _cdtype;
  double nSValue = in_Node.numberXMLParameter(string("s"), true, 0.5, 0.0, 1.0, string("rotate at s value"));
  s_values.push_back(nSValue);
  if(warnUnrequestedXml)
    in_Node.warnUnrequestedAttributes();
}

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::
~RotateAtS() {
}



template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
PrintOptions(ostream& out_os) {
  out_os << "    " << this->GetName() << "::  ";
  out_os << "line segment length = " << " " << this->lineSegmentLength << " ";
  out_os << "binary search = " << " " << this->binarySearch << " ";
  out_os << "vcMethod = " << " " << this->vcMethod << " ";
  out_os << "s_value = " << s_values[0];
  out_os << endl;
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
  
  _os << "\n" << this->GetName() << " ";
  _os << "\n\t" << this->lineSegmentLength;
  _os << "\n\t" << this->binarySearch;
  _os << "\n\t" << s_values[0];
 
  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os << this->GetName() << " ";
  _os << "ineSegmentLength" << " " << this->lineSegmentLength << " ";
  _os << "binarySearch" << " " << this->binarySearch << " ";
  for(size_t i=0; i<s_values.size(); i++){
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
     shared_ptr< DistanceMetricMethod>dm,
      const CFG &_c1, const CFG &_c2, CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput,
      double positionRes, double orientationRes,
      bool checkCollision, 
      bool savePath, bool saveFailedPath) {  
  bool connected = false;
  connected = IsConnectedOneWay(_env, Stats, dm, _c1, _c2, _col, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
  if (!connected && !isSymmetric) { //try the other way
    connected = IsConnectedOneWay(_env, Stats, dm, _c2, _c1, _col, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    if (savePath)
      reverse(lpOutput->path.begin(), lpOutput->path.end());
  }
  return connected;

}

template <class CFG, class WEIGHT>
bool
RotateAtS<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, Stat_Class& Stats,shared_ptr< DistanceMetricMethod >dm,
      const CFG &_c1, const CFG &_c2, CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput,
      double positionRes, double orientationRes,
      bool checkCollision, 
      bool savePath, bool saveFailedPath) {
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->vcMethod);
  //CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();

  char RatS[50] = "Rotate_at_s";
  sprintf(RatS,"%s=%3.1f",RatS, s_values[0]);
  for(size_t i=1; i<s_values.size(); ++i)
    sprintf(RatS,"%s,%3.1f",RatS, s_values[i]);
  Stats.IncLPAttempts( RatS );
  int cd_cntr= 0;
   
  if(this->lineSegmentLength && lineSegmentInCollision(_env, Stats, dm, _c1, _c2, 
								  lpOutput, cd_cntr, positionRes)) {
    Stats.IncLPCollDetCalls( RatS, cd_cntr );
    return false;
  }
  
  vector<Cfg *> sequence;
  _c1.GetMovingSequenceNodes(_c2, s_values, sequence);

  bool connected = true;
  
  //check sequence nodes
  if(checkCollision) {
    string Callee(this->GetName());
    string Method("-rotate_at_s::IsConnectedOneWay");
    Callee = Callee + Method;
    for(size_t i=1; i<sequence.size()-1; ++i) { //_c1 and _c2 not double checked
      cd_cntr++;
      if(!sequence[i]->InBoundingBox(_env) ||
         //sequence[i]->isCollision(_env, Stats, cd, *this->cdInfo, true, &(Callee))
         !vc->IsValid(vcm, *sequence[i], _env, Stats, *this->cdInfo,
         false, &Callee)
        ) {
         if(sequence[i]->InBoundingBox(_env) &&
            //sequence[i]->isCollision(_env, Stats, cd, *this->cdInfo, true, &(Callee)))
            !vc->IsValid(vcm, *sequence[i], _env, Stats, *this->cdInfo, false,
            &Callee))
            _col = *sequence[i];
        connected = false;
        break;
      }
    }
  }
    
  //check intermediate nodes  
  if(connected) {
    for(size_t i=0; i<sequence.size()-1; ++i) {
      if(this->binarySearch) 
        connected = IsConnectedSLBinary(_env, Stats, dm, *sequence[i], *sequence[i+1], _col,
				        lpOutput, cd_cntr, positionRes, orientationRes, 
				        checkCollision, savePath, saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, Stats, dm, *sequence[i], *sequence[i+1], _col,
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
  for(size_t i=0; i<sequence.size(); ++i) 
    if (sequence[i] != NULL)
      delete sequence[i];
  
  return connected;
};
#endif
