/**
 * RotateAtS.h
 * This class defines the rotate at s local planner which performs a
 * translation to the location "s" percent along the straight line
 * path, change all orientation dofs, then translate to the end
 *
 * Last Updated : 11/15/11
 * Update Author: Kasra Manavi
 */

#ifndef ROTATEATS_H_
#define ROTATEATS_H_

#include "StraightLine.h"

template <class CFG, class WEIGHT> class RotateAtS: public StraightLine<CFG, WEIGHT> {
 public:

  RotateAtS();
  RotateAtS(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~RotateAtS();

  virtual void PrintOptions(ostream& out_os);
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual bool IsConnected(Environment *env, Stat_Class& Stats,
    shared_ptr<DistanceMetricMethod > _dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
    LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision=true, bool _savePath=false, bool _saveFailedPath=false);

 protected:
  virtual bool IsConnectedOneWay(Environment *env, Stat_Class& Stats,
    shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
    LPOutput<CFG, WEIGHT>* lpOutput, double positionRes, double orientationRes,
    bool checkCollision=true, bool savePath=false, bool saveFailedPath=false);

  double s_value;
  vector<double> s_values;
  bool isSymmetric;
};

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::RotateAtS():
StraightLine<CFG, WEIGHT>() { 
  this->SetName("RotateAtS"); 
}

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::RotateAtS(XMLNodeReader& in_Node, MPProblem* in_pProblem): 
StraightLine<CFG, WEIGHT>(in_Node, in_pProblem) {
  this->SetName("RotateAtS");
  double nSValue = in_Node.numberXMLParameter(string("s"), true, 0.5, 0.0, 1.0, string("rotate at s value"));
  s_values.push_back(nSValue);
}

template <class CFG, class WEIGHT> RotateAtS<CFG, WEIGHT>::~RotateAtS() { }

template <class CFG, class WEIGHT> void
RotateAtS<CFG, WEIGHT>::PrintOptions(ostream& out_os) {
  out_os << "    " << this->GetName() << " ::"
         << " line_segment_length=" << this->lineSegmentLength
         << " binary_search=" << this->binarySearch
         << " vcMethod=" << this->vcMethod
         << " s_value=" << s_values[0] << endl;
}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
RotateAtS<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new RotateAtS<CFG, WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT> bool
RotateAtS<CFG,WEIGHT>::IsConnected(Environment *_env, Stat_Class& _stats,
shared_ptr< DistanceMetricMethod> _dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
bool _checkCollision, bool _savePath, bool _saveFailedPath) {  
  bool connected = false;
  connected = IsConnectedOneWay(_env,_stats,_dm,_c1,_c2,_col,_lpOutput,_posRes,_oriRes,_checkCollision,_savePath,_saveFailedPath);
  if (!connected && !isSymmetric) { // Try the other way
    connected = IsConnectedOneWay(_env,_stats,_dm,_c2,_c1,_col,_lpOutput,_posRes,_oriRes,_checkCollision,_savePath,_saveFailedPath);
    if (_savePath)
      reverse(_lpOutput->path.begin(), _lpOutput->path.end());
  }
  return connected;
}

template <class CFG, class WEIGHT> bool
RotateAtS<CFG,WEIGHT>::IsConnectedOneWay(Environment *_env, Stat_Class& _stats,
shared_ptr< DistanceMetricMethod > _dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->vcMethod);

  char RatS[50] = "Rotate_at_s";
  sprintf(RatS,"%s=%3.1f",RatS, s_values[0]);
  for(size_t i=1; i<s_values.size(); ++i) 
    sprintf(RatS,"%s,%3.1f",RatS, s_values[i]);
  _stats.IncLPAttempts( RatS );
  int cd_cntr= 0;
   
  if ( this->lineSegmentLength && 
       lineSegmentInCollision(_env,_stats,_dm,_c1,_c2,_lpOutput, cd_cntr, _posRes)) {
    _stats.IncLPCollDetCalls( RatS, cd_cntr );
    return false;
  }
  
  vector<Cfg *> sequence;
  _c1.GetMovingSequenceNodes(_c2, s_values, sequence);
  bool connected = true;
  
  // Check sequence nodes
  if ( _checkCollision ) {
    string Callee(this->GetName());
    string Method("-rotate_at_s::IsConnectedOneWay");
    CDInfo cdInfo;
    Callee = Callee + Method;
    for(size_t i=1; i<sequence.size()-1; ++i) { //_c1 and _c2 not double checked
      cd_cntr++;
      if ( !sequence[i]->InBoundingBox(_env) ||
           !vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, false, &Callee)) {
        if ( sequence[i]->InBoundingBox(_env) &&
             !vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, false, &Callee))
          _col = *sequence[i];
        connected = false;
        break;
      }
    }
  }
    
  // Check intermediate nodes  
  if ( connected ) {
    for ( size_t i=0; i<sequence.size()-1; ++i ) {
      if ( this->binarySearch ) 
        connected = IsConnectedSLBinary(_env, _stats, _dm, *sequence[i], *sequence[i+1], _col,
				            _lpOutput, cd_cntr, _posRes, _oriRes, _checkCollision, _savePath, _saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, _stats, _dm, *sequence[i], *sequence[i+1], _col,
				            _lpOutput, cd_cntr, _posRes, _oriRes, _checkCollision, _savePath, _saveFailedPath);

      if((_savePath || _saveFailedPath) && (i+1 != sequence.size()-1)) // Don't put _c2 on end
        _lpOutput->path.push_back(*sequence[i+1]);
      if(!connected) 
        break;
    }
  }

  if ( connected ) _stats.IncLPConnections( RatS );  
  _stats.IncLPCollDetCalls( RatS, cd_cntr );
  
  // Since we use vector<Cfg*>, we need to delete it
  for(size_t i=0; i<sequence.size(); ++i) 
    if (sequence[i] != NULL)
      delete sequence[i];
  
  return connected;
};
#endif
