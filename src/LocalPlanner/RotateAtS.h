/**
 * RotateAtS.h
 * This class defines the rotate at s local planner which performs a
 * translation to the location "s" percent along the straight line
 * path, change all orientation dofs, then translate to the end
 *
 * Last Updated : 1/18/12
 * Update Author: Aditya Mahadevan
 */

#ifndef ROTATEATS_H_
#define ROTATEATS_H_

#include "StraightLine.h"

template <class CFG, class WEIGHT> class RotateAtS:
public StraightLine<CFG, WEIGHT> {
  public:

    RotateAtS();
    RotateAtS(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~RotateAtS();

    virtual void PrintOptions(ostream& _os);
    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

    virtual bool IsConnected(Environment *_env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod > _dm,
        const CFG &_c1, const CFG &_c2, CFG &_col, 
        LPOutput<CFG, WEIGHT>* _lpOutput,
        double _posRes, double _oriRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false);

    virtual vector<CFG> ReconstructPath(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, 
        const CFG& _c1, const CFG& _c2, const vector<CFG>& _intermediates, double _posRes, double _oriRes);
  protected:
    virtual bool IsConnectedOneWay(Environment *_env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod >_dm,
        const CFG &_c1, const CFG &_c2, CFG &_col, 
        LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false);

    double m_sValue;
    vector<double> m_sValues;
    bool m_isSymmetric;
};

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::RotateAtS():
StraightLine<CFG, WEIGHT>() { 
  this->SetName("RotateAtS"); 
}

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::RotateAtS(XMLNodeReader& _node, MPProblem* _problem): 
StraightLine<CFG, WEIGHT>(_node, _problem) {
  this->SetName("RotateAtS");
  double nSValue = _node.numberXMLParameter("s", true, 0.5, 0.0, 1.0, "rotate at s value");
  m_sValues.push_back(nSValue);
}

template <class CFG, class WEIGHT> RotateAtS<CFG, WEIGHT>::~RotateAtS() { }

template <class CFG, class WEIGHT> void
RotateAtS<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << " ::"
    << " m_binary_search=" << this->m_binarySearch
    << " m_vcMethod=" << this->m_vcMethod
    << " m_sValue=" << m_sValues[0] << endl;
}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
RotateAtS<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new RotateAtS<CFG, WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT> bool
RotateAtS<CFG,WEIGHT>::IsConnected(Environment *_env, StatClass& _stats,
    shared_ptr< DistanceMetricMethod> _dm,
    const CFG &_c1, const CFG &_c2,
    CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision,
    bool _savePath,
    bool _saveFailedPath) { 
  //clear _lpOutput
  _lpOutput->Clear();
  bool connected = false;
  connected = IsConnectedOneWay(_env,_stats,_dm,_c1,_c2,_col,_lpOutput,_posRes,_oriRes,_checkCollision,_savePath,_saveFailedPath);
  if (!connected && !m_isSymmetric) { // Try the other way
    connected = IsConnectedOneWay(_env,_stats,_dm,_c2,_c1,_col,_lpOutput,_posRes,_oriRes,_checkCollision,_savePath,_saveFailedPath);
    if (_savePath)
      reverse(_lpOutput->path.begin(), _lpOutput->path.end());
    reverse(_lpOutput->intermediates.begin(), _lpOutput->intermediates.end());
  }
  if(connected) 
    _lpOutput->AddIntermediatesToWeights();
  return connected;
}

template <class CFG, class WEIGHT> bool
RotateAtS<CFG,WEIGHT>::IsConnectedOneWay(Environment *_env, StatClass& _stats,
    shared_ptr< DistanceMetricMethod > _dm,
    const CFG &_c1, const CFG &_c2,
    CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision,
    bool _savePath,
    bool _saveFailedPath) {
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->m_vcMethod);

  char RatS[50];
  sprintf(RatS,"%s=%3.1f",this->GetNameAndLabel().c_str(), m_sValues[0]);
  for(size_t i=1; i<m_sValues.size(); ++i) 
    sprintf(RatS,"%s,%3.1f",RatS, m_sValues[i]);
  _stats.IncLPAttempts( RatS );
  int cdCounter= 0;

  vector<Cfg *> sequence;
  _c1.GetMovingSequenceNodes(_c2, m_sValues, sequence);
  bool connected = true;

  // Check sequence nodes
  if ( _checkCollision ) {
    string callee = this->GetName();
    string method = "-rotate_at_s::IsConnectedOneWay";
    CDInfo cdInfo;
    callee = callee + method;
    for(size_t i=1; i<sequence.size()-1; ++i) { //_c1 and _c2 not double checked
      cdCounter++;
      if ( !sequence[i]->InBoundingBox(_env) ||
          !vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, false, &callee)) {
        if ( sequence[i]->InBoundingBox(_env) &&
            !vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, false, &callee))
          _col = *sequence[i];
        connected = false;
        break;
      }
    }
  }

  // Check intermediate nodes  
  if ( connected ) {
    for ( size_t i=0; i<sequence.size()-1; ++i ) {
      if ( this->m_binarySearch ) 
        connected = IsConnectedSLBinary(_env, _stats, _dm, *sequence[i], 
            *sequence[i+1], _col, _lpOutput, cdCounter,
            _posRes, _oriRes, _checkCollision,
            _savePath, _saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, _stats, _dm, *sequence[i],
            *sequence[i+1], _col, _lpOutput, cdCounter,
            _posRes, _oriRes, _checkCollision,
            _savePath, _saveFailedPath);

      if((_savePath || _saveFailedPath) && (i+1 != sequence.size()-1)) // Don't put _c2 on end
        _lpOutput->path.push_back(*sequence[i+1]);
      if(!connected) 
        break;
    }
  }

  if(connected){
    _lpOutput->intermediates.push_back(*sequence[1]);
    _lpOutput->intermediates.push_back(*sequence[2]);
  }

  if (connected) 
    _stats.IncLPConnections( RatS );  
  _stats.IncLPCollDetCalls( RatS, cdCounter );

  // Since we use vector<Cfg*>, we need to delete it
  for(size_t i=0; i<sequence.size(); ++i) 
    if (sequence[i] != NULL)
      delete sequence[i];

  return connected;
};

template <class CFG, class WEIGHT>
vector<CFG> 
RotateAtS<CFG, WEIGHT>::ReconstructPath(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, 
        const CFG& _c1, const CFG& _c2, const vector<CFG>& _intermediates, double _posRes, double _oriRes){
  StatClass dummyStats;
  LPOutput<CFG, WEIGHT>* lpOutput = new LPOutput<CFG, WEIGHT>();
  CFG col;
  int dummyCntr;
  if(this->m_binarySearch)
    IsConnectedSLBinary(_env, dummyStats, _dm, _c1, _intermediates[0], col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
  else
    IsConnectedSLSequential(_env, dummyStats, _dm, _c1, _intermediates[0], col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
  lpOutput->path.push_back(_intermediates[0]);
  if(this->m_binarySearch)
    IsConnectedSLBinary(_env, dummyStats, _dm, _intermediates[0], _intermediates[1], col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
  else
    IsConnectedSLSequential(_env, dummyStats, _dm, _intermediates[0], _intermediates[1], col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
  lpOutput->path.push_back(_intermediates[1]);
  if(this->m_binarySearch)
    IsConnectedSLBinary(_env, dummyStats, _dm, _intermediates[1], _c2, col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
  else
    IsConnectedSLSequential(_env, dummyStats, _dm, _intermediates[1], _c2, col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
  vector<CFG> path = lpOutput->path;
  delete lpOutput;
  return path;
}
#endif
