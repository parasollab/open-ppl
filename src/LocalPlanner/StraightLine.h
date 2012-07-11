/**
 * StraightLine.h
 *
 * This class performs straight line local planning which is used by a variety of computations,
 * including other local planners.
 */

#ifndef STRAIGHTLINE_H_
#define STRAIGHTLINE_H_

#include <deque>
#include "LocalPlannerMethod.h"
#include "ValidityChecker.hpp"
#include "Cfg_reach_cc.h"

template <class CFG, class WEIGHT>
class StraightLine: public LocalPlannerMethod<CFG, WEIGHT> {
  public:

    //////////////////////////////
    //Constructors and Destructor 
    //////////////////////////////
    StraightLine(string _vcMethod = "", bool _search = false);
    StraightLine(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~StraightLine();

    virtual void PrintOptions(ostream& _os);

    /**
     * Check if two Cfgs could be connected by straight line.
     */
    // Wrapper function to call appropriate impl IsConnectedFunc based on CFG type
    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod>_dm,
        const CFG& _c1, const CFG& _c2, CFG& _col, 
        LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, 
        bool _savePath = false, bool _saveFailedPath = false) {
      //clear lpOutput
      _lpOutput->Clear();
      bool connected = IsConnectedFunc<CFG>(_env, _stats, _dm, _c1, _c2, _col,
          _lpOutput, _positionRes, _orientationRes,
          _checkCollision, _savePath, _saveFailedPath);
      if(connected)
        _lpOutput->SetLPLabel(this->GetLabel());
      return connected;
    }

    // Default for non closed chains
    template <typename Enable>
      bool IsConnectedFunc(Environment* _env, StatClass& _stats,
          shared_ptr<DistanceMetricMethod>_dm,
          const CFG& _c1, const CFG& _c2, CFG& _col,
          LPOutput<CFG, WEIGHT>* _lpOutput,
          double _positionRes, double _orientationRes,
          bool _checkCollision = true, 
          bool _savePath = false, bool _saveFailedPath = false,
          typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy = 0
          );

    // Specialization for closed chains
    template <typename Enable>
      bool IsConnectedFunc(Environment* _env, StatClass& _stats,
          shared_ptr<DistanceMetricMethod>_dm, 
          const CFG& _c1, const CFG& _c2, CFG& _col,
          LPOutput<CFG, WEIGHT>* _lpOutput,
          double _positionRes, double _orientationRes,
          bool _checkCollision = true, 
          bool _savePath = false, bool _saveFailedPath = false,
          typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy = 0
          );

    string m_vcMethod;

  protected:
    /**
     * Check if two Cfgs could be connected by straight line.
     * This method implements straight line connection local planner
     * by checking collision of each Cfg along the line.
     * If the is any Cfg causes Robot collides with any obstacle,
     * false will be returned.
     */
    virtual bool 
      IsConnectedSLSequential(Environment* _env, StatClass& _stats,
          shared_ptr<DistanceMetricMethod> _dm, 
          const CFG& _c1, const CFG& _c2, CFG& _col,
          LPOutput<CFG,WEIGHT>* _lpOutput, int& _cdCounter,
          double _positionRes, double _orientationRes,
          bool _checkCollision = true, 
          bool _savePath = false, bool _saveFailedPath = false);

    /**
     * Check if two Cfgs could be connected by straight line 
     * This method uses binary search to check clearances of Cfgs between _c1 and 
     * _c2. If any Cfg with clearance less than 0.001 was found, false will be returned.
     */
    virtual bool 
      IsConnectedSLBinary(Environment* _env, StatClass& _stats, 
          shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, 
          CFG& _col, LPOutput<CFG,WEIGHT>* _lpOutput, int& _cdCounter,
          double _positionRes, double _orientationRes,  
          bool _checkCollision = true, 
          bool _savePath = false, bool _saveFailedPath = false);

    int m_binarySearch;
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class StraightLine declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
StraightLine<CFG, WEIGHT>::StraightLine(string _vcMethod, bool _search) : 
  LocalPlannerMethod<CFG, WEIGHT>(), m_vcMethod(_vcMethod), m_binarySearch(_search) {
    this->SetName("StraightLine");
  }

template <class CFG, class WEIGHT>
StraightLine<CFG, WEIGHT>::StraightLine(XMLNodeReader& _node, MPProblem* _problem) :
  LocalPlannerMethod<CFG, WEIGHT>(_node,_problem) {
    this->SetName("StraightLine");
    m_binarySearch = _node.numberXMLParameter("binary_search", false, 0, 0, 1, "binary search"); 
    m_vcMethod = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
  }

template <class CFG, class WEIGHT> 
StraightLine<CFG, WEIGHT>::~StraightLine() { }

template <class CFG, class WEIGHT> 
void
StraightLine<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << "::  ";
  _os << "binary search = " << " " << m_binarySearch << " ";
  _os << "vcMethod = " << " " << m_vcMethod << " ";
  _os << endl;
}

//// default implementation for non closed chains
template <class CFG, class WEIGHT>
template <typename Enable>
bool 
StraightLine<CFG, WEIGHT>::IsConnectedFunc(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod >_dm, const CFG& _c1, const CFG& _c2, 
    CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, 
    bool _savePath, bool _saveFailedPath,
    typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy) { 

  _stats.IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0; 

  bool connected;
  if(m_binarySearch) 
    connected = IsConnectedSLBinary(_env, _stats, _dm,
        _c1, _c2, _col, _lpOutput, 
        cdCounter, _positionRes, _orientationRes, 
        _checkCollision, _savePath, _saveFailedPath);
  else
    connected = IsConnectedSLSequential(_env, _stats, _dm,
        _c1, _c2, _col, _lpOutput, 
        cdCounter, _positionRes, _orientationRes, 
        _checkCollision, _savePath, _saveFailedPath);
  if(connected)
    _stats.IncLPConnections(this->GetNameAndLabel());

  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
}


//// specialized implementation for closed chains
template <class CFG, class WEIGHT>
template <typename Enable>
bool 
StraightLine<CFG, WEIGHT>::
IsConnectedFunc(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod > _dm, const CFG& _c1, const CFG& _c2, 
    CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, 
    bool _savePath, bool _saveFailedPath,
    typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy) {

  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vcMethod);
  string callee = this->GetName() + "::IsConnectedSLSequential";
  CDInfo cdInfo;

  _stats.IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0; 

  bool connected;
  if(CFG::OrientationsDifferent(_c1, _c2)) {
    CFG intermediate;
    bool success = intermediate.GetIntermediate(_c1, _c2); 
    if(_checkCollision){
      cdCounter++;
      if(!intermediate.InBoundary(_env) || 
          !vc->IsValid(vcm, intermediate, _env, _stats, cdInfo, true, &callee)
        ) {
        if(intermediate.InBoundary(_env))
          _col = intermediate;
        return false;
      }
    }
    if(!success)
      return false;

    if(m_binarySearch) {
      connected = (IsConnectedSLBinary(_env, _stats, _dm, 
            _c1, intermediate, 
            _col, _lpOutput, cdCounter, 
            _positionRes, _orientationRes, 
            _checkCollision, _savePath, _saveFailedPath) 
          &&
          IsConnectedSLBinary(_env, _stats, _dm,
            intermediate, _c2,
            _col, _lpOutput, cdCounter, 
            _positionRes, _orientationRes, 
            _checkCollision, _savePath, _saveFailedPath)
          );
    } else {
      connected = (IsConnectedSLSequential(_env, _stats, _dm, 
            _c1, intermediate, 
            _col, _lpOutput, cdCounter, 
            _positionRes, _orientationRes, 
            _checkCollision, _savePath, _saveFailedPath) 
          &&
          IsConnectedSLSequential(_env, _stats, _dm, 
            intermediate, _c2, 
            _col, _lpOutput, cdCounter, 
            _positionRes, _orientationRes, 
            _checkCollision, _savePath, _saveFailedPath)
          );

    }
  } else {
    if(m_binarySearch) {
      connected = IsConnectedSLBinary(_env, _stats, _dm,
          _c1, _c2,
          _col, _lpOutput, cdCounter, 
          _positionRes, _orientationRes, 
          _checkCollision, _savePath, _saveFailedPath);
    } else {
      connected = IsConnectedSLSequential(_env, _stats, _dm,
          _c1, _c2,
          _col, _lpOutput, cdCounter, 
          _positionRes, _orientationRes, 
          _checkCollision, _savePath, _saveFailedPath);
    }
  }
  if(connected)
    _stats.IncLPConnections(this->GetNameAndLabel());

  _stats.IncLPCollDetCalls( this->GetNameAndLabel(), cdCounter );
  return connected;
}


template <class CFG, class WEIGHT>
bool
StraightLine<CFG, WEIGHT>::
IsConnectedSLSequential(Environment* _env, StatClass& _stats,
    shared_ptr< DistanceMetricMethod > _dm, 
    const CFG& _c1, const CFG& _c2, CFG& _col,
    LPOutput<CFG,WEIGHT>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, 
    bool _savePath, bool _saveFailedPath) {
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vcMethod);
  int nTicks;
  CFG tick;
  tick = _c1; 
  CFG incr;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  incr.FindIncrement(_c1,_c2,&nTicks,_positionRes,_orientationRes, _env->GetRdRes());
#else
  incr.FindIncrement(_c1,_c2,&nTicks,_positionRes,_orientationRes);
#endif
  string callee = this->GetName() + "::IsConnectedSLSequential";
  CDInfo cdInfo;


  int nIter = 0;
  for(int i = 1; i < nTicks; i++){ //don't need to check the ends, _c1 and _c2
    tick.Increment(incr);
    _cdCounter++;
    if(_checkCollision){
      if(!tick.InBoundary(_env) || 
          !vc->IsValid(vcm, tick, _env, _stats, cdInfo, true, &callee)
        ) {
        if(tick.InBoundary(_env))   
          _col = tick;
        CFG negIncr;
        negIncr = incr; 
        negIncr.negative(incr);
        tick.Increment(negIncr);
        _lpOutput->edge.first.SetWeight(_lpOutput->edge.first.GetWeight() + nIter);
        _lpOutput->edge.second.SetWeight(_lpOutput->edge.second.GetWeight() + nIter);
        pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > tmp;
        tmp.first.first = _c1;
        tmp.first.second = tick;
        tmp.second.first = _lpOutput->edge.first;
        tmp.second.second = _lpOutput->edge.second;
        _lpOutput->savedEdge.push_back(tmp);
        return false;
      }
    }
    if(_savePath || _saveFailedPath){
      _lpOutput->path.push_back(tick);
    }
    nIter++;
  }
  _lpOutput->edge.first.SetWeight(_lpOutput->edge.first.GetWeight() + nIter);
  _lpOutput->edge.second.SetWeight(_lpOutput->edge.second.GetWeight() + nIter);

  return true;
};


template <class CFG, class WEIGHT>
bool 
StraightLine<CFG, WEIGHT>::
IsConnectedSLBinary(Environment* _env, StatClass& _stats, 
    shared_ptr<DistanceMetricMethod>_dm, const CFG& _c1, const CFG& _c2, 
    CFG& _col, LPOutput<CFG,WEIGHT>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,  
    bool _checkCollision, 
    bool _savePath, bool _saveFailedPath) {
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vcMethod);

  if(!_checkCollision)
    return IsConnectedSLSequential(_env, _stats, _dm, _c1, _c2,
        _col, _lpOutput, _cdCounter, 
        _positionRes, _orientationRes,
        _checkCollision, _savePath, _saveFailedPath);

  string callee = this->GetName() + "::IsConnectedSLBinary";
  CDInfo cdInfo;

  int nTicks;
  CFG incr;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  incr.FindIncrement(_c1,_c2,&nTicks,_positionRes,_orientationRes, _env->GetRdRes());
#else
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);
#endif

  deque<pair<int,int> > Q;
  Q.push_back(make_pair(0, nTicks));

  while(!Q.empty()) {
    pair<int,int> p = Q.front();
    int i = p.first;
    int j = p.second;
    Q.pop_front();

    int mid = i + (int)floor(((double)(j-i))/2);
    CFG midCfg = _c1;
    /*
    // this produces the exact same intermediate cfgs as the sequential 
    // version (no roundoff errors), but it is slow
    for(int z=0; z<mid; ++z)
    midCfg.Increment(incr);
    */
    // this produces almost the same intermediate cfgs as the sequential
    // version, but there may be some roundoff errors; it is much faster
    // than the above solution; the error should be much smaller than the
    // resolution so it should not be a problem
    midCfg.multiply(incr, mid);
    midCfg.add(_c1, midCfg);

    _cdCounter++;
    if(!midCfg.InBoundary(_env) ||
        !vc->IsValid(vcm, midCfg, _env, _stats, cdInfo, true, &callee) ) {
      if(midCfg.InBoundary(_env))
        _col=midCfg;
      return false;
    } else {
      if(i+1 != mid) 
        Q.push_back(make_pair(i, mid));
      if(mid+1 != j) 
        Q.push_back(make_pair(mid, j));      
    }
  }

  if(_savePath || _saveFailedPath) {
    CFG tick = _c1;
    for(int n=1; n<nTicks; ++n) {
      tick.Increment(incr);
      _lpOutput->path.push_back(tick);
    }
  }
  _lpOutput->edge.first.SetWeight(_lpOutput->edge.first.GetWeight() + nTicks-1);
  _lpOutput->edge.second.SetWeight(_lpOutput->edge.second.GetWeight() + nTicks-1);

  return true;
}

#endif
