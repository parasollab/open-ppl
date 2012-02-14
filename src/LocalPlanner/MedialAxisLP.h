/**
 * MedialAxisLP.h
 * This class defines the medial axis local planner which performs a
 * push of the pathway connecting 2 medial axis configurations along
 * the medial axis.
 *
 * Last Updated : 11/15/11
 * Update Author: Kasra Manavi
 */

#ifndef MALP_H_
#define MALP_H_

#include "boost/pointer_cast.hpp"

typedef ElementSet<LocalPlannerMethod<CfgType,WeightType> >::MethodPointer LocalPlannerPointer;
template <class CFG, class WEIGHT> class LocalPlanners;

template <class CFG, class WEIGHT> class MedialAxisLP: public LocalPlannerMethod<CFG, WEIGHT> {
  public:

    MedialAxisLP();
    MedialAxisLP(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~MedialAxisLP();

    virtual void PrintOptions(ostream& _os);
    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, 
        CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput, double _positionRes, 
        double _orientationRes, bool _checkCollision=true, 
        bool _savePath=false, bool _saveFailedPath=false);

  protected:
    virtual bool IsConnectedOneWay(Environment* _env, StatClass& _stats,
          shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, 
          CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
          double _positionRes, double _orientationRes,
          bool _checkCollision=true, bool _savePath=false, 
          bool _saveFailedPath=false);

    virtual bool IsConnectedRec(Environment* _env, StatClass& _stats,
          shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, 
          CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
          double _posRes, double _oriRes, int _itr=0);

    virtual bool EpsilonClosePath(Environment* _env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm,
          const CFG& _c1, const CFG& _c2, CFG& _mid, LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes);

    bool m_useBBX, m_cExact, m_pExact, m_positional;
    int m_clearance, m_penetration, m_historyLength, m_maxItr;
    string m_dm, m_vcm, m_maLP, m_envLP, m_macVC;
    double m_epsilon, m_sampleEpsilon;
};

// Definitions for Constructors and Destructor
template <class CFG, class WEIGHT>
MedialAxisLP<CFG, WEIGHT>::
MedialAxisLP() : LocalPlannerMethod<CFG, WEIGHT>() {
  this->SetName("MedialAxisLP");
}

template <class CFG, class WEIGHT> MedialAxisLP<CFG, WEIGHT>::
MedialAxisLP(XMLNodeReader& _node, MPProblem* in_pProblem) : 
  LocalPlannerMethod<CFG, WEIGHT>(_node, in_pProblem) {
    this->SetName("MedialAxisLP");

    m_vcm         = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
    m_dm          = _node.stringXMLParameter("dmMethod", true, "", "Distance Metric Method");
    m_maLP       = _node.stringXMLParameter("maLPMethod", true, "", "Local Planner Method");
    m_envLP      = _node.stringXMLParameter("envLPMethod", true, "", "Local Planner Method");
    m_macVC      = _node.stringXMLParameter("macVCMethod", true, "", "Local Planner Method");
    string strC     = _node.stringXMLParameter("clearanceType", true, "", "Clearance Computation (exact or approx)");
    string strP     = _node.stringXMLParameter("penetrationType", true, "", "Penetration Computation (exact or approx)");
    m_cExact = (strC.compare("exact")==0)?true:false;
    m_pExact = (strP.compare("exact")==0)?true:false;
    m_clearance       = _node.numberXMLParameter("clearanceRays", false, 10, 1, 1000, "Clearance Number");
    m_penetration     = _node.numberXMLParameter("penetrationRays", false, 10, 1, 1000, "Penetration Number");
    m_epsilon       = _node.numberXMLParameter("epsilon", false, 0.1, 0.0, 100.0, 
        "Epsilon-Close to the MA (fraction of the resolution)");
    m_sampleEpsilon = _node.numberXMLParameter("sampleEpsilon", false, 0.1, 0.0, 100.0, 
        "Epsilon-Close to the MA (fraction of the resolution)");
    m_historyLength     = _node.numberXMLParameter("historyLength", false, 5, 3, 1000, "History Length");
    m_maxItr        = _node.numberXMLParameter("maxItr", false, 2, 1, 1000, "Max Number of Recursive Iterations");
    m_useBBX         = _node.boolXMLParameter("useBBX", false, true, "Use the Bounding Box as an Obstacle");
    m_positional    = _node.boolXMLParameter("positional", false, true, "Use only positional DOFs");
  }

template <class CFG, class WEIGHT> 
MedialAxisLP<CFG, WEIGHT>::~MedialAxisLP() { }

// Definitions for I/O and Access
template <class CFG, class WEIGHT> 
void 
MedialAxisLP<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << "::  ";
  _os << "vcMethod = " << " " << m_vcm << " ";
  _os << endl;
}

template <class CFG, class WEIGHT> 
LocalPlannerMethod<CFG, WEIGHT>* 
MedialAxisLP<CFG, WEIGHT>::CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT>* copy = new MedialAxisLP<CFG, WEIGHT>(*this);
  return copy;
}

// Main IsConnected Function
template <class CFG, class WEIGHT> 
bool 
MedialAxisLP<CFG,WEIGHT>::IsConnected(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod> _dm,
    const CFG& _c1, const CFG& _c2, CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {  

  if (this->m_debug) {
    cout << "\nMedialAxisLP::IsConnected" << endl
      << "Start: " << _c1 << endl
      << "End  : " << _c2 << endl;
  }

  bool connected = false;
  _savePath = true;
  _stats.IncLPAttempts(this->GetNameAndLabel());
  if (this->m_debug) {
    VDComment("Initial CFGs");
    VDAddTempCfg(_c1,true);
    VDAddTempCfg(_c2,true);
    VDClearComments();
  }

  //clear _lpOutput                                                                                                       
  _lpOutput->path.clear();
  _lpOutput->edge.first.SetWeight(0);
  _lpOutput->edge.second.SetWeight(0);
  _lpOutput->savedEdge.clear();

  connected = IsConnectedRec(_env, _stats, _dm, _c1, _c2, _col, _lpOutput, _positionRes, _orientationRes, 1);

  // **** Print Edge.path File ****
  //if ( connected && this->m_debug) {
  //  stringstream f_out;
  //  f_out << "MALP_Edge." << rand()%1000+1 << ".path";
  //  cout << "MALP Edge Path File: " << f_out.str() << endl << endl << flush;
  //  WritePathConfigurations( f_out.str().c_str(), _lpOutput->path, _env);
  //}
  // **** END Edge.path Print  ****

  if ( connected ) {
    _stats.IncLPConnections(this->GetNameAndLabel() );  
    _lpOutput->edge.first.SetWeight(_lpOutput->path.size());
    _lpOutput->edge.second.SetWeight(_lpOutput->path.size());
  }

  if (this->m_debug) {
    VDClearLastTemp();
    VDClearLastTemp();
  }

  return connected;
}

template <class CFG, class WEIGHT> 
bool 
MedialAxisLP<CFG,WEIGHT>::IsConnectedOneWay(Environment* _env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm,
    const CFG& _c1, const CFG& _c2, CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes, bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  return IsConnectedRec(_env, _stats, _dm, _c1, _c2, _col, _lpOutput, _positionRes, _orientationRes, 1);
};

// Recursive IsConnected function
template <class CFG, class WEIGHT> 
bool 
MedialAxisLP<CFG,WEIGHT>::IsConnectedRec(Environment* _env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm,
    const CFG& _c1, const CFG& _c2, CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _posRes, double _oriRes, int _itr) {

  if (this->m_debug) {
    cout << "  MedialAxisLP::IsConnectedRec" << endl
      << "  Start  : " << _c1 << endl
      << "  End    : " << _c2 << endl;
  }

  if ( _itr > m_maxItr ) return false;

  ValidityChecker<CFG>*                      vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vcm);
  LocalPlannerPointer envLPMethod = this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_envLP);
  LPOutput<CFG, WEIGHT> maLPOutput, tmpLPOutput;
  int cdCounter=0, nTicks=0;
  CFG mid;

  Cfg* diff = _c1.CreateNewCfg();
  diff->subtract(_c1, _c2);
  nTicks = (int)max(diff->PositionMagnitude()/_posRes,
      diff->OrientationMagnitude()/_oriRes);

  if ( nTicks <= 1 ) {
    if ( envLPMethod->IsConnected(_env,_stats,_dm,_c1,_c2,_col,
          &tmpLPOutput,_posRes,_oriRes,true,true,true) ) {
      for ( size_t j=0; j<tmpLPOutput.path.size(); ++j)
        _lpOutput->path.push_back(tmpLPOutput.path[j]);
      return true;  
    } else {
      return false;
    }
  }

  // Test for MA closeness
  if ( EpsilonClosePath(_env,_stats,_dm,_c1,_c2,mid,&maLPOutput,_posRes,_oriRes) ) {
    for ( size_t j=0; j<maLPOutput.path.size(); ++j)
      _lpOutput->path.push_back(maLPOutput.path[j]);
    return true;
  }

  // Failed epsilon close, recurse
  if (this->m_debug) cout << "  New Mid: " << mid << endl;
  if (this->m_debug) VDAddTempCfg(mid,true);
  LPOutput<CFG, WEIGHT> lpOutputS, lpOutputE;

  if ( !IsConnectedRec(_env,_stats,_dm,_c1,mid,_col,&lpOutputS,_posRes,_oriRes,_itr+1) ) {
    if (this->m_debug) VDClearLastTemp();
    return false;
  }
  if ( !IsConnectedRec(_env,_stats,_dm,mid,_c2,_col,&lpOutputE,_posRes,_oriRes,_itr+1) ) {
    if (this->m_debug) VDClearLastTemp();
    return false;
  }

  // Push path onto local planner output
  _lpOutput->path.push_back(_c1);
  for ( size_t i=0; i<lpOutputS.path.size(); ++i)
    _lpOutput->path.push_back(lpOutputS.path[i]);
  _lpOutput->path.push_back(mid);
  for ( size_t i=0; i<lpOutputE.path.size(); ++i)
    _lpOutput->path.push_back(lpOutputE.path[i]);
  _lpOutput->path.push_back(_c2);

  VDClearLastTemp();
  _stats.IncLPCollDetCalls( this->GetNameAndLabel(), cdCounter );  
  return true;
};

template <class CFG, class WEIGHT> bool MedialAxisLP<CFG,WEIGHT>::
EpsilonClosePath( Environment *_env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm,
    const CFG &_c1, const CFG &_c2, CFG &_mid, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _posRes, double _oriRes) {

  if (this->m_debug) cout << "MedialAxisLP::EpsilonClosePath()" << endl;
  ValidityChecker<CFG>*                       vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_macVC);
  LocalPlannerPointer envLPMethod = this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_envLP);

  shared_ptr<MedialAxisClearanceValidity> macVCM = boost::dynamic_pointer_cast<MedialAxisClearanceValidity>(vcm);

  LPOutput<CFG, WEIGHT> maLPOutput, tmpLPOutput, testLPOutput;
  CFG col, tmp;
  int nTicks;
  bool passed=true, found=false;

  // Closer than epsilon so calc pushed mid and test at _env res
  if ( _dm->Distance(_env,_c1,_c2) < m_epsilon ) {
    if (this->m_debug) cout << "Segment is shorter than epsilon..." << endl;

    _mid.FindIncrement(_c1,_c2,&nTicks,_posRes,_oriRes);
    for (int i=_mid.PosDOF(); i<_mid.DOF(); i++) {
      if ( _mid.GetSingleParam(i) > 0.5 )
        _mid.SetSingleParam(i,_mid.GetSingleParam(i) - 1.0, false);
    }
    _mid.multiply(_mid,((double)nTicks)/2.0);
    _mid.add(_c1,_mid);

    PushToMedialAxis(this->GetMPProblem(),_env,_mid,_stats,m_vcm,m_dm,m_cExact,m_clearance,
        m_pExact,m_penetration,m_useBBX,m_sampleEpsilon,m_historyLength,this->m_debug, m_positional);
    return ( envLPMethod->IsConnected(_env,_stats,_dm,_c1,_c2,col,_lpOutput,
          _posRes,_oriRes,true,true,true) );
  }

  // Calculate relationship between resolution and m_epsilon
  double rEpsilon = _dm->Distance(_env,_c1,_c2)/m_epsilon;

  if (this->m_debug) cout << " Ticks wanted: " << rEpsilon << endl;

  tmp.FindIncrement(_c1,_c2,&nTicks,_posRes,_oriRes);
  rEpsilon = (double)nTicks/rEpsilon;
  double posEps = rEpsilon*_posRes;
  double oriEps = rEpsilon*_oriRes;

  if (this->m_debug) cout << " Ticks calculated: " << nTicks << endl;

  // Check if path is epsilon close, call again without collision checking for LPOuput if failed
  maLPOutput.path.push_back(_c1);
  LocalPlannerPointer maLPMethod = this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_maLP);
  passed  = maLPMethod->IsConnected(_env,_stats,_dm,_c1,_c2,col,&testLPOutput,posEps,oriEps,true,true,true);
  if ( passed ) {
    for ( size_t j=0; j<testLPOutput.path.size(); ++j)
      maLPOutput.path.push_back(testLPOutput.path[j]);
  } else {
    maLPMethod->IsConnected(_env,_stats,_dm,_c1,_c2,col,&maLPOutput,posEps,oriEps,false,true,true);
  }
  maLPOutput.path.push_back(_c2);

  // Save pushed mid
  vector< pair<CfgType,CfgType> > history = macVCM->GetHistory();

  if (history.size() <= 1) {
    if (history.size() == 1 &&
        !(history[0].first == history[0].second) ) { // Validity Checker didn't push properly
      _mid = history[0].second;
    } else {
      _mid = maLPOutput.path[maLPOutput.path.size()/2];
      PushToMedialAxis(this->GetMPProblem(),_env,_mid,_stats,m_vcm,m_dm,m_cExact,m_clearance,
          m_pExact,m_penetration,m_useBBX,m_sampleEpsilon,m_historyLength,this->m_debug,m_positional);
    }
  } else {
    for (size_t i=0; i<history.size(); ++i) {
      if (history[i].first == maLPOutput.path[maLPOutput.path.size()/2] ||
          history[i].first == maLPOutput.path[maLPOutput.path.size()/2-1]) {
        _mid = history[i].second;
        found = true;
        break;
      }
    }
    if ( !found ) {
      _mid = maLPOutput.path[maLPOutput.path.size()/2];
      PushToMedialAxis(this->GetMPProblem(),_env,_mid,_stats,m_vcm,m_dm,m_cExact,m_clearance,
          m_pExact,m_penetration,m_useBBX,m_sampleEpsilon,m_historyLength,this->m_debug,m_positional);
    }
  }
  macVCM->ClearHistory();

  // If epsilon close, test at _env res
  if ( passed ) {
    if (this->m_debug) cout << "maLPMethod->IsConnected Passed: maLPOutput.size: " << maLPOutput.path.size() << endl;
    // Test individual segments
    for (size_t i=0; i<maLPOutput.path.size()-1; ++i) {
      if ( envLPMethod->IsConnected(_env,_stats,_dm,maLPOutput.path[i],maLPOutput.path[i+1],col,
            &testLPOutput,_posRes,_oriRes,true,true,true) ) {
        for ( size_t j=0; j<testLPOutput.path.size(); ++j)
          tmpLPOutput.path.push_back(testLPOutput.path[j]);
        if (i != maLPOutput.path.size()-2)
          tmpLPOutput.path.push_back(maLPOutput.path[i+1]);
      } else {
        passed = false;
        break;
      }
    }
    // If passed, save path and exit
    if (passed == true) {
      if (this->m_debug) cout << "envLPMethod->IsConnected Passed: tmpLPOutput.size: " << tmpLPOutput.path.size() << endl;
      for ( size_t j=0; j<tmpLPOutput.path.size(); ++j)
        _lpOutput->path.push_back(tmpLPOutput.path[j]);
      return true;
    } else {
      return false;
    }
  } else
    return false;
}

#endif
