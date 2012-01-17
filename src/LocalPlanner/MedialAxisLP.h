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

#include "StraightLine.h"
typedef ElementSet<LocalPlannerMethod<CfgType,WeightType> >::MethodPointer LocalPlannerPointer;
template <class CFG, class WEIGHT> class LocalPlanners;

template <class CFG, class WEIGHT> class MedialAxisLP: public StraightLine<CFG, WEIGHT> {
 public:

  MedialAxisLP();
  MedialAxisLP(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~MedialAxisLP();

  virtual void PrintOptions(ostream& out_os);
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual bool IsConnected(Environment *env, StatClass& Stats,
    shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
    CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput, double positionRes, 
    double orientationRes, bool checkCollision=true, 
    bool savePath=false, bool saveFailedPath=false);

	MPProblem* mp;
  bool use_bbx,bi_search,m_cExact,m_pExact;
  int clearance,penetration,history_len,m_maxItr;
  string str_dm,str_vcm,m_maStrLP,m_envStrLP,m_macStrVC;
  double m_epsilon, m_sampleEpsilon;

 protected:
  virtual bool 
    IsConnectedOneWay(Environment *env, StatClass& Stats,
                      shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
                      CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput,
                      double positionRes, double orientationRes,
                      bool checkCollision=true, bool savePath=false, 
                      bool saveFailedPath=false);

  virtual bool 
    IsConnectedRec(Environment *_env, StatClass& _stats,
                   shared_ptr<DistanceMetricMethod> _dm, const CFG &_c1, const CFG &_c2, 
                   CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
                   double _posRes, double _oriRes, int _itr=0);

  virtual bool
	  EpsilonClosePath(Environment *_env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm,
		  							 const CFG &_c1, const CFG &_c2, CFG &_mid, LPOutput<CFG, WEIGHT>* _lpOutput,
                     double _posRes, double _oriRes);

  int binarySearch;///<Mantain certain amount of clearance of Robot duing connection time.
  cd_predefined cdtype;
	LocalPlannerPointer m_maLPMethod, m_envLPMethod;
};

// Definitions for Constructors and Destructor
template <class CFG, class WEIGHT>
  MedialAxisLP<CFG, WEIGHT>::
  MedialAxisLP() : StraightLine<CFG, WEIGHT>() {
    this->SetName("MedialAxisLP");
}

template <class CFG, class WEIGHT> MedialAxisLP<CFG, WEIGHT>::
 MedialAxisLP(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
 StraightLine<CFG, WEIGHT>(in_Node, in_pProblem) {
  this->SetName("MedialAxisLP");

	mp              = in_pProblem;
	str_vcm         = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
	str_dm          = in_Node.stringXMLParameter("dm_method", true, "", "Distance Metric Method");
	m_maStrLP       = in_Node.stringXMLParameter("ma_lp_method", true, "", "Local Planner Method");
	m_envStrLP      = in_Node.stringXMLParameter("env_lp_method", true, "", "Local Planner Method");
  m_macStrVC      = in_Node.stringXMLParameter("mac_vc_method", true, "", "Local Planner Method");
	string strC     = in_Node.stringXMLParameter("clearance_type", true, "", "Clearance Computation (exact or approx)");
	string strP     = in_Node.stringXMLParameter("penetration_type", true, "", "Penetration Computation (exact or approx)");
  m_cExact = (strC.compare("exact")==0)?true:false;
  m_pExact = (strP.compare("exact")==0)?true:false;
	clearance       = in_Node.numberXMLParameter("clearance_rays", false, 10, 1, 1000, "Clearance Number");
	penetration     = in_Node.numberXMLParameter("penetration_rays", false, 10, 1, 1000, "Penetration Number");
	m_epsilon       = in_Node.numberXMLParameter("epsilon", false, 0.1, 0.0, 100.0, 
                                               "Epsilon-Close to the MA (fraction of the resolution)");
	m_sampleEpsilon = in_Node.numberXMLParameter("epsilon", false, 0.1, 0.0, 100.0, 
                                               "Epsilon-Close to the MA (fraction of the resolution)");
	history_len     = in_Node.numberXMLParameter("history_len", false, 5, 3, 1000, "History Length");
	m_maxItr        = in_Node.numberXMLParameter("max_itr", false, 2, 1, 1000, "Max Number of Recursive Iterations");
  bi_search       = in_Node.boolXMLParameter("bi_search", false, true, "Binary Search Straightline");
	use_bbx         = in_Node.boolXMLParameter("use_bbx", false, true, "Use the Bounding Box as an Obstacle");

  this->m_vcMethod = str_vcm;
	for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr)
    if (citr->getName() == "lp_methods") {
      LocalPlanners<CFG, WEIGHT>* lp = new LocalPlanners<CFG, WEIGHT>(*citr, in_pProblem);
      m_maLPMethod = lp->GetLocalPlannerMethod(m_maStrLP);
      m_envLPMethod = lp->GetLocalPlannerMethod(m_envStrLP);
    } else {
        citr->warnUnknownNode();
    }

}

template <class CFG, class WEIGHT> MedialAxisLP<CFG, WEIGHT>::~MedialAxisLP() { }

// Definitions for I/O and Access
template <class CFG, class WEIGHT> void MedialAxisLP<CFG, WEIGHT>::PrintOptions(ostream& out_os) {
  out_os << "    " << this->GetName() << "::  ";
  out_os << "binary search = " << " " << bi_search << " ";
  out_os << "vcMethod = " << " " << this->m_vcMethod << " ";
  out_os << endl;
}

template <class CFG, class WEIGHT> LocalPlannerMethod<CFG, WEIGHT>* MedialAxisLP<CFG, WEIGHT>::CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT>* _copy = new MedialAxisLP<CFG, WEIGHT>(*this);
  return _copy;
}


// Main IsConnected Function
template <class CFG, class WEIGHT> bool MedialAxisLP<CFG,WEIGHT>::
 IsConnected(Environment *_env, StatClass& Stats,
             shared_ptr< DistanceMetricMethod>dm,
             const CFG &_c1, const CFG &_c2, CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput,
             double positionRes, double orientationRes,
             bool checkCollision, 
             bool savePath, bool saveFailedPath) {  

  //cout << "\nMedialAxisLP::IsConnected" << endl;
  //cout << "Start: " << _c1 << endl;
  //cout << "End  : " << _c2 << endl;

  bool connected = false;
  savePath = true;
  Stats.IncLPAttempts(this->GetName());
  VDComment("Initial CFGs");
  VDAddTempCfg(_c1,true);
  VDAddTempCfg(_c2,true);
  VDClearComments();

  lpOutput->path.clear();
  connected = IsConnectedRec(_env, Stats, dm, _c1, _c2, _col, lpOutput, positionRes, orientationRes, 1);

	//**** Print Edge.path File ****
  //if ( connected ) {
	//  stringstream f_out;
	//  f_out << "MALP_Edge." << rand()%1000+1 << ".path";
	//  cout << "MALP Edge Path File: " << f_out.str() << endl << endl << flush;
	//  WritePathConfigurations( f_out.str().c_str(), lpOutput->path, _env);
  //}
	// **** END Edge.path Print  ****

  if ( connected )
    Stats.IncLPConnections(this->GetName() );  
  VDClearLastTemp();
  VDClearLastTemp();
  return connected;
}

template <class CFG, class WEIGHT> bool MedialAxisLP<CFG,WEIGHT>::
 IsConnectedOneWay(Environment *_env, StatClass& Stats,shared_ptr< DistanceMetricMethod >dm,
                   const CFG &_c1, const CFG &_c2, CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput,
                   double positionRes, double orientationRes, bool checkCollision, 
                   bool savePath, bool saveFailedPath) {
  return IsConnectedRec(_env, Stats, dm, _c1, _c2, _col, lpOutput, positionRes, orientationRes, 1);
};

// Recursive IsConnected function
template <class CFG, class WEIGHT> bool MedialAxisLP<CFG,WEIGHT>::
 IsConnectedRec(Environment *_env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm,
                const CFG &_c1, const CFG &_c2, CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
                double _posRes, double _oriRes, int _itr) {

  //cout << "  MedialAxisLP::IsConnectedRec" << endl;
  //cout << "  Start  : " << _c1 << endl;
  //cout << "  End    : " << _c2 << endl;

  if ( _itr > m_maxItr ) return false;

  ValidityChecker<CFG>*                      vc = mp->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->m_vcMethod);
  LPOutput<CFG, WEIGHT> maLPOutput, tmpLPOutput;
  bool passed=true;
  int cd_cntr=0;
  CFG mid;

  // TODO: Update to use DIST, not GAP
  double gap=0.0, dist=_dm->Distance(_env,_c1,_c2);
  for(int i=0; i<_c1.DOF(); ++i) {
    if ( i < _c1.PosDOF() ) gap += pow((_c2.GetSingleParam(i)-_c1.GetSingleParam(i)),2);
	  else                    gap += pow((DirectedAngularDistance(_c2.GetSingleParam(i),_c1.GetSingleParam(i))),2);
  }

  if ( sqrt(gap) < _env->GetPositionRes() ) {
	  if ( m_envLPMethod->IsConnected(_env,_stats,_dm,_c1,_c2,_col,
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
  //cout << "  New Mid: " << mid << endl;
  VDAddTempCfg(mid,true);
  LPOutput<CFG, WEIGHT> lpOutputS, lpOutputE;

  if ( !IsConnectedRec(_env,_stats,_dm,_c1,mid,_col,&lpOutputS,_posRes,_oriRes,_itr+1) ) {
    VDClearLastTemp();
    return false;
	}
  if ( !IsConnectedRec(_env,_stats,_dm,mid,_c2,_col,&lpOutputE,_posRes,_oriRes,_itr+1) ) {
    VDClearLastTemp();
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
  _stats.IncLPCollDetCalls( this->GetName(), cd_cntr );  
  return true;
};

template <class CFG, class WEIGHT> bool MedialAxisLP<CFG,WEIGHT>::
 EpsilonClosePath( Environment *_env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm,
                   const CFG &_c1, const CFG &_c2, CFG &_mid, LPOutput<CFG, WEIGHT>* _lpOutput,
                   double _posRes, double _oriRes) {

  ValidityChecker<CFG>*                      vc = mp->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_macStrVC);
  LPOutput<CFG, WEIGHT> maLPOutput, tmpLPOutput;
  CFG col;
  bool passed = true;

  // TODO: Convert to use DIST, no GAP
  double gap = 0.0, dist = _dm->Distance(_env,_c1,_c2);
  for(int i=0; i<_c1.DOF(); ++i) {
    if ( i < _c1.PosDOF() ) gap += pow((_c2.GetSingleParam(i)-_c1.GetSingleParam(i)),2);
	  else                    gap += pow((DirectedAngularDistance(_c2.GetSingleParam(i),_c1.GetSingleParam(i))),2);
  }
  //cout << "MedialAxisLP::EpsilonClosePath::Gap/Dist: " << sqrt(gap) << "/" << dist << endl;

  // Closer than epsilon so calc pushed mid and test at env res
  if ( sqrt(gap) < m_epsilon ) {
    _mid.add(_c1,_c2);
    _mid.divide(_mid,2);
    PushToMedialAxis(mp,_env,_mid,_stats,str_vcm,str_dm,m_cExact,clearance,
                     m_pExact,penetration,use_bbx,m_sampleEpsilon,history_len,false);
	  return ( m_envLPMethod->IsConnected(_env,_stats,_dm,_c1,_c2,col,_lpOutput,
                                        _posRes,_oriRes,true,true,true) );
	}

  // Check if path is epsilon close
  maLPOutput.path.push_back(_c1);
  passed = m_maLPMethod->IsConnected(_env,_stats,_dm,_c1,_c2,col,&maLPOutput,m_epsilon,m_epsilon,true,true,true);
  maLPOutput.path.push_back(_c2);

  // Save pushed mid
  vector< pair<CfgType,CfgType> > history = vcm->GetHistory();
  //cout << "History.size() = " << history.size() << "   maLPOutput.size() = " << maLPOutput.path.size() << endl;
  if (history.size() <= 1) {
    if (history.size() == 1 &&
        !(history[0].first == history[0].second) ) { // Validity Checker didn't push properly
	    _mid = history[0].second;
		} else {
      //cout << "Manually set mid" << endl;
      _mid = maLPOutput.path[maLPOutput.path.size()/2];
      PushToMedialAxis(mp,_env,_mid,_stats,str_vcm,str_dm,m_cExact,clearance,
                       m_pExact,penetration,use_bbx,m_sampleEpsilon,history_len,false);
    }
  } else {
    for (int i=0; i<history.size(); ++i) {
      if (history[i].first == maLPOutput.path[maLPOutput.path.size()/2] ||
          history[i].first == maLPOutput.path[maLPOutput.path.size()/2-1]) {
				//cout << "Found mid in history" << endl;
	  		  _mid = history[i].second;
		  	  break;
			}
		}
  }
  vcm->ClearHistory();

  // If epsilon close, test at env res
	if ( passed ) {
    //cout << "maLPMethod->IsConnected Passed" << endl;

    // Test individual segments
    for (int i=0; i<maLPOutput.path.size()-1; ++i) {
	    if ( m_envLPMethod->IsConnected(_env,_stats,_dm,maLPOutput.path[i],maLPOutput.path[i+1],col,
                                      &tmpLPOutput,_posRes,_oriRes,true,true,true) ) {
        if (i != maLPOutput.path.size()-2)
          tmpLPOutput.path.push_back(maLPOutput.path[i+1]);
      } else {
        passed = false;
        break;
      }
		}
    // If passed, save path and exit
    if (passed == true) {
      //cout << "envLPMethod->IsConnected Passed" << endl;
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
