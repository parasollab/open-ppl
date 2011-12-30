/**
 * StraightLine.h
 * This class performs straight line local planning which is used by a variety of computations,
 * including other local planners.
 *
 * Last Updated : 11/15/11
 * Update Author: Kasra Manavi
 */
#ifndef STRAIGHTLINE_H_
#define STRAIGHTLINE_H_

#include <deque>
#include "LocalPlannerMethod.h"
#include "type_traits/is_closed_chain.h" //used to switch between default and specialized impl. of IsConnected
#include "ValidityChecker.hpp"
#include "Cfg_reach_cc.h"

template <class CFG, class WEIGHT>
class StraightLine: public LocalPlannerMethod<CFG, WEIGHT> {
 public:
  
  /** @name Constructors and Destructor */
  // @{
  StraightLine();
  StraightLine(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~StraightLine();
  // @}

  virtual void PrintOptions(ostream& out_os);
  virtual string GetVCMethod();
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  /**
   * Check if two Cfgs could be connected by straight line.
   *   -# First, if lineSegmentLength is not zero, 
   *      this method calls lineSegmentInCollision to make sure 
   *      _c1 and _c2 could see each other. If the answer is No,
   *      false will be returned. Otherwise, goes step 2.
   *   -# Second, IsConnected_straightline_simple is called
   *      and return what it returns.
   *
   * @return See description above.
   * @see lineSegmentInCollision and IsConnected_straightline_simple
   */
  // Wrapper function to call appropriate impl based on CFG type
  virtual bool IsConnected(Environment *env, Stat_Class& Stats,
         shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
         LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision=true, 
         bool savePath=false, bool saveFailedPath=false) {
    //clear lpOutput
    lpOutput->path.clear();
    lpOutput->edge.first.SetWeight(0);
    lpOutput->edge.second.SetWeight(0);
    lpOutput->savedEdge.clear();
    return _IsConnected<CFG>(env, Stats, dm, _c1, _c2, _col,
			     lpOutput, positionRes, orientationRes,
			     checkCollision, savePath, saveFailedPath);
  }
  // Default for non closed chains
  template <typename Enable>
  bool _IsConnected(Environment *_env, Stat_Class& Stats,
         shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, CFG &_col,
         LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision=true, 
         bool savePath=false, bool saveFailedPath=false,
	 typename boost::disable_if<is_closed_chain<Enable> >::type* dummy = 0
	);
  // Specialization for closed chains
  template <typename Enable>
  bool _IsConnected(Environment *_env, Stat_Class& Stats,
         shared_ptr<DistanceMetricMethod>dm, const CFG &_c1, const CFG &_c2, CFG &_col,
         LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision=true, 
         bool savePath=false, bool saveFailedPath=false,
	 typename boost::enable_if<is_closed_chain<Enable> >::type* dummy = 0
	);

  string m_vcMethod;

 protected:
  /**
   * Check if two Cfgs could be connected by straight line.
   * This method implements straight line connection local planner
   * by checking collision of each Cfg along the line.
   * If the is any Cfg causes Robot collides with any obstacle,
   * false will be returned.
   *
   * @note if usingClearance is true, then the call will
   *       be redirect to IsConnected_SLclearance
   *
   * @param env Used for isCollision
   * @param dm Used for IsConnected_SLclearance
   * @param _c1 start Cfg
   * @param _c2 goal Cfg
   * @param _lp Used for IsConnected_SLclearance 
   * @param lpOutput Used to record path information, such as length of path...
   *
   * @return true if all Cfg are collision free.
   * @see IsConnected_straightline and Cfg::FindIncrement.
   */
  virtual bool 
    IsConnectedSLSequential(Environment *env, Stat_Class& Stats,
          shared_ptr<DistanceMetricMethod > dm, 
          const CFG &_c1, const CFG &_c2, CFG &_col,
          LPOutput<CFG,WEIGHT>* lpOutput, int &cd_cntr,
          double positionRes, double orientationRes,
          bool checkCollision=true, 
          bool savePath=false, bool saveFailedPath=false);
  /**
   * Check if the line, _c1->_c2, collides with any obstacle or not.
   * This is done by create a MultiBody, which is a triangle approximating
   * line segment _c1->_c2, and check collision of this triangle and other
   * obstacles in envronment.
   * @return True if there is collision. Following cases return false:
   *   -# the distance between _c1 and _c2 are shorter than lineSegmentLength
   *      (?maybe because this is too short so more expensive methods are used?)
   *   -# There is not collision between this triangle and other obstacles.
   *
   * @see CollisionDetection::IsInCollision
   */
  virtual bool 
    lineSegmentInCollision(Environment *env, Stat_Class& Stats, 
         shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
         LPOutput<CFG,WEIGHT>* lpOutput, 
         int &cd_cntr, double positionRes);

  /**
   * Check if two Cfgs could be connected by straight line with certain cleanrance.
   * This method uses binary search to check clearances of Cfgs between _c1 and 
   * _c2. If any Cfg with clearance less than 0.001 was found, false will be returned.
   *
   * @return true if no Cfg whose cleanrance is less than 0.001. Otherwise, false will be returned.
   */
  virtual bool 
    IsConnectedSLBinary(Environment *env, Stat_Class& Stats, 
      shared_ptr<DistanceMetricMethod> dm, const CFG &_c1, const CFG &_c2, 
      CFG &_col, LPOutput<CFG,WEIGHT>* lpOutput, int &cd_cntr,
      double positionRes, double orientationRes,  
      bool checkCollision=true, 
      bool savePath=false, bool saveFailedPath=false);

  int binarySearch;
  int lineSegmentLength;// Default is 0
  cd_predefined cdtype;
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class StraightLine declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
StraightLine<CFG, WEIGHT>::StraightLine() : 
 LocalPlannerMethod<CFG, WEIGHT>() {
  this->SetName("StraightLine");
}

template <class CFG, class WEIGHT>
StraightLine<CFG, WEIGHT>::StraightLine(XMLNodeReader& in_Node, MPProblem* in_pProblem)  :
 LocalPlannerMethod<CFG, WEIGHT>(in_Node,in_pProblem) {
  this->SetName("StraightLine");
  lineSegmentLength = in_Node.numberXMLParameter("length", false, 0, 0, 5000, "lineSegmentLength"); 
  binarySearch = in_Node.numberXMLParameter("binary_search", false, 0, 0, 1, "binary search"); 
  m_vcMethod = in_Node.stringXMLParameter("vc_method", false, "", "Validity Test Method");
}

template <class CFG, class WEIGHT> 
StraightLine<CFG, WEIGHT>::~StraightLine() { }

template <class CFG, class WEIGHT> void
StraightLine<CFG, WEIGHT>::PrintOptions(ostream& out_os) {
  out_os << "    " << this->GetName() << "::  ";
  out_os << "line segment length = " << " " << lineSegmentLength << " ";
  out_os << "binary search = " << " " << binarySearch << " ";
  out_os << "vcMethod = " << " " << m_vcMethod << " ";
  out_os << endl;
}

template <class CFG, class WEIGHT> string
StraightLine<CFG, WEIGHT>::GetVCMethod() {
  return m_vcMethod;
}
 
template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
StraightLine<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new StraightLine<CFG, WEIGHT>(*this);
  return _copy;
}

//// default implementation for non closed chains
template <class CFG, class WEIGHT>
template <typename Enable>
bool 
StraightLine<CFG, WEIGHT>::
_IsConnected(Environment *_env, Stat_Class& Stats,
         shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
         CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision, 
         bool savePath, bool saveFailedPath,
 	 typename boost::disable_if<is_closed_chain<Enable> >::type* dummy) 
{ 
  Stats.IncLPAttempts("Straightline");
  int cd_cntr = 0; 
 
  ///\todo fix this bug ... CD count not right.
  ///\todo fix lineSegment implementation!  very poor for counting stats, etc.
  if(lineSegmentLength) {
    Stats.IncLPCollDetCalls("Straightline", cd_cntr );
    if( lineSegmentInCollision(_env, Stats, dm, _c1, _c2, lpOutput, cd_cntr, positionRes)) {
       return false;  //not connected
    }
  }
    bool connected;
    if(binarySearch) 
      connected = IsConnectedSLBinary(_env, Stats, dm, _c1, _c2, _col, lpOutput, 
                                      cd_cntr, positionRes, orientationRes, 
                                      checkCollision, savePath, saveFailedPath);
    else
      connected = IsConnectedSLSequential(_env, Stats, dm, _c1, _c2, _col, lpOutput, 
                                           cd_cntr, positionRes, orientationRes, 
                                           checkCollision, savePath, saveFailedPath);
    if(connected)
      Stats.IncLPConnections("Straightline");

    Stats.IncLPCollDetCalls("Straightline", cd_cntr );
    return connected;
}


//// specialized implementation for closed chains
template <class CFG, class WEIGHT>
template <typename Enable>
bool 
StraightLine<CFG, WEIGHT>::
_IsConnected(Environment *_env, Stat_Class& Stats,
         shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
         CFG &_col, LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision, 
         bool savePath, bool saveFailedPath,
	 typename boost::enable_if<is_closed_chain<Enable> >::type* dummy)
{
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vcMethod);
  string Callee(this->GetName());
  string Method("-straightline::IsConnectedSLSequential");
  CDInfo cdInfo;
  Callee=Callee+Method;
  
  Stats.IncLPAttempts("Straightline");
  int cd_cntr = 0; 
  
  bool connected;
  if(CFG::OrientationsDifferent(_c1, _c2)) {
    CFG intermediate;
    bool success = intermediate.GetIntermediate(_c1, _c2); 
    if(checkCollision){
      cd_cntr ++;
      if(!intermediate.InBoundingBox(_env) || 
          !vc->IsValid(vcm, intermediate, _env, Stats, cdInfo, true, &Callee)
        ) {
        return false;
      }
    }
    if(!success)
      return false;
    
    if(binarySearch) {
      connected = (IsConnectedSLBinary(_env, Stats, dm, 
				       _c1, intermediate, 
				       _col, lpOutput, cd_cntr, 
				       positionRes, orientationRes, 
				       checkCollision, savePath, saveFailedPath) 
		   &&
		   IsConnectedSLBinary(_env, Stats, dm,
				       intermediate, _c2,
				       _col, lpOutput, cd_cntr, 
				       positionRes, orientationRes, 
				       checkCollision, savePath, saveFailedPath)
		   );
      /*if(!connected) { //attempt other direction
	connected = (IsConnectedSLBinary(_env, Stats, dm, 
					 _c2, intermediate, 
					 _col, lpOutput, cd_cntr, 
					 positionRes, orientationRes, 
					 checkCollision, savePath, saveFailedPath) 
		     &&
		     IsConnectedSLBinary(_env, Stats, dm,
					 intermediate, _c1,
					 _col, lpOutput, cd_cntr, 
					 positionRes, orientationRes, 
					 checkCollision, savePath, saveFailedPath)
		     );
	if(savePath)
	  reverse(lpOutput->path.begin(), lpOutput->path.end());
      }*/
    } else {
      connected = (IsConnectedSLSequential(_env, Stats, dm, 
					   _c1, intermediate, 
					   _col, lpOutput, cd_cntr, 
					   positionRes, orientationRes, 
					   checkCollision, savePath, saveFailedPath) 
		   &&
		   IsConnectedSLSequential(_env, Stats, dm, 
					   intermediate, _c2, 
					   _col, lpOutput, cd_cntr, 
					   positionRes, orientationRes, 
					   checkCollision, savePath, saveFailedPath)
		   );
      
      /*if(!connected) { //attempt other direction
	connected = (IsConnectedSLSequential(_env, Stats, dm, 
					     _c2, intermediate, 
					     _col, lpOutput, cd_cntr, 
					     positionRes, orientationRes, 
					     checkCollision, savePath, saveFailedPath) 
		     &&
		     IsConnectedSLSequential(_env, Stats, dm, 
					     intermediate, _c1, 
					     _col, lpOutput, cd_cntr, 
					     positionRes, orientationRes, 
					     checkCollision, savePath, saveFailedPath)
		     );
	if(savePath)
	  reverse(lpOutput->path.begin(), lpOutput->path.end());
      }*/
    }
  } else {
    //cout << "orientations same\n";
    if(binarySearch) {
      connected = IsConnectedSLBinary(_env, Stats, dm,
				      _c1, _c2,
				      _col, lpOutput, cd_cntr, 
				      positionRes, orientationRes, 
				      checkCollision, savePath, saveFailedPath);
    } else {
      connected = IsConnectedSLSequential(_env, Stats, dm,
					  _c1, _c2,
					  _col, lpOutput, cd_cntr, 
					  positionRes, orientationRes, 
					  checkCollision, savePath, saveFailedPath);
    }
  }
  if(connected)
    Stats.IncLPConnections( "Straightline" );
  Stats.IncLPCollDetCalls( "Straightline", cd_cntr );
  return connected;
}


template <class CFG, class WEIGHT>
bool
StraightLine<CFG, WEIGHT>::
       IsConnectedSLSequential(Environment *_env, Stat_Class& Stats,
     shared_ptr< DistanceMetricMethod > dm, 
      const CFG &_c1, const CFG &_c2, CFG &_col,
      LPOutput<CFG,WEIGHT>* lpOutput, int &cd_cntr,
      double positionRes, double orientationRes,
      bool checkCollision, 
      bool savePath, bool saveFailedPath) { 
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vcMethod);
  int n_ticks;
  CFG tick;
  tick = _c1; 
  CFG incr;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  incr.FindIncrement(_c1,_c2,&n_ticks,positionRes,orientationRes, _env->GetRdRes());
#else
  incr.FindIncrement(_c1,_c2,&n_ticks,positionRes,orientationRes);
#endif
  string Callee(this->GetName());
  string Method("-straightline::IsConnectedSLSequential");
  CDInfo cdInfo;
  Callee=Callee+Method;
  
  
  int nTicks = 0;
  for(int i = 1; i < n_ticks; i++){ //don't need to check the ends, _c1 and _c2
    tick.Increment(incr);
    cd_cntr ++;
    if(checkCollision){
      if(!tick.InBoundingBox(_env) || 
          !vc->IsValid(vcm, tick, _env, Stats, cdInfo, true, &Callee)
        ) {
        if(tick.InBoundingBox(_env) && 
            !vc->IsValid(vcm, tick, _env, Stats, cdInfo, true, &Callee))   
          _col = tick;
        CFG neg_incr;
        neg_incr = incr; 
        neg_incr.negative(incr);
        tick.Increment(neg_incr);
        lpOutput->edge.first.SetWeight(lpOutput->edge.first.GetWeight() + nTicks);
        lpOutput->edge.second.SetWeight(lpOutput->edge.second.GetWeight() + nTicks);
        pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > tmp;
        tmp.first.first = _c1;
        tmp.first.second = tick;
        tmp.second.first = lpOutput->edge.first;
        tmp.second.second = lpOutput->edge.second;
        lpOutput->savedEdge.push_back(tmp);
        return false;
      }
    }
    if(savePath || saveFailedPath){
      lpOutput->path.push_back(tick);
    }
    nTicks++;
  }
  lpOutput->edge.first.SetWeight(lpOutput->edge.first.GetWeight() + nTicks);
  lpOutput->edge.second.SetWeight(lpOutput->edge.second.GetWeight() + nTicks);

  return true;
};

// TODO::Remove depreciated function
template <class CFG, class WEIGHT>
bool 
StraightLine<CFG, WEIGHT>::
lineSegmentInCollision(Environment *_env, Stat_Class& Stats, 
          shared_ptr<DistanceMetricMethod> dm, const CFG &_c1, const CFG &_c2, 
           LPOutput<CFG,WEIGHT> *lpOutput, 
           int &cd_cntr, double positionRes) {
    CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();
    CFG diff;
    CDInfo cdInfo;
    //diff = _c1.CreateNewCfg();
    diff.subtract(_c1, _c2);
    int steps = (int)(diff.PositionMagnitude()/positionRes);
    //delete diff;
    
    if( steps <= lineSegmentLength )
      return false;
    
    //before really starting to connect the two cfgs, ie. _c1, _c2.
    //First make sure that there are 'seemingly connectable'.
    //i.e. their CMSs can see each other.
    //this method seems to only work for rigid-body robot.
    //moreover, it requires robot's CMS inside the robot.
    
    //if(Cfg::GetType() == RIGID_BODY) 
    Vector3D v1 = _c1.GetRobotCenterPosition(); //First pt for triangle
    Vector3D v2 = _c2.GetRobotCenterPosition(); //Second pt for triangle
    Vector3D v3 = v1 + Vector3D(0.000001, 0, 0);
    Vector3D center = (v1+v2+v3)/3.0;     //Third pt for triangle
    
    ///Create a triangle in MOVIE.BYU format.
    char str[200];
    sprintf(str, "%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s", "1 3 1 3 \n 1 3\n", v1[0], "  ", v1[1], 
      "  ", v1[2], "\n", v2[0], "  ", v2[1], "  ", v2[2], "\n", v3[0], "  ", v3[1], 
      "  ", v3[2], "\n1 2 -3 ");
    std::istringstream istr(str);
    
    //Creat a MultiBody for this triangle
    shared_ptr<MultiBody> lineSegment(new MultiBody());
    //Creat a FreeBody  for this triangle
    FreeBody fb(lineSegment.get());
    fb.ReadBYU(*((istream*)(&istr)));
    fb.buildCDstructure(cdtype);
    Transformation t=Transformation(Orientation(IdentityMatrix), center);
    fb.Configure(t);  //Transform it from (0,0,0) to center
    
    //lineSegment->AddBody(&fb);  //Add this free body to MultiBody
    lineSegment->AddBody(fb);  //Add this free body to MultiBody
    
    cd_cntr++; //?
    
    //Check collision
    //if( cd->IsInCollision(_env, Stats, *this->cdInfo, lineSegment) )
    string Callee(this->GetName()),Method("-StraightLine::lineSegmentInCollision");
    Callee+=Method;

    shared_ptr<BoundingBox> bb =  _env->GetBoundingBox();
    for(int m = 0; m<lineSegment->GetFreeBodyCount(); ++m) {
      GMSPolyhedron &poly = lineSegment->GetFreeBody(m)->GetWorldPolyhedron();
      for(size_t j = 0 ; j < poly.vertexList.size() ; j++){
        if (!bb->IfSatisfiesConstraints(poly.vertexList[j]))
        return true; //Collide (out of Bounding Box)
      }
    }

    if(cd->IsInCollision(_env, Stats, cdInfo, lineSegment, true, &Callee) ) {
     return true; //Collide
   }
   return false;  //No collision
}


template <class CFG, class WEIGHT>
bool 
StraightLine<CFG, WEIGHT>::
IsConnectedSLBinary(Environment *_env, Stat_Class& Stats, 
       shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
        CFG &_col, LPOutput<CFG,WEIGHT>* lpOutput, int &cd_cntr,
        double positionRes, double orientationRes,  
        bool checkCollision, 
        bool savePath, bool saveFailedPath) {

  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vcMethod);
  
  if(!checkCollision)
    return IsConnectedSLSequential(_env, Stats, dm, _c1, _c2,
 				   _col, lpOutput, cd_cntr, 
 				   positionRes, orientationRes,
				   checkCollision, savePath, saveFailedPath);

  string Callee(this->GetName());
  string Method("-straightline::IsConnectedSLBinary");
  CDInfo cdInfo;
  Callee=Callee+Method;

  int n_ticks;
  CFG incr;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  incr.FindIncrement(_c1,_c2,&n_ticks,positionRes,orientationRes, _env->GetRdRes());
#else
  incr.FindIncrement(_c1, _c2, &n_ticks, positionRes, orientationRes);
#endif

  if(savePath || saveFailedPath) {
    CFG tick = _c1;
    for(int n=1; n<n_ticks; ++n) {
      tick.Increment(incr);
      lpOutput->path.push_back(tick);
    }
  }

  deque<pair<int,int> > Q;
  Q.push_back(make_pair(0, n_ticks));

  while(!Q.empty()) {
    pair<int,int> p = Q.front();
    int i = p.first;
    int j = p.second;
    Q.pop_front();

    int mid = i + (int)floor(((double)(j-i))/2);
    CFG mid_cfg = _c1;
    /*
    // this produces the exact same intermediate cfgs as the sequential 
    // version (no roundoff errors), but it is slow
    for(int z=0; z<mid; ++z)
      mid_cfg.Increment(incr);
    */
    // this produces almost the same intermediate cfgs as the sequential
    // version, but there may be some roundoff errors; it is much faster
    // than the above solution; the error should be much smaller than the
    // resolution so it should not be a problem
    mid_cfg.multiply(incr, mid);
    mid_cfg.add(_c1, mid_cfg);
    
    cd_cntr++;
    if(!mid_cfg.InBoundingBox(_env) ||
       !vc->IsValid(vcm, mid_cfg, _env, Stats, cdInfo, true, &Callee) ) {
      //if(mid_cfg.InBoundingBox(_env) &&
      //   !vc->IsValid(vcm, mid_cfg, _env, Stats, cdInfo, true, &Callee))
        _col=mid_cfg;
      return false;
    } else {
      if(i+1 != mid) 
	Q.push_back(make_pair(i, mid));
      if(mid+1 != j) 
	Q.push_back(make_pair(mid, j));      
    }
  }

  lpOutput->edge.first.SetWeight(lpOutput->edge.first.GetWeight() + n_ticks-1);
  lpOutput->edge.second.SetWeight(lpOutput->edge.second.GetWeight() + n_ticks-1);

  return true;
}


#endif
