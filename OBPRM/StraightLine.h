#ifndef StraightLine_h
#define StraightLine_h

#include "LocalPlannerMethod.h"

template <class CFG, class WEIGHT>
class StraightLine: public LocalPlannerMethod<CFG, WEIGHT> {
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
  StraightLine(cd_predefined _cdtype);
  StraightLine(cd_predefined _cdtype, TiXmlNode* in_pNode, MPProblem* in_pProblem);

  ///Destructor.	
  virtual ~StraightLine();

  //@}
  virtual bool SameParameters(const LocalPlannerMethod<CFG,WEIGHT> &other) const;

  //////////////////////
  // Access
  virtual char* GetName() const;
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os);
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  /**Check if two Cfgs could be connected by straight line.
   *   -# First, if lineSegmentLength is not zero, 
   *      this method calls lineSegmentInCollision to make sure 
   *      _c1 and _c2 could see each other. If the answer is No,
   *      false will be returned. Otherwise, goes step 2.
   *   -# Second, IsConnected_straightline_simple is called
   *      and return what it returns.
   *
   *@return See description above.
   *@see lineSegmentInCollision and IsConnected_straightline_simple
   */
  virtual bool IsConnected(Environment *env, Stat_Class& Stats,
			   CollisionDetection *cd,
			   DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
			   LPOutput<CFG, WEIGHT>* lpOutput,
			   double positionRes, double orientationRes,
			   bool checkCollision=true, 
			   bool savePath=false, bool saveFailedPath=false);

 protected:
  /**Check if two Cfgs could be connected by straight line.
   *This method implements straight line connection local planner
   *by checking collision of each Cfg along the line.
   *If the is any Cfg causes Robot collides with any obstacle,
   *false will be returned.
   *
   *@note if usingClearance is true, then the call will
   *be redirect to IsConnected_SLclearance
   *
   *@param env Used for isCollision
   *@param dm Used for IsConnected_SLclearance
   *@param _c1 start Cfg
   *@param _c2 goal Cfg
   *@param _lp Used for IsConnected_SLclearance 
   *@param lpOutput Used to record path information, such
   *as length of path...
   *
   *@return true if all Cfg are collision free.
   *@see IsConnected_straightline and Cfg::FindIncrement.
   */
  virtual bool 
    IsConnectedSLSequential(Environment *env, Stat_Class& Stats,
			    CollisionDetection *cd, DistanceMetric * dm, 
			    const CFG &_c1, const CFG &_c2, 
			    LPOutput<CFG,WEIGHT>* lpOutput, int &cd_cntr,
			    double positionRes, double orientationRes,
			    bool checkCollision=true, 
			    bool savePath=false, bool saveFailedPath=false);
  /**Check if the line, _c1->_c2, collides with any obstacle or not.
   *This is done by create a MultiBody, which is a triangle approximating
   *line segment _c1->_c2, and check collision of this triangle and other
   *obstacles in envronment.
   *@return True if there is collision. Following cases return false:
   *   -# the distance between _c1 and _c2 are shorter than lineSegmentLength
   *      (?maybe because this is too short so more expensive methods are used?)
   *   -# There is not collision between this triangle and other obstacles.
   *
   *@see CollisionDetection::IsInCollision
   */
  virtual bool 
    lineSegmentInCollision(Environment *env, Stat_Class& Stats, 
			   CollisionDetection *cd,
			   DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
			   LPOutput<CFG,WEIGHT>* lpOutput, 
			   int &cd_cntr, double positionRes);

  /**Check if two Cfgs could be connected by straight line with certain cleanrance.
   *This method uses binary search to check clearances of Cfgs between _c1 and 
   *_c2. If any Cfg with clearance less than 0.001 was found, false will be returned.
   *
   *@return true if no Cfg whose cleanrance is less than 0.001. Otherwise, false will be returned.
   */
  virtual bool 
    IsConnectedSLBinary(Environment *env, Stat_Class& Stats, 
			CollisionDetection *cd,
			DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
			LPOutput<CFG,WEIGHT>* lpOutput, int &cd_cntr,
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
  /**If true, local planner will check clearances of all Cfgs along path. 
    *(default is false)
    */
  //usingClearance and lineSegment were static in the original LocalPlanner
  //bool usingClearance;
  num_param<int> binarySearch;///<Mantain certain amount of clearance of Robot duing connection time.
  //int lineSegmentLength;///< default is 0.
  num_param<int>    lineSegmentLength;///< default is 0.

  cd_predefined cdtype;
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class StraightLine declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
StraightLine<CFG, WEIGHT>::
StraightLine(cd_predefined _cdtype) : LocalPlannerMethod<CFG, WEIGHT>(),
  lineSegmentLength      ("lineSegmentLength",        0,  0, 5000),
  binarySearch   ("binarySearch",          0,  0,    1) {
  lineSegmentLength.PutDesc      ("INTEGER","(lineSegmentLength, default 0");
  binarySearch.PutDesc   ("INTEGER","(check line sequentially(0 default) or with a binary search(1)");
  cdtype = _cdtype;

 


}

template <class CFG, class WEIGHT>
StraightLine<CFG, WEIGHT>::
StraightLine(cd_predefined _cdtype, TiXmlNode* in_pNode, MPProblem* in_pProblem) : LocalPlannerMethod<CFG, WEIGHT>(in_pNode,in_pProblem),
  lineSegmentLength      ("lineSegmentLength",        0,  0, 5000),
  binarySearch   ("binarySearch",          0,  0,    1) {
  lineSegmentLength.PutDesc      ("INTEGER","(lineSegmentLength, default 0");
  binarySearch.PutDesc   ("INTEGER","(check line sequentially(0 default) or with a binary search(1)");
  cdtype = _cdtype;
  LOG_DEBUG_MSG("StraightLine::StraightLine()");

  int length; 
  if(TIXML_SUCCESS  == in_pNode->ToElement()->QueryIntAttribute("length",&length)) {
        lineSegmentLength.SetValue(length);
      } else {
        LOG_DEBUG_MSG("MPStrategyMethod::length not found");
      }
 
  int binary_search;
  if(TIXML_SUCCESS  == in_pNode->ToElement()->QueryIntAttribute("binary_search",&binary_search)) {
        binarySearch.SetValue(binary_search);
      } else {
        LOG_DEBUG_MSG("MPStrategyMethod::binary_search not Found");
      }
  LOG_DEBUG_MSG("~StraightLine::StraightLine()");
}


template <class CFG, class WEIGHT>
StraightLine<CFG, WEIGHT>::
~StraightLine() {
}


template <class CFG, class WEIGHT>
bool
StraightLine<CFG, WEIGHT>::
SameParameters(const LocalPlannerMethod<CFG,WEIGHT> &other) const {
  bool result = false;
  if (lineSegmentLength.GetValue() == ((StraightLine<CFG,WEIGHT>&) other).lineSegmentLength.GetValue() &&  
      binarySearch.GetValue() == ((StraightLine<CFG,WEIGHT>&) other).binarySearch.GetValue())
    result = true;
  return result;
}


template <class CFG, class WEIGHT>
char*
StraightLine<CFG, WEIGHT>::
GetName() const {
  return "straightline";
}


template <class CFG, class WEIGHT>
void
StraightLine<CFG, WEIGHT>::
SetDefault() {
  LocalPlannerMethod<CFG, WEIGHT>::SetDefault();
  lineSegmentLength.PutValue(0);
  binarySearch.PutValue(0);
}

template <class CFG, class WEIGHT>
void
StraightLine<CFG, WEIGHT>::
ParseCommandLine(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {    
    if( lineSegmentLength.AckCmdLine(&i, argc, argv) ) {
    } 
    else if ( binarySearch.AckCmdLine(&i, argc, argv) ) {
    } else {
      cerr << "\nERROR ParseCommandLine: Don\'t understand \"";
      for(int j=0; j<argc; j++)
        cerr << argv[j] << " ";
      cerr << "\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit (-1);
    }
  }
}


template <class CFG, class WEIGHT>
void
StraightLine<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; lineSegmentLength.PrintUsage(_os);
  _os << "\n\t"; binarySearch.PrintUsage(_os);
 
  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG, class WEIGHT>
void
StraightLine<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os << GetName() << " ";
  _os << lineSegmentLength.GetFlag() << " " << lineSegmentLength.GetValue() << " ";
  _os << binarySearch.GetFlag() << " " << binarySearch.GetValue() << " ";
  _os << endl;
}


template <class CFG, class WEIGHT>
void
StraightLine<CFG, WEIGHT>::
PrintOptions(ostream& out_os) {
  out_os << "    " << GetName() << "::  ";
  out_os << "line segment length = " << " " << lineSegmentLength.GetValue() << " ";
  out_os << "binary search = " << " " << binarySearch.GetValue() << " ";
  out_os << endl;
}


template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
StraightLine<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new StraightLine<CFG, WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
bool 
StraightLine<CFG, WEIGHT>::
IsConnected(Environment *_env, Stat_Class& Stats,
	    CollisionDetection *cd, DistanceMetric *dm, 
	    const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
	    double positionRes, double orientationRes,
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {
	
  Stats.IncLPAttempts( "Straightline" );
  int cd_cntr = 0; 

  ///\todo fix this bug ... CD count not right.
  if(lineSegmentLength.GetValue() && lineSegmentInCollision(_env, Stats, cd, dm, _c1, _c2, lpOutput, cd_cntr, positionRes)) {
    Stats.IncLPCollDetCalls( "Straightline", cd_cntr );
    return false;	//not connected
  }

  bool connected;
  if(binarySearch.GetValue()) 
    connected = IsConnectedSLBinary(_env, Stats, cd, dm, _c1, _c2, lpOutput, cd_cntr, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
  else
    connected = IsConnectedSLSequential(_env, Stats, cd, dm, _c1, _c2, lpOutput, cd_cntr, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);

  if(connected)
    Stats.IncLPConnections( "Straightline" );
  
  Stats.IncLPCollDetCalls( "Straightline", cd_cntr );
  return connected;
}


template <class CFG, class WEIGHT>
bool
StraightLine<CFG, WEIGHT>::
       IsConnectedSLSequential(Environment *_env, Stat_Class& Stats,
			CollisionDetection *cd, DistanceMetric * dm, 
			const CFG &_c1, const CFG &_c2, 
			LPOutput<CFG,WEIGHT>* lpOutput, int &cd_cntr,
			double positionRes, double orientationRes,
			bool checkCollision, 
			bool savePath, bool saveFailedPath) {
  int n_ticks;
  CFG tick;
  tick = _c1; 
  CFG incr;
  incr.FindIncrement(_c1,_c2,&n_ticks,positionRes,orientationRes);
  std::string Callee(GetName());
  {std::string Method("-straightline::IsConnectedSLSequential");Callee=Callee+Method;}
  

  int nTicks = 0;
  for(int i = 0; i < n_ticks ; i++){
    tick.Increment(incr);
    cd_cntr ++;
    if(checkCollision){
      if(!tick.InBoundingBox(_env) ||
	 tick.isCollision(_env,Stats,cd, *cdInfo,true,&(Callee))){
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

template <class CFG, class WEIGHT>
bool 
StraightLine<CFG, WEIGHT>::
lineSegmentInCollision(Environment *_env, Stat_Class& Stats, 
		       CollisionDetection *cd, 
		       DistanceMetric* dm, const CFG &_c1, const CFG &_c2, 
		       LPOutput<CFG,WEIGHT> *lpOutput, 
		       int &cd_cntr, double positionRes) {
    CFG diff;
    //diff = _c1.CreateNewCfg();
    diff.subtract(_c1, _c2);
    int steps = (int)(diff.PositionMagnitude()/positionRes);
    //delete diff;
    
    if( steps <= lineSegmentLength.GetValue() )
      return false;
    
    //before really starting to connect the two cfgs, ie. _c1, _c2.
    //First make sure that there are 'seemingly connectable'.
    //i.e. their CMSs can see each other.
    //this method seems to only work for rigid-body robot.
    //moreover, it requires robot's CMS inside the robot.
    
    //if(Cfg::GetType() == RIGID_BODY) 
    Vector3D v1 = _c1.GetRobotCenterPosition();	//First pt for triangle
    Vector3D v2 = _c2.GetRobotCenterPosition(); //Second pt for triangle
    Vector3D v3 = v1 + Vector3D(0.000001, 0, 0);
    Vector3D center = (v1+v2+v3)/3.0;			//Third pt for triangle
    
    ///Create a triangle in MOVIE.BYU format.
    char str[200];
    sprintf(str, "%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s", "1 3 1 3 \n 1 3\n", v1[0], "  ", v1[1], 
	    "  ", v1[2], "\n", v2[0], "  ", v2[1], "  ", v2[2], "\n", v3[0], "  ", v3[1], 
	    "  ", v3[2], "\n1 2 -3 ");
    std::istringstream istr(str);
    
    //Creat a MultiBody for this triangle
    MultiBody * lineSegment = new MultiBody(_env);
    //Creat a FreeBody  for this triangle
    FreeBody fb(lineSegment);
    fb.ReadBYU(cdtype, *((istream*)(&istr)));
    Transformation t=Transformation(Orientation(IdentityMatrix), center);
    fb.Configure(t);	//Transform it from (0,0,0) to center
    
    lineSegment->AddBody(&fb);	//Add this free body to MultiBody
    
    cd_cntr ++; //?
    
    //Check collision
    //if( cd->IsInCollision(_env, Stats, *cdInfo, lineSegment) )
    std::string Callee(GetName()),Method("-StraightLine::lineSegmentInCollision");
    Callee+=Method;

    BoundingBox *bb =  _env->GetBoundingBox();
    for(int m = 0; m<lineSegment->GetFreeBodyCount(); ++m) {
      GMSPolyhedron &poly = lineSegment->GetFreeBody(m)->GetWorldPolyhedron();
      for(int j = 0 ; j < poly.numVertices ; j++){
	if (!bb->IfSatisfiesConstraints(poly.vertexList[j]))
	  return true; //Collide (out of Bounding Box)
      }
    }

    if(cd->IsInCollision(_env, Stats, *cdInfo, lineSegment, true, &Callee) )
      return true;	//Collide

    return false;	//No collision
}


template <class CFG, class WEIGHT>
bool 
StraightLine<CFG, WEIGHT>::
IsConnectedSLBinary(Environment *_env, Stat_Class& Stats, 
		    CollisionDetection *cd, 
		    DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
		    LPOutput<CFG,WEIGHT>* lpOutput, int &cd_cntr,
		    double positionRes, double orientationRes,  
		    bool checkCollision, 
		    bool savePath, bool saveFailedPath) {

  std::string Callee(GetName());
  {std::string Method("-straightline::IsConnectedSLBinary");Callee=Callee+Method;}
  
  if(checkCollision) {
    double clr = 0;

    ///Bound Shpere
    double rmax = _env->GetMultiBody(_env->GetRobotIndex())->GetBoundingSphereRadius();
    
    //Check if orientation of two Cfg are similar
    CFG c1diffc2;
    c1diffc2.subtract(_c1,_c2);
    bool sameOrientation = c1diffc2.OrientationMagnitude() <= orientationRes;

    typedef pair<CFG,CFG> cfgPair;
    deque<cfgPair> pairQ;
    pairQ.push_back(cfgPair(_c1,_c2));

    while(! pairQ.empty() ) {
      cfgPair &tmp = pairQ.front();	//dequeue
     
      //average of two Cfg, mid point.
      CFG mid;
      mid.WeightedSum(tmp.first, tmp.second, 0.5);
      
      cd_cntr ++;	//?
      
      if(cd->clearanceAvailable()) { //&& binarySearch
	//get clearance when robot's cfg is mid
	if((clr = mid.Clearance(_env,Stats,cd)) <= 0.001) // 0.001 tolerance.
	  return false;	///too close to obstacle, failed

	if(!sameOrientation) { //if have different orientation
	  clr -= rmax;	   //clearance - bounding sphere radius
	  if(clr < 0) clr = 0.0;
	}
      } else {
	if(!mid.InBoundingBox(_env) ||
	   mid.isCollision(_env,Stats,cd,*cdInfo,true,&(Callee)))
	  return false;
      }

      CFG diff;
      diff.subtract(tmp.first, tmp.second);
      double halfDist = diff.PositionMagnitude()/2;
      double halfOriDist = diff.OrientationMagnitude()/2;
      
      if(clr < halfDist) { //if clearance smaller than half of distance
	//if they are longer than resolution, partition it
	if(positionRes < halfDist || orientationRes < halfOriDist ) {
	  CFG tmp1;
	  tmp1.WeightedSum(tmp.first, mid, 1.0-clr/halfDist);
	  CFG tmp2;
	  tmp2.WeightedSum(mid, tmp.second, clr/halfDist);
	  pairQ.push_back(cfgPair(tmp.first, tmp1));
	  pairQ.push_back(cfgPair(tmp2, tmp.second)); 
       	}  
      }
      pairQ.pop_front();
    } //end while
  }//end if

  int steps = 0;  
  CFG incr;
  incr.FindIncrement(_c1,_c2,&steps,positionRes,orientationRes);
  if(savePath || saveFailedPath){
    CFG tick = _c1;
    for(int i=0; i<steps; i++) {
      tick.Increment(incr);
      lpOutput->path.push_back(tick);
    }
  }

  lpOutput->edge.first.SetWeight(lpOutput->edge.first.GetWeight() + steps);
  lpOutput->edge.second.SetWeight(lpOutput->edge.second.GetWeight() + steps);

  return true; //success

}//end of IsConnected_SLclearance


#endif
