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
        LOG_DEBUG_MSG("StraightLine::length not found");
      }
 
  int binary_search;
  if(TIXML_SUCCESS  == in_pNode->ToElement()->QueryIntAttribute("binary_search",&binary_search)) {
        binarySearch.SetValue(binary_search);
      } else {
        LOG_DEBUG_MSG("StraightLine::binary_search not Found");
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
  ///\todo fix lineSegment implementation!  very poor for counting stats, etc.
  if(lineSegmentLength.GetValue()) {
    Stats.IncLPCollDetCalls( "Straightline", cd_cntr );
    if( lineSegmentInCollision(_env, Stats, cd, dm, _c1, _c2, lpOutput, cd_cntr, positionRes)) {
       return false;  //not connected
    }
  }
    bool connected;
    if(binarySearch.GetValue()) 
      connected = IsConnectedSLBinary(_env, Stats, cd, dm, _c1, _c2, lpOutput, 
                                      cd_cntr, positionRes, orientationRes, 
                                      checkCollision, savePath, saveFailedPath);
    else
      connected = IsConnectedSLSequential(_env, Stats, cd, dm, _c1, _c2, lpOutput, 
                                          cd_cntr, positionRes, orientationRes, 
                                          checkCollision, savePath, saveFailedPath);

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
  for(int i = 1; i < n_ticks; i++){ //don't need to check the ends, _c1 and _c2
    tick.Increment(incr);
    cd_cntr ++;
    if(checkCollision){
      //bool bbox_check = tick.InBoundingBox(_env);
      //bool col_check = tick.isCollision(_env,Stats,cd, *cdInfo,true,&(Callee));  
      //if(!bbox_check || col_check){  ///changed to precompute bool vals,
                                     ///compiler was optimizing and counts got screwed up --roger 9.17.2005
      if(!tick.InBoundingBox(_env) || 
         tick.isCollision(_env,Stats,cd,*cdInfo,true,&(Callee)) ) {
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
    MultiBody * lineSegment = new MultiBody(_env);
    //Creat a FreeBody  for this triangle
    FreeBody fb(lineSegment);
    fb.ReadBYU(cdtype, *((istream*)(&istr)));
    Transformation t=Transformation(Orientation(IdentityMatrix), center);
    fb.Configure(t);  //Transform it from (0,0,0) to center
    
    lineSegment->AddBody(&fb);  //Add this free body to MultiBody
    
    cd_cntr++; //?
    
    //Check collision
    //if( cd->IsInCollision(_env, Stats, *this->cdInfo, lineSegment) )
    std::string Callee(GetName()),Method("-StraightLine::lineSegmentInCollision");
    Callee+=Method;

    BoundingBox *bb =  _env->GetBoundingBox();
    for(int m = 0; m<lineSegment->GetFreeBodyCount(); ++m) {
      GMSPolyhedron &poly = lineSegment->GetFreeBody(m)->GetWorldPolyhedron();
      for(int j = 0 ; j < poly.numVertices ; j++){
        if (!bb->IfSatisfiesConstraints(poly.vertexList[j]))
        delete lineSegment;
        return true; //Collide (out of Bounding Box)
      }
    }

    if(cd->IsInCollision(_env, Stats, *this->cdInfo, lineSegment, true, &Callee) ) {
     delete lineSegment;
     return true; //Collide
   }
   delete lineSegment;
   return false;  //No collision
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
  if(!checkCollision)
    return IsConnectedSLSequential(_env, Stats, cd, dm, _c1, _c2,
				   lpOutput, cd_cntr, 
				   positionRes, orientationRes,
				   checkCollision, savePath, saveFailedPath);

  std::string Callee(GetName());
  std::string Method("-straightline::IsConnectedSLBinary");
  Callee=Callee+Method;

  int n_ticks;
  CFG incr;
  incr.FindIncrement(_c1, _c2, &n_ticks, positionRes, orientationRes);

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
       mid_cfg.isCollision(_env, Stats, cd, *cdInfo, true, &(Callee))) {
      return false;
    } else {
      if(i+1 != mid) 
	Q.push_back(make_pair(i, mid));
      if(mid+1 != j) 
	Q.push_back(make_pair(mid, j));      
    }
  }

  if(savePath || saveFailedPath) {
    CFG tick = _c1;
    for(int n=1; n<n_ticks; ++n) {
      tick.Increment(incr);
      lpOutput->path.push_back(tick);
    }
  }

  lpOutput->edge.first.SetWeight(lpOutput->edge.first.GetWeight() + n_ticks-1);
  lpOutput->edge.second.SetWeight(lpOutput->edge.second.GetWeight() + n_ticks-1);

  return true;
}


#endif
