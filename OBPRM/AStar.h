#ifndef AStar_h
#define AStar_h

#include "LocalPlannerMethod.h"

template <class CFG, class WEIGHT>
class AStar: public LocalPlannerMethod<CFG, WEIGHT> {
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
  AStar();
  ///Destructor.	
  virtual ~AStar();

  //@}
  virtual bool SameParameters(const LocalPlannerMethod<CFG,WEIGHT> &other) const;

  //////////////////////
  // Access
  virtual char* GetName() const = 0;
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy() = 0;
      /**Roughly check if two Cfgs could be connected using clearance.
        *Algorithm is given here:
        *   -# set clearance1 as clearance for _c1
        *   -# set clearance2 as clearance for _c2
        *   -# set dist as distance from _c1 to c2
        *   -# if clearance1+clearance2 > dist
        *       -# connected
        *   -# else
        *       -# not connected
        *
        *@see Cfg::ApproxCSpaceClearance and Cfg::Clearance
        */
  virtual 
    bool IsConnected(Environment *env, Stat_Class& Stats,
		     CollisionDetection *cd,
		     DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
		     LPOutput<CFG, WEIGHT>* lpOutput,
		     double positionRes, double orientationRes,
		     bool checkCollision=true, 
		     bool savePath=false, bool saveFailedPath=false);
 protected:
  virtual 
    bool IsConnectedOneWay(Environment *env, Stat_Class& Stats,
			   CollisionDetection *cd,
			   DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
			   LPOutput<CFG, WEIGHT>* lpOutput,
			   double positionRes, double orientationRes,
			   bool checkCollision=true, 
			   bool savePath=false, bool saveFailedPath=false);

  virtual int ChooseOptimalNeighbor(Environment *_env, Stat_Class& Stats, CollisionDetection *cd, DistanceMetric *dm, const CFG &_c1, const CFG &_c2, vector<Cfg*> &neighbors) = 0;
  //@}
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  //@{
  num_param<int>    n_tries;       ///< How many time will be tried to connect to goal. (not used!?)
  num_param<int>    n_neighbors;   ///< How many neighbors will be seached abound current Cfg. (not used?!)
  //@}


};

/////////////////////////////////////////////////////////////////////
//
//  definitions for class StraightLine declarations
//
/////////////////////////////////////////////////////////////////////


template <class CFG, class WEIGHT>
AStar<CFG, WEIGHT>::
AStar() : LocalPlannerMethod<CFG, WEIGHT>(),
  n_tries     ("tries",        6,  1, 20),
  n_neighbors ("neighbors",    3,  3, 15){
  n_tries.PutDesc     ("INTEGER", "(tries, default=6");
  n_neighbors.PutDesc ("INTEGER", "(neighbors, default=3");

}


template <class CFG, class WEIGHT>
AStar<CFG, WEIGHT>::
~AStar() {
}


template <class CFG, class WEIGHT>
bool
AStar<CFG, WEIGHT>::
SameParameters(const LocalPlannerMethod<CFG,WEIGHT> &other) const {
  bool result = false;
  if (n_tries.GetValue() == ((AStar<CFG,WEIGHT>&) other).n_tries.GetValue() && 
      n_neighbors.GetValue() == ((AStar<CFG,WEIGHT>&) other).n_neighbors.GetValue() )
    result = true;
  return result;
}


template <class CFG, class WEIGHT>
void
AStar<CFG, WEIGHT>::
SetDefault() {
  LocalPlannerMethod<CFG, WEIGHT>::SetDefault();
  n_tries.PutValue(6);
  n_neighbors.PutValue(3);
}


template <class CFG, class WEIGHT>
void
AStar<CFG, WEIGHT>::
ParseCommandLine(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    if( n_tries.AckCmdLine(&i, argc, argv) ) {
    } else if( n_neighbors.AckCmdLine(&i, argc, argv) ) { //only allow 3, 9, or 15
      if (n_neighbors.GetValue() > 9)
	n_neighbors.PutValue(15);
      else if (n_neighbors.GetValue() > 3)
	n_neighbors.PutValue(9);
      else //In case Parameters doesn't make sure it is at least 3
	n_neighbors.PutValue(3);
    } else {
      cerr << "\nERROR ParseCommandLine: Don\'t understand \"";
      for(int j=0; j<argc; j++)
        cerr << argv[j] << " ";
      cerr <<"\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit (-1);
    }
  }
}


template <class CFG, class WEIGHT>
void
AStar<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; n_tries.PrintUsage(_os);
  _os << "\n\t"; n_neighbors.PrintUsage(_os);
 
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
AStar<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os << GetName() << " ";
  _os << n_tries.GetFlag() << " " << n_tries.GetValue() << " ";
  _os << n_neighbors.GetFlag() << " " << n_neighbors.GetValue() << " ";
  _os << endl;
}

template <class CFG, class WEIGHT>
bool
AStar<CFG,WEIGHT>::
IsConnected(Environment *_env, Stat_Class& Stats, 
	    CollisionDetection *cd, DistanceMetric *dm, 
	    const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
	    double positionRes, double orientationRes,
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {
  bool connected = false;
  connected = IsConnectedOneWay(_env, Stats, cd, dm, _c1, _c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
  if (!connected) { //try the other way
    connected = IsConnectedOneWay(_env, Stats, cd, dm, _c2, _c1, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    if (savePath)
      reverse(lpOutput->path.begin(), lpOutput->path.end());
  }
  return connected;
}


template <class CFG, class WEIGHT>
bool
AStar<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, Stat_Class& Stats,
		  CollisionDetection *cd, DistanceMetric *dm, 
		  const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
		  double positionRes, double orientationRes,
		  bool checkCollision, 
		  bool savePath, bool saveFailedPath) {
  Stats.IncLPAttempts( "AStar" );
  int cd_cntr = 0;
  
  CFG p;
  p = _c1;
  CFG incr;
  incr = _c1; 
  CFG diagonal;
  diagonal = _c1;
  vector<Cfg*> neighbors;
  int n_ticks;

  
  std::string Callee, Method("-AStar::IsConnectedOneWay()");
  
  
  incr.FindIncrement(_c1,_c2,&n_ticks,positionRes,orientationRes);
  
  bool connected = true;
  int noTries=0;
  int nTicks = 0;
  
  std::string tmpStr = Callee+Method;
  do {
    /* First check the diagonal to find out if it it available */
    diagonal = p;
    diagonal.IncrementTowardsGoal(_c2,incr);
    
    cd_cntr++;

    Callee=diagonal.GetName();
    if(!diagonal.isCollision(_env,Stats,cd, *cdInfo , &tmpStr)){
      p = diagonal;
    } else {
      neighbors.clear();
      p.FindNeighbors(_env, Stats,  _c2, incr, cd, n_neighbors.GetValue(),
		       *cdInfo, neighbors);
      if (neighbors.size()==0) { 
	connected = false;
        pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > tmp;
	tmp.first.first = _c1;
	tmp.first.second = p;
	tmp.second.first = lpOutput->edge.first;
	tmp.second.second = lpOutput->edge.second;
	lpOutput->savedEdge.push_back(tmp);
	break;
      }
      p = *(neighbors[ ChooseOptimalNeighbor(_env, Stats, cd, dm, _c1, _c2, neighbors) ]);
    }
    nTicks++;   
    
    if(savePath || saveFailedPath) {
      lpOutput->path.push_back(p);
    }
    
    if ((++noTries> n_tries.GetValue()*n_ticks)) { //if num_of_try > total_ticks*6->give up
      connected = false;
      pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > tmp;
      tmp.first.first = _c1;
      tmp.first.second = p;
      tmp.second.first = lpOutput->edge.first;
      tmp.second.second = lpOutput->edge.second;
      lpOutput->savedEdge.push_back(tmp);
      break;
    }
    
  } while(!p.AlmostEqual(_c2));
  
  lpOutput->edge.first.SetWeight(lpOutput->edge.first.GetWeight() + nTicks);
  lpOutput->edge.second.SetWeight(lpOutput->edge.second.GetWeight() + nTicks);
  
  Stats.IncLPCollDetCalls("AStar", cd_cntr );
  if(connected)
    Stats.IncLPConnections( "AStar" );
  
  for(int i=0; i<neighbors.size();i++) {
      if (neighbors[i] != NULL)
        delete neighbors[i];
  }
  
  return connected;
};

//
// AStarDistance

template <class CFG, class WEIGHT>
class AStarDistance: public AStar<CFG, WEIGHT> {
 public:
  AStarDistance();
  virtual ~AStarDistance();

  virtual char* GetName() const;
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual int ChooseOptimalNeighbor(Environment *_env, Stat_Class& Stats,
				    CollisionDetection *cd, DistanceMetric *dm,
				    const CFG &_c1, const CFG &_c2,
				    vector<Cfg*> &neighbors); 
};

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::
AStarDistance() : AStar<CFG, WEIGHT>() {

}

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::
~AStarDistance() {

}

template <class CFG, class WEIGHT>
char*
AStarDistance<CFG, WEIGHT>::
GetName() const {
  return "a_star_distance";
}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
AStarDistance<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new AStarDistance<CFG, WEIGHT>(*this);
  return _copy;
}

//find Cfg closest to goal. ASTAR_DISTANCE
template <class CFG, class WEIGHT>
int
AStarDistance<CFG, WEIGHT>::
ChooseOptimalNeighbor(Environment *_env, Stat_Class& Stats, CollisionDetection *cd, DistanceMetric *dm, const CFG &_c1, const CFG &_c2, vector<Cfg*> &neighbors) {
  double minDistance=MAXFLOAT;
  int retPosition=0;
  double value = 0;
  for(int i=0;i<neighbors.size();i++) {
    value=dm->Distance(_env,*neighbors[i],_c2);
    if (value<minDistance) {
      retPosition=i;
      minDistance=value;
    }    
  }
  return retPosition;
}


//
// AStarClearance

template <class CFG, class WEIGHT>
class AStarClearance: public AStar<CFG, WEIGHT> {
 public:
  AStarClearance();
  virtual ~AStarClearance();

  virtual char* GetName() const;
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual int ChooseOptimalNeighbor(Environment *_env, Stat_Class& Stats,
				    CollisionDetection *cd,
				    DistanceMetric *dm, 
				    const CFG &_c1, const CFG &_c2,
				    vector<Cfg*> &neighbors); 
};

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::
AStarClearance() : AStar<CFG, WEIGHT>() {

}

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::
~AStarClearance() {

}

template <class CFG, class WEIGHT>
char*
AStarClearance<CFG, WEIGHT>::
GetName() const {
  return "a_star_clearance";
}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
AStarClearance<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new AStarClearance<CFG, WEIGHT>(*this);
  return _copy;
}

//find Cfg with largest clearance. ASTAR_CLEARANCE
template <class CFG, class WEIGHT>
int
AStarClearance<CFG, WEIGHT>::
ChooseOptimalNeighbor(Environment *_env, Stat_Class& Stats, CollisionDetection *cd, DistanceMetric *dm, const CFG &_c1, const CFG &_c2, vector<Cfg*> &neighbors) {
  double maxClearance=-MAXFLOAT;
  int retPosition=0;
  double value = 0;
  for(int i=0;i<neighbors.size();i++) {
    value=neighbors[i]->Clearance(_env,Stats,cd);
    if (value>maxClearance) {
      retPosition=i;
      maxClearance=value;
    }   
  }
  return retPosition;
}

#endif
