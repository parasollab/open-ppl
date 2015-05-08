#ifndef _scheduling_mode_
#define _scheduling_mode_
//This enum should go inside the ConnectMapComponent collection
enum SCHEDULING_MODE {LARGEST_TO_SMALLEST, SMALLEST_TO_LARGEST, CLOSEST_TO_FARTHEST, FARTHEST_TO_CLOSEST};
#endif


#ifndef RAYTRACER_H
#define RAYTRACER_H
///////////////////////////////////////
// RayTracer.h
// Created
//    Traces Rays in CSpace
//////////////////////////////////////
//#include <string>

#include "Environment.h"
#include "GraphAlgo.h"

#include "RRTcomponents.h"
#include "RRTexpand.h"
#include "ConnectionMethod.h"

#include "Cfg.h"
#include "Roadmap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class RayCSpace {
 public:
  RayCSpace();
  ~RayCSpace();
  void init(CFG origin, CFG direction, CFG target); //initialize the ray
  void finish();
  void bounce(CFG direction);//Stores point in list and changes direction

  //This function has to call a collision detection routine (throug cfg)
  bool collide(Environment *env, StatClass& Stats, CollisionDetection *cd,
	       CDInfo& cdInfo, DistanceMetric *dm,
	       double maxLength, int &cd_counts);//Check for collision in the environment
  bool connectTarget (Environment *env, StatClass& Stats,
		      CollisionDetection *cd,
		      CDInfo& cdInfo, DistanceMetric *dm,
		      CFG &dir, int &cd_counts);
  bool connectTarget (Environment *env, StatClass& Stats,
		      CollisionDetection *cd,
		      CDInfo& cdInfo, DistanceMetric *dm,
		      int &cd_counts);
  double length(void); //Length of the ray, I still have to figure out the units
  void writePath(Environment *env);
  void writePathConfigurations(char output_file[80],
			       vector<CFG> path, Environment *env);
  template <class WEIGHT>
    void addRoadmapNodes(Roadmap<CFG,WEIGHT> &rdmp, StatClass& Stats,
			 LocalPlanners<CFG,WEIGHT> *lp,
			 CollisionDetection *cd, DistanceMetric *dm);
  void setTargetVector(vector<CFG> *target_vector);
 private:
  CFG direction; //direction to shoot a ray
  CFG origin; //origin of the ray
  CFG collisionConfiguration;
  CFG lastFreeConfiguration;
  double rayLength;
  int numberOfBounces;
  double collisionDistance;
  double traveledDistance;
  CFG target, reached_target;
  vector<CFG> *target_vector;
  bool using_target_vector;
  vector<CFG> path;
  //Roadmap trace;
};



//definition of RayCSpace

template <class CFG>
RayCSpace<CFG>::RayCSpace() {
}
template <class CFG>
RayCSpace<CFG>::~RayCSpace() {
}

template <class CFG>
void RayCSpace<CFG>::init(CFG origin, CFG direction, CFG target) {
  this->origin = origin;
  this->direction = direction;
  this->target = target;
  rayLength = 0;
  numberOfBounces = 0;
  // path.reserve(10000);//its not good to have this hard coded. (needed?)
  path.clear();
  path.push_back(origin);
  traveledDistance=0;
  collisionDistance=0;
  using_target_vector = false;
  target_vector = NULL;
}

template <class CFG>
void RayCSpace<CFG>::finish(void) {
  rayLength += traveledDistance;
  path.push_back(reached_target);
}

template <class CFG>
void RayCSpace<CFG>::bounce(CFG direction) {
  this->direction = direction;
  //rayLength += collisionDistance;
  //path.push_back(collisionConfiguration);//includes collisionConfiguration in list
  //origin = collisionConfiguration;
  rayLength += traveledDistance;
  path.push_back(lastFreeConfiguration);
  origin = lastFreeConfiguration;
  numberOfBounces++;
}

//Based on Cfg::ApproxCspaceClearance:
template <class CFG>
bool RayCSpace<CFG>::collide(Environment *env, StatClass& Stats,
			     CollisionDetection *cd,
			     CDInfo& cdInfo, DistanceMetric *dm,
			     double maxLength, int &cd_counts) {
  bool collision=false;

  CFG cfg = origin; //Cfg of first collision
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int n_ticks;
  double tmpDist;

  CFG dir = direction; //direction of the ray
  CFG tick = cfg;//to walk through a straight line heading direction
  CFG incr;
  incr.FindIncrement(cfg,dir,&n_ticks,positionRes,orientationRes);
  int tk = 0; // controls the number of ticks generated

  std::string Callee, Method("-RayCSpace::collide");

  std::string tmpStr = Callee+Method;
  while(!collision && (dm->Distance(env,cfg,tick) < maxLength) ) {
    lastFreeConfiguration = tick;
    tick.Increment(incr); //next configuration to check
    Callee= tick.GetName();
    if( (tick.isCollision(env,Stats,cd,cdInfo,true,&tmpStr)) ||
	!(tick.InBoundary(env)) ) {
      collisionConfiguration = tick;
      collisionDistance = dm->Distance(env, cfg, tick);
      collision = true;
    }
    tk++;// increases the tick
  }
  if (collision) {
    traveledDistance = dm->Distance(env, cfg, collisionConfiguration);
  }
  else
    traveledDistance += dm->Distance(env, cfg, tick);
  cd_counts += tk;
  return collision;
}

template <class CFG>
bool RayCSpace<CFG>::connectTarget(Environment *env, StatClass& Stats,
				   CollisionDetection *cd,
				   CDInfo& cdInfo, DistanceMetric *dm,
				   int &cd_counts) {
  if (using_target_vector && target_vector != NULL) {
    //    cout << "looking for connections with " << target_vector->size() << " confs" << endl;
    for (unsigned int i = 0; i < target_vector->size(); ++i)
      if (connectTarget(env, Stats, cd, cdInfo, dm, (*target_vector)[i], cd_counts))
	return true;
    return false;
  }
  else
    return connectTarget(env, Stats, cd, cdInfo, dm, target, cd_counts);
  return false;
}
//Based on code by Shawna (Cfg::cAproxCspaceClearance:
template <class CFG>
bool RayCSpace<CFG>::connectTarget(Environment *env, StatClass& Stats,
				   CollisionDetection *cd,
				   CDInfo& cdInfo, DistanceMetric *dm,
				   CFG &dir, int &cd_counts) {
  bool collision=false;

  CFG cfg = origin; //Cfg of first collision
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int n_ticks;
  double tmpDist;

  //  Cfg dir = target; //direction of the ray
  CFG tick = cfg;//to walk through a straight line heading direction
  CFG incr;
  incr.FindIncrement(cfg,dir,&n_ticks,positionRes,orientationRes);
  int tk = 0; // controls the number of ticks generated

  std::string Callee, Method("-RayCSpace::connectTarget");

  std::string tmpStr = Callee+Method;
  while(tk < n_ticks && !collision) {
    lastFreeConfiguration = tick;
    tick.Increment(incr); //next configuration to check
    Callee = tick.GetName();
    if( (tick.isCollision(env,Stats,cd,cdInfo,true,&tmpStr)) ||
	!(tick.InBoundary(env)) ) {
      tmpDist = dm->Distance(env, cfg, tick);//distance % the ticks
      collisionConfiguration = tick;
      collisionDistance = dm->Distance(env, cfg, tick);
      collision = true;
    }
    tk++;// increases the tick
  }
  if (collision) {
    traveledDistance = dm->Distance(env, cfg, collisionConfiguration);
  }
  else
    traveledDistance += dm->Distance(env, cfg, tick);
  if (!collision)
    reached_target = dir;
  cd_counts += tk;
  return !collision;
}

template <class CFG>
double RayCSpace<CFG>::length(void) {
  return rayLength;
}

template <class CFG>
void RayCSpace<CFG>::writePath(Environment *env) {
  writePathConfigurations("RayCSpace.path", path, env);
}

template <class CFG>
void RayCSpace<CFG>::writePathConfigurations(char output_file[80],
					vector<CFG> path, Environment *env ) {
  FILE *fp;
  if((fp = fopen(output_file,"w")) == NULL){
    printf("\n\t Can't open file %s \n",output_file);
    exit(1);
  }
  //Cfg::print_preamble_to_file(env, fp, path.size());
  fprintf(fp,"VIZMO_PATH_FILE   Path Version %d\n", PATHVER_20001125);
  fprintf(fp,"%d\n", 1);
  fprintf(fp,"%d \n", path.size());
  for(int i = 0 ; i < path.size() ; i++){
    vector<double> tmp=path[i].GetData();
    // Translate all path configurations such that their resulting
    // center of gravity is what was saved (ie, the rover original)
    for(int j=0; j<tmp.size(); ++j) {
      fprintf(fp,"%f ",tmp[j]);
    }
    if(i!=(path.size()-1))fprintf(fp,"\n");

  }
  fprintf(fp,"\n");
  fclose(fp);
}

//This works assuming that the ray is a single thread that can be stretched
template <class CFG>
template <class WEIGHT>
void RayCSpace<CFG>::addRoadmapNodes(Roadmap<CFG,WEIGHT> &rdmp, StatClass& Stats, LocalPlanners<CFG,WEIGHT> *lp, CollisionDetection *cd, DistanceMetric *dm) {
  VID current, previous;

  LPOutput<CFG,WEIGHT> lpOutput;

  previous = rdmp.m_pRoadmap->AddVertex(path[0]);
  for (int i= 1; i < path.size(); i++, previous = current) {
    current = rdmp.m_pRoadmap->AddVertex(path[i]);
    //    cout << endl << "Adding edge from " << previous << " to " << current<< endl;
    if (!rdmp.m_pRoadmap->IsEdge(path[i], path[i-1]) &&
	lp->IsConnected(rdmp.GetEnvironment(), Stats, cd, dm, path[i], path[i-1], &lpOutput, rdmp.GetEnvironment()->GetPositionRes(),  rdmp.GetEnvironment()->GetOrientationRes()) )
      rdmp.m_pRoadmap->AddEdge(previous, current, lpOutput.edge);
  }
}

template <class CFG>
void RayCSpace<CFG>::setTargetVector(vector<CFG> *target_vector) {
  this->target_vector = target_vector;
  using_target_vector = true;
}



#define MAX_BOUNCES 10000
#define MAX_RAY_LENGTH 10000
#define MAX_RAYS 1
////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class RayTracer: public ConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  RayTracer();
  //RayTracer(Roadmap<CFG,WEIGHT>*, CollisionDetection*,
  //			    DistanceMetric*, LocalPlanners<CFG,WEIGHT>*);
  ~RayTracer();

  //////////////////////
  // Access
  void SetDefault();

  // Access from this method only
  void setOptions(char* bouncing_mode, int max_rays, int max_bounces, int max_ray_length);
  void setSource(CFG configuration);
  void setTarget(CFG configuration);
  void setTargetCfgs(vector<CFG> *target_cfgs);
  void setInitialDirection();//set direction of the ray to trace
  void newDirection(/*policy*/);//set a new direction according to some policy
  bool exhausted(); //returns true if all the possibilities have been explored

  //////////////////////
  // I/O methods

  void ParseCommandLine(std::istringstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual ConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection methods
  void ConnectComponents();
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, StatClass& Stats,
			 CollisionDetection*,
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges);

  void connectCCs(SCHEDULING_MODE scheduling_mode, unsigned int schedule_max_size, unsigned int sample_max_size, StatClass& Stats);
  bool connectCCs(VID cci_id, vector<CFG> &rep_cci_cfgs, VID ccj_id, vector<CFG> &rep_ccj_cfgs, Roadmap<CFG,WEIGHT> &target_rdmp, StatClass& Stats, bool try_backwards=false);
  bool trace(Roadmap<CFG,WEIGHT> &ray_rdmp, StatClass& Stats);//trace a ray in the established direction
  bool findPath(CFG &source, CFG &target, Roadmap<CFG,WEIGHT> &ray_rdmp, StatClass& Stats);
  bool findPath(CFG &source, CFG &target, vector<CFG> *target_cfgs, Roadmap<CFG,WEIGHT> &ray_rdmp, StatClass& Stats);

 private:
  void getBoundaryCfgs(const vector<CFG> &input, vector<CFG> &output, unsigned int k); // move to ConnectionMethod base class

  //////////////////////
  // Data
  char RayTbouncingMode[100];
  int RayTmaxRays;
  int RayTmaxBounces;
  int RayTmaxRayLength;

  //The following should belong to the collection, not the ray tracer
  unsigned int SampleMaxSize;
  unsigned int ScheduleMaxSize;
  SCHEDULING_MODE SchedulingMode;

  Roadmap<CFG,WEIGHT> *rdmp;
  CollisionDetection *cd;
  DistanceMetric *dm;
  LocalPlanners<CFG,WEIGHT> *lp;

  //Data specific of RayTracer
  RayCSpace<CFG> ray; // Ray to be traced
  CFG source, target;
  vector<CFG> *target_cfgs;
  bool using_target_vector;
  CFG direction; //
  int max_rays; // Maximum number of rays to test when finding a path
  int max_bounces; // Maximum number of bounces for each ray
  int max_ray_length; // Maximum length for each ray
  int rays_tested; // Number of rays tested so far
  bool all_explored; // True if all possible rays have been traced
  enum BouncingMode {TARGET_ORIENTED, RANDOM, HEURISTIC, NORMAL};
  BouncingMode bouncing_policy; // Policy to shoot rays, possible values in setDirection
  int cd_counts;

};


/////////////////////////////////////////////////////////////////////////////////   Connection Method:  RayTracer
template <class CFG, class WEIGHT>
RayTracer<CFG,WEIGHT>::RayTracer():
  ConnectionMethod<CFG,WEIGHT>() {
  element_name = "RayTracer";
  SetDefault();
}


template <class CFG, class WEIGHT>
RayTracer<CFG,WEIGHT>::~RayTracer() {
  rdmp = NULL;
  cd = NULL;
  dm = NULL;
  lp = NULL;
}


template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::
ParseCommandLine(std::istringstream& is) {
  char c;
  SetDefault();
  char str_rd[100]; //to parse strings
  int point = is.tellg();
  try {
    if (is >> RayTbouncingMode) {
      if (strcmp(RayTbouncingMode,"targetOriented") && //RayTbouncingMode != string("targetOriented") &&
          strcmp(RayTbouncingMode,"random") && //RayTbouncingMode != string("random") &&
	  strcmp(RayTbouncingMode,"heuristic") && //RayTbouncingMode != string("heuristic") &&
	  strcmp(RayTbouncingMode,"normal") ) { //RayTbouncingMode != string("normal")) {
	//cout << "read: " << RayTbouncingMode << point << "\n";
	is.seekg(point);
	//cout << is.str() << endl;

      } else {
        c = is.peek();
          while(c == ' ' || c == '\n') {
          is.get();
          c = is.peek();
        }
        if (c >= '0' && c <= '9') {
	  if (is >> RayTmaxRays) {
	    if (RayTmaxRays < 1)
	      throw BadUsage();

            c = is.peek();
            while(c == ' ' || c == '\n') {
              is.get();
              c = is.peek();
            }
            if (c >= '0' && c <= '9') {
	      if (is >> RayTmaxBounces) {
	        if (RayTmaxBounces < 1)
	          throw BadUsage();

                c = is.peek();
                while(c == ' ' || c == '\n') {
                  is.get();
                  c = is.peek();
                }
                if (c >= '0' && c <= '9') {
	          if (is >> RayTmaxRayLength) {
	            if (RayTmaxRayLength < 1)
		      throw BadUsage();

	            if (is >> str_rd) {
		      if (!strcmp(str_rd,"largestToSmallest"))//if (str_rd == string("largestToSmallest"))
		        SchedulingMode = LARGEST_TO_SMALLEST;
		      else if (!strcmp(str_rd,"smallestToLargest"))//else if (str_rd == string("smallestToLargest"))
		        SchedulingMode = SMALLEST_TO_LARGEST;
		      else if (!strcmp(str_rd,"closestToFarthest"))//else if (str_rd == string("closestToFarthest"))
		        SchedulingMode = CLOSEST_TO_FARTHEST;
		      else if (!strcmp(str_rd,"farthestToClosest"))//else if (str_rd == string("farthestToClosest"))
		        SchedulingMode = FARTHEST_TO_CLOSEST;
		      else
		        throw BadUsage();

                      c = is.peek();
                      while(c == ' ' || c == '\n') {
                        is.get();
                        c = is.peek();
                      }
                      if (c >= '0' && c <= '9') {
		        if (is >> ScheduleMaxSize) {
		          if (ScheduleMaxSize < 1)
		            throw BadUsage();

                          c = is.peek();
                          while(c == ' ' || c == '\n') {
                            is.get();
                            c = is.peek();
                          }
                          if (c >= '0' && c <= '9') {
		            if (is >> SampleMaxSize) {
		              if (SampleMaxSize < 1)
		                throw BadUsage();
		            } else
                              throw BadUsage();
                          }

		        } else
                          throw BadUsage();
                      }

	            } else
                      throw BadUsage();

	          } else
                    throw BadUsage();
                }

	      } else
                throw BadUsage();
            }

	  } else
            throw BadUsage();
        }

      } //else paired with if checking targetOriented, heuristic, random etc...
    }

  } catch (BadUsage) {
    cerr << "Error in \'RayTracer\' parameters" << endl;
    PrintUsage(cerr);
    exit(-1);
  }
}


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>*
RayTracer<CFG,WEIGHT>::
CreateCopy() {
  RayTracer<CFG,WEIGHT>* _copy =
           new RayTracer<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void
RayTracer<CFG,WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";
  _os << "\tSTRING \tINT \tINT \tINT \tSTRING \tINT \tINT (bouncingMode:targetOriented maxRays:1 maxBounces:10000 maxRayLength:10000 \tschedulingMode:largestToSmallest scheduleMaxSize:20 sampleMaxSize:10)";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
RayTracer<CFG,WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << "bouncingMode:targetOriented, maxRays: " << RayTmaxRays
      << ", maxBounces: " << RayTmaxBounces
      << ", maxRayLength: " <<RayTmaxRayLength
      <<  "\n\tschedulingMode:largestToSmallest scheduleMaxSize: " << ScheduleMaxSize
      << ", sampleMaxSize: "<< SampleMaxSize << ")";
  _os << endl;
}


template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::SetDefault() {

  all_explored = false;
  rays_tested = 0;
  //setOptions(string("targetOriented"), 1, 10000, 10000);
  setOptions("targetOriented", 1, 10000, 10000);

  strcpy(RayTbouncingMode,"targetOriented");//string("targetOriented");
  RayTmaxRays = MAX_RAYS;
  RayTmaxBounces = MAX_BOUNCES;
  RayTmaxRayLength = MAX_RAY_LENGTH;

  SchedulingMode = LARGEST_TO_SMALLEST;
  ScheduleMaxSize = 20;
  SampleMaxSize = 10;
}


template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::ConnectComponents() {
  cout << "Connecting CCs with method: RayTracer" << endl;
  cout << "DO NOTHING" <<endl;
}


template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
		  CollisionDetection* _cd ,
		  DistanceMetric * _dm,
		  LocalPlanners<CFG,WEIGHT>* _lp,
		  bool addPartialEdge,
		  bool addAllEdges) {
  cout << "RayTracer (RayTbouncingMode=" << RayTbouncingMode
       << ", RayTmaxRays=" << RayTmaxRays
       << ", RayTmaxBounces=" << RayTmaxBounces
       << ", RayTmaxRayLength=" << RayTmaxRayLength
       << ", SchedulingMode=" << SchedulingMode
       << ", ScheduleMaxSize=" << ScheduleMaxSize
       << ", SampleMaxSize=" << SampleMaxSize
       << "): " << flush;

  rdmp = _rm;
  cd = _cd;
  dm = _dm;
  lp = _lp;
  //cout << "Connecting CCs with method: RayTracer" << endl;
  all_explored = false;
  rays_tested = 0;
  setOptions(RayTbouncingMode, RayTmaxRays, RayTmaxBounces, RayTmaxRayLength);
  connectCCs(SchedulingMode, ScheduleMaxSize, SampleMaxSize, Stats);
  rdmp = NULL;
  cd = NULL;
  dm = NULL;
  lp = NULL;
}


template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::setOptions(char* bouncing_mode, int max_rays, int max_bounces, int max_ray_length) {
  this->max_rays = max_rays;
  this->max_bounces = max_bounces;
  this->max_ray_length = max_ray_length;
  //set the bouncing policy
  if (!strcmp(bouncing_mode,"targetOriented"))//if (bouncing_mode == string("targetOriented"))
    bouncing_policy =TARGET_ORIENTED;
  else if (!strcmp(bouncing_mode,"random"))//else if (bouncing_mode == string("random"))
    bouncing_policy = RANDOM;
  else if (!strcmp(bouncing_mode,"heuristic"))//else if (bouncing_mode == string("heuristic"))
    bouncing_policy = HEURISTIC;
  else if (!strcmp(bouncing_mode,"normal"))//else if (bouncing_mode == string("normal"))
    bouncing_policy = NORMAL;
  else
    bouncing_policy = TARGET_ORIENTED;

}


// Set source of the ray (it is a configuration)
template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::setSource(CFG configuration) {
  //cout << "Setting the source of the ray\n";
  source = configuration;
}


// Set target of the ray (it is a configuration)
template <class CFG, class WEIGHT>
void  RayTracer<CFG,WEIGHT>::setTarget(CFG configuration) {
  //cout << "Setting the target (goal)\n";
  target = configuration;
}


template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::setTargetCfgs(vector<CFG> *target_cfgs) {
  using_target_vector = true;
  this->target_cfgs = target_cfgs;
}


// Set direction in which the ray to be traced is going to be
// shot for the first time, a policy can be defined to select
// how the direction is generated. In this first implementation
// of the ray tracer I am going to shut rays in the direction
// of the target and when a ray hits an object it is going to
// bounce in a random direction from it (of course this has to
// change if I want to have a coherent tracer).
template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::setInitialDirection() {
  //  cout << "Setting direction of the first ray: ";
  switch (bouncing_policy) {
  case TARGET_ORIENTED: direction = target;
    //    cout << "Direction of the target\n";
    break;
  case HEURISTIC:
    //cout << "Heuristic method.\n";
    break;
  case RANDOM:
    //cout << "Random,\n";
    break;
  case NORMAL:
    //cout << "Normal\n";
    break;
  default: direction = target;
    //cout << "Direction of the target\n";
    break;
  }
  all_explored = false;
  rays_tested = 0;

}


//This function sets a direction to trace a new ray. This direction
//is not related with bouncings at all!.
template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::newDirection() {
  //cout << "sets a new Direction according to the policy defined in setDirection\n";
  rays_tested++;
  if (rays_tested>=max_rays)
    all_explored = true;
  //by the momment this is going to be just a random direction
  direction.GetRandomCfg(rdmp->GetEnvironment());
  //cout << "new direction set" << endl;
}


template <class CFG, class WEIGHT>
bool RayTracer<CFG,WEIGHT>::exhausted() {
  //  cout << "all_explored status:" << all_explored << endl;
  return all_explored;
}


template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::connectCCs(SCHEDULING_MODE scheduling_mode, unsigned int schedule_max_size, unsigned int sample_max_size, StatClass& Stats) {

  //cout << "RayTracer: (scheduling_mode: " << scheduling_mode << "; schedule_max_size: " << schedule_max_size << "; sample_max_size: " << sample_max_size << ")"<<endl;

  cd_counts = 0;

  vector< pair<size_t,VID> > ccs; //list of CCs in the roadmap
  stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
  get_cc_stats(*(rdmp->m_pRoadmap), cmap, ccs);//get the CCs

  bool path_found = false;



  vector< pair<VID,VID> > cc_trl_schdl;
  cc_trl_schdl.clear();

//    //variables to set from the command line
//    //connectCCs option
//    unsigned int sample_max_size = 10; //number of representative elements of each cc
//    //scheduler options
//    unsigned int schedule_max_size = 20;
//    SchedulingMode scheduling_mode=LARGEST_TO_SMALLEST;

  //aditional options (not really an option, just in the back of my mind)
  bool try_backwards = false;
  RRTcomponents<CFG,WEIGHT>* rrtcomp = new RRTcomponents<CFG,WEIGHT>();
  //scheduling the order to try to connect pairs
  vector< pair<int,VID> >::iterator cci, ccj;
  switch (scheduling_mode) {
  case FARTHEST_TO_CLOSEST:
    rrtcomp->OrderCCByCloseness(rdmp, dm, ccs);
  case SMALLEST_TO_LARGEST:
    for (cci = ccs.end()-1; cci > ccs.begin() && cc_trl_schdl.size() < schedule_max_size; --cci)
      for (ccj = cci-1; ccj >= ccs.begin() && cc_trl_schdl.size() < schedule_max_size; --ccj) {
	cc_trl_schdl.push_back(pair<VID,VID>(cci->second, ccj->second));
      }
    break;
  case CLOSEST_TO_FARTHEST:
    rrtcomp->OrderCCByCloseness(rdmp, dm, ccs);
  default:
  case LARGEST_TO_SMALLEST:
    for (cci = ccs.begin(); cci+1 < ccs.end() && cc_trl_schdl.size() < schedule_max_size; ++cci)
      for (ccj = cci+1; ccj < ccs.end() && cc_trl_schdl.size() < schedule_max_size; ++ccj) {
	cc_trl_schdl.push_back(pair<VID,VID>(cci->second, ccj->second));
      }
    break;
  }
  //cout << "Size of schedule: " << cc_trl_schdl.size() << endl;

  //trying to connect ccs in order defined by scheduling process
  for (vector< pair<VID,VID> >::iterator cc_itrtr =cc_trl_schdl.begin(); cc_itrtr < cc_trl_schdl.end(); ++cc_itrtr) {
    cmap.reset();
    if (!is_same_cc(*(rdmp->m_pRoadmap),cmap, cc_itrtr->first, cc_itrtr->second)) {

      Roadmap<CFG,WEIGHT> target_rdmp;
      target_rdmp.environment = rdmp->GetEnvironment();

      vector<CFG> cci_cfgs;
      vector<VID> cci_cfgs_aux;
      CFG cci_tmp = (*(rdmp->m_pRoadmap->find_vertex(cc_itrtr->first))).property();
      cmap.reset();
      get_cc(*(rdmp->m_pRoadmap), cmap, rdmp->m_pRoadmap->GetVID(cci_tmp), cci_cfgs_aux);
      cci_cfgs = rdmp->m_pRoadmap->ConvertVIDs2Vertices(cci_cfgs_aux);
      vector<CFG> rep_cci_cfgs;
      getBoundaryCfgs(cci_cfgs, rep_cci_cfgs, sample_max_size);

      vector<CFG> ccj_cfgs;
      vector<VID> ccj_cfgs_aux;
      CFG ccj_tmp = (*(rdmp->m_pRoadmap->find_vertex(cc_itrtr->second))).property();
      cmap.reset();
      get_cc(*(rdmp->m_pRoadmap), cmap, rdmp->m_pRoadmap->GetVID(ccj_tmp), ccj_cfgs_aux);
      ccj_cfgs = rdmp->m_pRoadmap->ConvertVIDs2Vertices(ccj_cfgs_aux);
      vector<CFG> rep_ccj_cfgs;
      getBoundaryCfgs(ccj_cfgs, rep_ccj_cfgs, sample_max_size);

      connectCCs(cc_itrtr->first, rep_cci_cfgs, cc_itrtr->second, rep_ccj_cfgs, target_rdmp, Stats, try_backwards);

      vector<VID> target_rdmp_vertices;
      target_rdmp.m_pRoadmap->GetVerticesVID(target_rdmp_vertices);
      rdmp->m_pRoadmap->MergeRoadMap(target_rdmp.m_pRoadmap, target_rdmp_vertices);
      target_rdmp.environment = NULL;
      target_rdmp.m_pRoadmap = NULL;
    }
  }
  //cout << endl << "RayTracer: Number of CD tests: " << cd_counts << endl;
}


template <class CFG, class WEIGHT>
bool RayTracer<CFG,WEIGHT>::connectCCs(VID cci_id, vector<CFG> &rep_cci_cfgs, VID ccj_id, vector<CFG> &rep_ccj_cfgs, Roadmap<CFG,WEIGHT> &target_rdmp, StatClass& Stats, bool try_backwards) {
  bool path_found = false;

  //cout << endl << "!!!connecting CC " << cci_id << " To CC " << ccj_id << endl;
  //first get cfgs representative of cci and ccj
  //cout << "\trep_cci_cfgs size: " << rep_cci_cfgs.size();
  //cout << "\trep_ccj_cfgs size: " << rep_ccj_cfgs.size() << endl;

  //find paths from cci to ccj
  for (unsigned int i = 0; i < rep_cci_cfgs.size(); ++i) {
    Roadmap<CFG,WEIGHT> ray_rdmp;
    ray_rdmp.environment = rdmp->GetEnvironment();

    if (findPath(rep_cci_cfgs[i],  rep_ccj_cfgs[0], &rep_ccj_cfgs, ray_rdmp, Stats)) {
      //cout << endl << "A path was found" << endl;
      vector<VID> ray_vertices;
      ray_rdmp.m_pRoadmap->GetVerticesVID(ray_vertices);
      target_rdmp.m_pRoadmap->MergeRoadMap(ray_rdmp.m_pRoadmap, ray_vertices);
      path_found = true;
      ray_rdmp.environment = NULL;
      ray_rdmp.m_pRoadmap = NULL;
      break;
    }
    else
      //cout << endl << "No path was found"<< endl;
    ray_rdmp.environment = NULL;
    ray_rdmp.m_pRoadmap = NULL;
  }

  //now from ccj to cci
  if (!path_found && try_backwards)
    for (unsigned int i = 0; i < rep_ccj_cfgs.size(); ++i) {
      Roadmap<CFG,WEIGHT> ray_rdmp;
      ray_rdmp.environment = rdmp->GetEnvironment();
      if (findPath(rep_ccj_cfgs[i],  rep_cci_cfgs[0], &rep_cci_cfgs, ray_rdmp, Stats)) {
	//cout << "A path was found" << endl;
	vector<VID> ray_vertices;
	ray_rdmp.m_pRoadmap->GetVerticesVID(ray_vertices);
	target_rdmp.m_pRoadmap->MergeRoadMap(ray_rdmp.m_pRoadmap, ray_vertices);
	path_found = true;
	ray_rdmp.environment = NULL;
	ray_rdmp.m_pRoadmap = NULL;
	break;
      }
      else
	//cout << endl << "Bad luck, no path found the other way either" <<endl;
      ray_rdmp.environment = NULL;
      ray_rdmp.m_pRoadmap = NULL;
    }
  //

  return path_found;
}


//  bool RayTracer::trace(CollisionDetection *cd, CDInfo& cdinfo,
//  		      DistanceMetric * dm) {
template <class CFG, class WEIGHT>
bool RayTracer<CFG,WEIGHT>::trace(Roadmap<CFG,WEIGHT> &ray_rdmp, StatClass& Stats) {
  bool path_found = false; //true if a path has been found, false otherwise
  long number_bouncings = 0; //number of bouncings of the ray
  double ray_length = 0; //length of the ray (depending on metrics, I suppose)

  ray.init(source, direction, target); //initialize the ray
  if (using_target_vector)
    ray.setTargetVector(target_cfgs);
  //cout << "looking for a path with " << max_bounces << " max_bounces and " << max_ray_length << " max_ray_length " << endl;
  while (!path_found && number_bouncings < max_bounces &&
	 ray.length() < max_ray_length) {
    if (ray.connectTarget(ray_rdmp.GetEnvironment(), Stats, cd, *cdInfo, dm, cd_counts)) {
      ray.finish();
      path_found = true;
    }
    else if (ray.collide(ray_rdmp.GetEnvironment(), Stats, cd, *cdInfo, dm,max_ray_length, cd_counts)) { // if there is a collision
      //cout<< "\tif the collided object is the target's screen\n";
      //cout << "\t\tpath_found=true;\n";
      //cout << "\telse\n";
      //cout << "\t\tray.bounce(collisionPoint, newdirection);\n";
      //cout << "there was a collision, the ray is going to bounce \n";
      CFG cfg;
      cfg.GetRandomCfg(ray_rdmp.GetEnvironment());
      ray.bounce(cfg);//consider the two cases commented out
      //cout << "the ray has bounced \n";
      ++number_bouncings;
    }
    else { //the ray has gone further than the max_ray_length without colliding;
      path_found = false;
    }
  }
  if (path_found) {
    //ray.save(environment);
    //ray.addRoadmapNodes(ray_rdmp);
    //ray.writePath(environment);
  }
  ray.addRoadmapNodes(ray_rdmp, Stats, lp, cd, dm);
  return path_found;
}


template <class CFG, class WEIGHT>
bool RayTracer<CFG,WEIGHT>::findPath(CFG &source, CFG &target, vector<CFG> *target_cfgs, Roadmap<CFG,WEIGHT> &ray_rdmp, StatClass& Stats) {
  setTargetCfgs(target_cfgs);
  return findPath(source,target, ray_rdmp, Stats);
}


template <class CFG, class WEIGHT>
bool RayTracer<CFG,WEIGHT>::findPath(CFG &source, CFG &target, Roadmap<CFG,WEIGHT> &ray_rdmp, StatClass& Stats) {
  bool path_found = false;
  setSource(source);
  setTarget(target);
  setInitialDirection();

  do {
    //Trace the ray
    //cout<< "Trying new direction for ray"<<endl;
    path_found= trace(ray_rdmp, Stats);
    newDirection();
  } while (!path_found && !exhausted());
  if (path_found)
    return true;
  return false;
}


template <class CFG, class WEIGHT>
void RayTracer<CFG,WEIGHT>::getBoundaryCfgs(const vector<CFG> &input, vector<CFG> &output, unsigned int sample_max_size) {
  //get center of mass of all Cfgs in input
  CFG center_mass;
  int i=0;
  for (i = 1, center_mass = input[0]; i < input.size(); ++i)
    center_mass.add(center_mass,input[i]);
  center_mass.divide(center_mass,i);

  //copy input to distances (pair of distances to center of mass, VID)
  //vector <ConnectMapNodes<CFG,WEIGHT>::CfgDistType> distances;
  vector<pair<CFG,double> > distances;
  for (i = 0; i < input.size(); ++i) {
    CFG tmp = input[i];
    double dist = dm->Distance(rdmp->GetEnvironment(), center_mass, tmp);
    //distances.push_back(ConnectMapNodes<CFG,WEIGHT>::CfgDistType(input[i], dist));
    pair<CFG,double> tmpPair(input[i],dist);
    distances.push_back(tmpPair);
  }

  //order distances by distance to center of mass (first field of pair)
  sort(distances.begin(), distances.end(), ptr_fun(ConnectionMethod<CFG, WEIGHT>::CfgDist_Compare));

  //copy the first sample_max_size to output
  output.clear();
  int lim_inf = distances.size() - 1 - sample_max_size;
  for (i = distances.size() - 1; (i > lim_inf) && (i >= 0) ; --i)
    output.push_back(distances[i].first);
}


#endif /*_RAYTRACER_H_INCLUDED*/

