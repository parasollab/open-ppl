/* @file ConnectCCMethod.h
 *     	Base class for all the methods to connect CCs
 *	@date 01/21/03*/

/*Called from the main function to create the connection methods from
  the command line and defaults*/

#include "Parameters.h"
#include "Roadmap.h"
#include "util.h"
#include <string>
#include "ConnectMapNodes.h"

#define STEP_FACTOR  50        // default for rrt stepFactor
#define ITERATIONS   50        // default for rrt iterations
#define SMALL_CC      5        // default for rrt and connectCCs: smallcc size
#define KPAIRS        5        // default for connectCCs
// Interface for component connection methods
class ComponentConnectionMethod { //interface only
 public:
  ComponentConnectionMethod() {
  }
  ComponentConnectionMethod(Input *,Roadmap *, CollisionDetection*, DistanceMetric*, LocalPlanners*,ConnectMapNodes*);
  ~ComponentConnectionMethod(){}

  virtual int ParseCommandLine(int *argc, char **argv, istrstream &input_stream) = 0;
  virtual void SetDefault() = 0;
  virtual void ConnectComponents(/*parameters for CC connection*/) = 0;
  string GetName() { return element_name; }
 protected:
  string element_name; //the name of the method as defined in the command line
};

// One particular component connection method: Test
class TestConnectionMethod: public ComponentConnectionMethod {
 public:
  TestConnectionMethod(): ComponentConnectionMethod() { 
    element_name = string ("test");
    //set defaults
    is_default = false;
    //parse to overwrite the defaults
    //ParseCommandLine(0, "");
  }
  int ParseCommandLine(int *argc, char **argv, istrstream &input_stream) { }

  void SetDefault() {
    is_default = true;
  }
  void ConnectComponents(/*parameters for CC connection*/) {
    cout << "Connecting CCs with method: Test" << endl;
  }
 private:
  bool is_default;
};


//the following three constants go with the ray-tracer class
const int MAX_BOUNCES = 10000;
const int MAX_RAY_LENGTH = 10000;
const int MAX_RAYS = 1;
class RayTracerConnectionMethod: public ComponentConnectionMethod {
 public:
  RayTracerConnectionMethod(): ComponentConnectionMethod() { 
    element_name = string ("RayTracer");
    //set defaults
    is_default = false;

    RayTbouncingMode = string("targetOriented");
    RayTmaxRays = MAX_RAYS;
    RayTmaxBounces = MAX_BOUNCES;
    RayTmaxRayLength = MAX_RAY_LENGTH;

    SchedulingMode = LARGEST_TO_SMALLEST;
    ScheduleMaxSize = 20;
    SampleMaxSize = 10;

    //parse to overwrite the defaults
    //ParseCommandLine(0, "");
  }

  //this function need to be fixed to only input parameters belonging to this method. At this moment it includes parameters that belong to the scheduler
  int ParseCommandLine(int *argc, char **argv, istrstream &input_stream) { 
    string SchedulingModeStr;

    if (input_stream >> RayTbouncingMode) {
      if (RayTbouncingMode != string("targetOriented") && 
	  RayTbouncingMode != string("random") && 
	  RayTbouncingMode != string("heuristic") && 
	  RayTbouncingMode != string("normal")) {
	cout << endl << "INVALID: bouncingMode = " << RayTbouncingMode;
	exit(-1);
      } else {
	if (input_stream >> RayTmaxRays) {
	  if (RayTmaxRays < 1) {
	    cout << endl << "INVALID: maxRays = " << RayTmaxRays;
	    exit(-1);
	  } else {
	    if (input_stream >> RayTmaxBounces) {
	      if (RayTmaxBounces < 1) {
		cout << endl << "INVALID: maxBounces = " << RayTmaxBounces;
		exit(-1);
	      } else {
		if (input_stream >> RayTmaxRayLength) {
		  if (RayTmaxRayLength < 1) {
		    cout << endl << "INVALID: maxRayLength = " << RayTmaxRayLength;
		    exit(-1);
		  }
		  else {
		    if (input_stream >> SchedulingModeStr) {
		      if (SchedulingModeStr != string("largestToSmallest") &&
			  SchedulingModeStr != string("smallestToLargest") &&
			  SchedulingModeStr != string("closestToFarthest") &&
			  SchedulingModeStr != string("farthestToClosest")) {
			cout << endl << "INVALID: schedulingMode = " << SchedulingMode;
			exit(-1);		      
		      } else {
			if (SchedulingModeStr == string("largestToSmallest"))
			  SchedulingMode = LARGEST_TO_SMALLEST;
			else if (SchedulingModeStr == string("smallestToLargest"))
			  SchedulingMode = SMALLEST_TO_LARGEST;
			else if (SchedulingModeStr == string("closestToFarthest"))
			  SchedulingMode = CLOSEST_TO_FARTHEST;
			else if (SchedulingModeStr == string("farthestToClosest"))
			  SchedulingMode = FARTHEST_TO_CLOSEST;
			else 
			  SchedulingMode = LARGEST_TO_SMALLEST;
			if (input_stream >> ScheduleMaxSize) {
			  if (ScheduleMaxSize < 1) {
			    cout << endl << "INVALID: scheduleMaxSize = " << ScheduleMaxSize;
			    exit(-1);
			  } else
			    if (input_stream >> SampleMaxSize) {
			      if (SampleMaxSize < 1) {
				cout << endl << "INVALID:sampleMaxSize = " << SampleMaxSize;
				exit(-1);
			      }
			      cout << "!!!!!read SAMPLEMAXSIZE"<< SampleMaxSize<<endl;
			    }
			  cout << "!!!!!read SCHEDULEMAXSIZE"<< ScheduleMaxSize<<endl;
			}
		      }
		    } 
		  }
		}
	      }
	    }
	  }
	}
      }
    }
    
  }

  void SetDefault() {
    is_default = true;
  }
  void ConnectComponents(/*parameters for CC connection*/) {
    cout << "Connecting CCs with method: RayTracer" << endl;
  }
 private:
  bool is_default;

  //RayTracer options
  string RayTbouncingMode;
  int RayTmaxRays;
  int RayTmaxBounces;
  int RayTmaxRayLength;

  //The following should belong to the collection, not the ray tracer, but 
  //at this moment we want to migrate the code to the new way, once it is
  //done, those parameters should go to the collection
  enum SCHEDULING_MODE {LARGEST_TO_SMALLEST, SMALLEST_TO_LARGEST, CLOSEST_TO_FARTHEST, FARTHEST_TO_CLOSEST};
  unsigned int SampleMaxSize;
  unsigned int ScheduleMaxSize;
  SCHEDULING_MODE SchedulingMode;


};
//RRTConnectionMethod
class RRTConnectionMethod: public ComponentConnectionMethod {
 public:
  RRTConnectionMethod();
  RRTConnectionMethod(Input * input,Roadmap * rmp, CollisionDetection* colldetect, 
                               DistanceMetric* distmet, LocalPlanners* localp, 
                               ConnectMapNodes* cmn);
  ~RRTConnectionMethod();
 
  int ParseCommandLine(int *argc, char **argv, istrstream &input_stream);
  void SetDefault();
  void ConnectComponents();

 private:
  bool is_default;
  CN * cn1;
  Roadmap * rdmp; 
  CollisionDetection* cd; 
  DistanceMetric* dm; 
  LocalPlanners* lp; 
  ConnectMapNodes* cn;
  int iterations;  // default
  int stepFactor;  // default
  int smallcc; 
};
//ConnectCCsConnectionMethod
class ConnectCCsConnectionMethod: public ComponentConnectionMethod {
 public:
  ConnectCCsConnectionMethod();
  ConnectCCsConnectionMethod(Input * input,Roadmap * rmp, CollisionDetection* colldetect, 
                               DistanceMetric* distmet, LocalPlanners* localp, 
                               ConnectMapNodes* cmn);
  ~ConnectCCsConnectionMethod();
 
  int ParseCommandLine(int *argc, char **argv, istrstream &input_stream);
  void SetDefault();
  void ConnectComponents();

 private:
  bool is_default;
  CN * cn1;
  Roadmap * rdmp; 
  CollisionDetection* cd; 
  DistanceMetric* dm; 
  LocalPlanners* lp; 
  ConnectMapNodes* cn;
  int kpairs;
  int smallcc; 
};

// A collection of component connection methods
class ConnectMapComponents {
 public:
  ConnectMapComponents();
  ConnectMapComponents(Input * input,Roadmap * rdmp, CollisionDetection* cd, 
		       DistanceMetric* dm, LocalPlanners* lp, 
		       ConnectMapNodes* cn);
  ~ConnectMapComponents();

  static vector<ComponentConnectionMethod *> GetDefault() {
    vector<ComponentConnectionMethod *> tmp;
    
    TestConnectionMethod* test = new TestConnectionMethod();
    test->SetDefault();
    tmp.push_back(test);

    RRTConnectionMethod* rrt = new RRTConnectionMethod();
    rrt->SetDefault();
    tmp.push_back(rrt);


    ConnectCCsConnectionMethod* connectCCs = new ConnectCCsConnectionMethod();
    connectCCs->SetDefault();
    tmp.push_back(connectCCs);    

    RayTracerConnectionMethod* rt = new RayTracerConnectionMethod();
    rt->SetDefault();
    tmp.push_back(rt);

    return tmp;
  }

  //Read the command line
  int ReadCommandLine(int *argc, char **argv);
  //Read general options for the collection from the command line
  int ReadCommandLineCollection(int *argc, char **argv);
  //Output usage of ConnectMapComponents collection
  void PrintUsage(ostream& _os, char *executable_name);
  //Output values of parameters of ConnectMapComponents collection
  void PrintValues(ostream& _os);
  //Output default values of ConnectMapComponents collection
  void PrintDefaults();

  void ConnectComponents(/**/) {
    vector<ComponentConnectionMethod *>::iterator itr;
    for ( itr = selected.begin(); itr != selected.end(); itr++ )
      (*itr)->ConnectComponents();
  }

 private:
  vector<ComponentConnectionMethod *> all;
  vector<ComponentConnectionMethod *> selected;

  vector<string> component_connection_command_line_options;//double check whether we want this

  

  //input string options are put in the following variables
  str_param<char*> default_file;
  str_param<char*> map_file; //roadmap
  n_str_param options; //component connection options
};

