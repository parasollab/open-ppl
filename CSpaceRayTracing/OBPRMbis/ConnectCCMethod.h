#ifndef _CONNECTCCMETHOD_H_INCLUDED
#define _CONNECTCCMETHOD_H_INCLUDED
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
#include "RayTracer.h"

// Interface for component connection methods
class ComponentConnectionMethod { //interface only
 public:
  ComponentConnectionMethod() {
  }
  ComponentConnectionMethod(Roadmap *rdmp, CollisionDetection *cd, DistanceMetric *dm, LocalPlanners *lp, ConnectMapNodes *cn) {
    this->rdmp  = rdmp;
    this->cd = cd;
    this->dm = dm;
    this->lp = lp;
    this->cn = cn;
  }
  ~ComponentConnectionMethod(){}

  virtual int ParseCommandLine(int *argc, char **argv, istrstream &input_stream) = 0;
  virtual void SetDefault() = 0;
  virtual void ConnectComponents(/*parameters for CC connection*/) = 0;
  string GetName() { return element_name; }
 protected:
  string element_name; //the name of the method as defined in the command line

  Roadmap *rdmp;
  CollisionDetection *cd;
  DistanceMetric *dm;
  LocalPlanners *lp;
  ConnectMapNodes *cn;
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
#define MAX_BOUNCES 10000
#define MAX_RAY_LENGTH 10000
#define MAX_RAYS 1
//
class RayTracerConnectionMethod: public ComponentConnectionMethod {
 public:
  RayTracerConnectionMethod();
  RayTracerConnectionMethod(Roadmap*, CollisionDetection*, DistanceMetric*, LocalPlanners*, ConnectMapNodes*);
  //this function need to be fixed to only input parameters belonging to this method. At this moment it includes parameters that belong to the scheduler
  int ParseCommandLine(int *argc, char **argv, istrstream &input_stream);
  void SetDefault();
  void ConnectComponents(/*parameters for CC connection*/);
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
  unsigned int SampleMaxSize;
  unsigned int ScheduleMaxSize;
  SCHEDULING_MODE SchedulingMode;

  //Get rid of the following setids as soon as possible
  SID cdsetid;
  SID dmsetid;
};

//constants used in RRT
#define STEP_FACTOR  50        // default for rrt stepFactor
#define ITERATIONS   50        // default for rrt iterations
#define SMALL_CC      5        // default for rrt and connectCCs: smallcc size
#define KPAIRS        5        // default for connectCCs
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

#endif /*_CONNECTCCMETHOD_H_INCLUDED*/
