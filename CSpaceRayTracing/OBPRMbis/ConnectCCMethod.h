#ifndef _CONNECTCCMETHOD_H_INCLUDED
#define _CONNECTCCMETHOD_H_INCLUDED
/* @file ConnectCCMethod.h
 *     	Base class for all the methods to connect CCs
 *	@date 01/21/03*/

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
    rdmp = NULL;
    cd = NULL;
    dm = NULL;
    lp = NULL;
    cn = NULL;    
  }
  ComponentConnectionMethod(Roadmap *rdmp, CollisionDetection *cd, DistanceMetric *dm, LocalPlanners *lp, ConnectMapNodes *cn) {
    this->rdmp  = rdmp;
    this->cd = cd;
    this->dm = dm;
    this->lp = lp;
    this->cn = cn;
  }
  //Check whether this is safe enough to avoid memory leaks and other problems
  ~ComponentConnectionMethod(){
    rdmp = NULL;
    cd = NULL;
    dm = NULL;
    lp = NULL;
    cn = NULL;
  }
  virtual int ParseCommandLine(int *argc, char **argv, istrstream &input_stream) = 0;
  virtual void SetDefault() = 0;
  virtual void ConnectComponents() = 0;
  string GetName() { return element_name; }
 protected:
  string element_name; //the name of the method as defined in the command line

  Roadmap *rdmp;
  CollisionDetection *cd;
  DistanceMetric *dm;
  LocalPlanners *lp;
  ConnectMapNodes *cn;
};

//----------
//Ray Tracer
#define MAX_BOUNCES 10000
#define MAX_RAY_LENGTH 10000
#define MAX_RAYS 1
class RayTracerConnectionMethod: public ComponentConnectionMethod {
 public:
  RayTracerConnectionMethod();
  RayTracerConnectionMethod(Roadmap*, CollisionDetection*, DistanceMetric*, LocalPlanners*, ConnectMapNodes*);
  ~RayTracerConnectionMethod();

  int ParseCommandLine(int *argc, char **argv, istrstream &input_stream);
  void SetDefault();
  void ConnectComponents();

 private:
  string RayTbouncingMode;
  int RayTmaxRays;
  int RayTmaxBounces;
  int RayTmaxRayLength;

  //The following should belong to the collection, not the ray tracer
  unsigned int SampleMaxSize;
  unsigned int ScheduleMaxSize;
  SCHEDULING_MODE SchedulingMode;

  //Get rid of the following setids as soon as possible
  SID cdsetid;
  SID dmsetid;
};

//------------
//RRT
#define STEP_FACTOR  50        // default for rrt stepFactor
#define ITERATIONS   50        // default for rrt iterations
#define SMALL_CC      5        // default for rrt and connectCCs: smallcc size
#define KPAIRS        5        // default for connectCCs
class RRTConnectionMethod: public ComponentConnectionMethod {
 public:
  RRTConnectionMethod();
  RRTConnectionMethod(Roadmap*, CollisionDetection*, DistanceMetric*, LocalPlanners*, ConnectMapNodes*);
  ~RRTConnectionMethod();
 
  int ParseCommandLine(int *argc, char **argv, istrstream &input_stream);
  void SetDefault();
  void ConnectComponents();

 private:
  CN * cn1;
  int iterations;  // default
  int stepFactor;  // default
  int smallcc; 
};

//-----------
//ConnectCCsConnectionMethod
class ConnectCCsConnectionMethod: public ComponentConnectionMethod {
 public:
  ConnectCCsConnectionMethod();
  ConnectCCsConnectionMethod(Roadmap*, CollisionDetection*, DistanceMetric*, LocalPlanners*, ConnectMapNodes*);
  ~ConnectCCsConnectionMethod();
 
  int ParseCommandLine(int *argc, char **argv, istrstream &input_stream);
  void SetDefault();
  void ConnectComponents();

 private:
  CN * cn1;

  int kpairs;
  int smallcc; 
};

// A collection of component connection methods
class ConnectMapComponents {
 public:
  ConnectMapComponents();
  ConnectMapComponents(Roadmap*, CollisionDetection*, DistanceMetric*, LocalPlanners*, ConnectMapNodes*);
  ~ConnectMapComponents();

  static vector<ComponentConnectionMethod *> GetDefault() {
    vector<ComponentConnectionMethod *> tmp;
    
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

  n_str_param options; //component connection options
};

#endif /*_CONNECTCCMETHOD_H_INCLUDED*/
