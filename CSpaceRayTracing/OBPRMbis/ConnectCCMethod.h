/* @file ConnectCCMethod.h
 *     	Base class for all the methods to connect CCs
 *	@date 01/21/03*/

/*Called from the main function to create the connection methods from
  the command line and defaults*/

#include "Parameters.h"
#include "Roadmap.h"
#include "util.h"
#include <string>


// Interface for component connection methods
class ComponentConnectionMethod { //interface only
 public:
  ComponentConnectionMethod() {
  }
  ComponentConnectionMethod(Input *,Roadmap *, CollisionDetection*, DistanceMetric*, LocalPlanners*,ConnectMapNodes*);
  ~ComponentConnectionMethod();

  virtual int ParseCommandLine(int *argc, char **argv) = 0;
  virtual void SetDefault() = 0;
  virtual void ConnectComponents(/*parameters for CC connection*/) = 0;
 private:
};

// One particular component connection method: Test
class TestConnectionMethod: public ComponentConnectionMethod {
 public:
  TestConnectionMethod(): ComponentConnectionMethod() { 
    //set defaults
    is_default = false;
    //parse to overwrite the defaults
    //ParseCommandLine(0, "");
  }
  int ParseCommandLine(int *argc, char **argv) { }

  void SetDefault() {
    is_default = true;
  }
  void ConnectComponents(/*parameters for CC connection*/) {
    cout << "Connecting CCs with method: Test" << endl;
  }
 private:
  bool is_default;
};

class RayTracerConnectionMethod: public ComponentConnectionMethod {
 public:
  RayTracerConnectionMethod(): ComponentConnectionMethod() { 
    //set defaults
    is_default = false;
    //parse to overwrite the defaults
    //ParseCommandLine(0, "");
  }
  int ParseCommandLine(int *argc, char **argv) { }

  void SetDefault() {
    is_default = true;
  }
  void ConnectComponents(/*parameters for CC connection*/) {
    cout << "Connecting CCs with method: RayTracer" << endl;
  }
 private:
  bool is_default;

};

class RRTConnectionMethod: public ComponentConnectionMethod {
 public:
  RRTConnectionMethod(): ComponentConnectionMethod() { 
    //set defaults
    is_default = false;
    //parse to overwrite the defaults
    //ParseCommandLine(0, "");
  }
  int ParseCommandLine(int *argc, char **argv) { }

  void SetDefault() {
    is_default = true;
  }
  void ConnectComponents(/*parameters for CC connection*/) {
    cout << "Connecting CCs with method: RRT" << endl;
  }
 private:
  bool is_default;

};


// A collection of component connection methods
class ConnectMapComponents {
 public:
  ConnectMapComponents() {
    selected.clear();
    all.clear();
    //need to fill out the vector of connection_methods 

    TestConnectionMethod* test= new TestConnectionMethod();
    all.push_back(test);    

    RRTConnectionMethod* rrt = new RRTConnectionMethod();
    all.push_back(rrt);

    RayTracerConnectionMethod* rt = new RayTracerConnectionMethod();
    all.push_back(rt);
  }

  ~ConnectMapComponents() {
    selected.clear();
    all.clear();
  }

  static vector<ComponentConnectionMethod *> GetDefault() {
    vector<ComponentConnectionMethod *> tmp;
    
    TestConnectionMethod* test = new TestConnectionMethod();
    test->SetDefault();
    tmp.push_back(test);

    RRTConnectionMethod* rrt = new RRTConnectionMethod();
    rrt->SetDefault();
    tmp.push_back(rrt);

    RayTracerConnectionMethod* rt = new RayTracerConnectionMethod();
    rt->SetDefault();
    tmp.push_back(rt);

    return tmp;
  }

  //Fill connectCC_command_line_options with (tag options) from the command line
  int ReadCommandLine(int *argc, char **argv) {
    selected.clear();
    vector<ComponentConnectionMethod *>::iterator itr;
    for ( itr = all.begin(); itr != all.end(); itr++ )
      if ( (*itr)->ParseCommandLine(argc, argv) )
	selected.push_back(*itr);
    if ( selected.size() == 0 )
      selected = ConnectMapComponents::GetDefault();
  }

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
  str_param<char*> defaultFile;
  str_param<char*> mapFile; //roadmap
  n_str_param option_str; //component connection options
};
