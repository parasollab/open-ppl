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


// Interface for component connection methods
class ComponentConnectionMethod { //interface only
 public:
  ComponentConnectionMethod() {
  }
  ComponentConnectionMethod(Input *,Roadmap *, CollisionDetection*, DistanceMetric*, LocalPlanners*,ConnectMapNodes*);
  ~ComponentConnectionMethod(){}

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

/*class CNdata:public CN{
   CNdata():CN(){

   }
   ~CNdata();
   void setStepFactor(int step){ stepFactor = step;}
   void setIterations(int iter){ iterations = iter;}
   void setSmallCCSize(int small){ smallcc = small; }
   void setName(char* nm) { name = nm; }
};
*/
class RRTConnectionMethod: public ComponentConnectionMethod {
 public:
  RRTConnectionMethod(): ComponentConnectionMethod() { 
    //set defaults
    is_default = false;
    //parse to overwrite the defaults
    //ParseCommandLine(0, "");
  }
  RRTConnectionMethod(Input * input,Roadmap * rmp, CollisionDetection* colldetect, 
                               DistanceMetric* distmet, LocalPlanners* localp, 
                               ConnectMapNodes* cmn):ComponentConnectionMethod(){
  rdmp = rmp; cd = colldetect; 
  dm = distmet; lp = localp; 
  cn = cmn;
  cn1 = new CN(0,1,100,200);

//  strcpy(cn1.name,cnname);
//  cn1.connector = &ConnectMapNodes::ConnectNodes_RRTConnect;
//  cn1.iterations = 10;
//  cn1.stepFactor = 10;
//  cn1.smallcc    = 10;
//       cn1.cnid = AddElementToUniverse(cn1);
//       if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
//          cout << endl << "In MakeSet: couldn't change element info";
//          exit(-1);
//       }



  }
  ~RRTConnectionMethod() { }
  int ParseCommandLine(int *argc, char **argv) { }

  void SetDefault() {
    is_default = true;
  }
  void ConnectComponents(/*parameters for CC connection*/) {
        //Roadmap * _rm,CollisionDetection* cd,
        //LocalPlanners* lp,DistanceMetric * dm,
        //CN& _cn, CNInfo& info
    cout << "Connecting CCs with method: RRT" << endl;
    ConnectMapNodes::ConnectNodes_RRTConnect(rdmp, &*cd, &*lp, &*dm,
    						(*cn1),(*cn).cnInfo);
  }
 private:
  bool is_default;
CN * cn1;
Roadmap * rdmp; 
CollisionDetection* cd; 
DistanceMetric* dm; 
LocalPlanners* lp; 
ConnectMapNodes* cn;
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
  ConnectMapComponents(Input * input,Roadmap * rdmp, CollisionDetection* cd, 
                               DistanceMetric* dm, LocalPlanners* lp, 
                               ConnectMapNodes* cn){
    RRTConnectionMethod* rrt = new RRTConnectionMethod(input,rdmp,cd,dm,lp,cn);
    all.push_back(rrt);  
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

