/* @file ConnectCCMethod.h
 *     	Base class for all the methods to connect CCs
 *	@date 01/21/03*/

/*Called from the main function to create the connection methods from the command line and defaults*/

#include "Parameters.h"
#include "Roadmap.h"
#include "util.h"
#include <string>

////////////////////////////////////////////////////////////////////////////
class ConnectCCMethod { //interface only
 public:
  ConnectCCMethod() {
  }
  ConnectCCMethod(Input *,Roadmap *, CollisionDetection*, DistanceMetric*, LocalPlanners*,ConnectMapNodes*);
  ~ConnectCCMethod();

/*    virtual int ParseCommandLine(int *argc, char **argv) = 0; */
/*    virtual void SetDefault() = 0; */
/*    virtual void ConnectCCs(parameters for CC connection) = 0; */
  int ParseCommandLine(int *argc, char **argv){}
  void SetDefault(){}
  void ConnectCCs(/*parameters for CC connection*/){}

 private:
};

///////////////////////////////////////////////////////////////////////////
class TestConnectionMethod: public ConnectCCMethod {
 public:
  TestConnectionMethod(): ConnectCCMethod() { 
    //set defaults
    is_default = false;
    //parse to overwrite the defaults
    //ParseCommandLine(0, "");
  }
  int ParseCommandLine(int *argc, char **argv) { }

  void SetDefault() {
    is_default = true;
  }
  void ConnectCCs(/*parameters for CC connection*/) {

  }
 private:
  bool is_default;
};

////////////////////////////////////////////////////////////////////////////
class ConnectCCMethodCaller {
 public:
  ConnectCCMethodCaller() {
    selected.clear();
    all.clear();
    //need to fill out the vector of connection_methods 

    TestConnectionMethod test;
    all.push_back( (ConnectCCMethod) test);    
  }
  ~ConnectCCMethodCaller() {
    selected.clear();
    all.clear();
  }

  static vector<ConnectCCMethod> GetDefault() {
    vector<ConnectCCMethod> tmp;
    
    TestConnectionMethod test;
    test.SetDefault();
    tmp.push_back(test);
    return tmp;
  }

  //Fill connectCC_command_line_options with (tag options) from the command line
  int ReadCommandLine(int *argc, char **argv) {
    selected.clear();
    vector<ConnectCCMethod>::iterator current;
    for ( current = all.begin(); current != all.end(); current++ ) {
      if ( current->ParseCommandLine(argc, argv) )
	selected.push_back(*current);
    }
    if ( selected.size() == 0 )
      selected = ConnectCCMethodCaller::GetDefault();
  }

  void CreateConnectionMethods();

 private:
  vector<ConnectCCMethod> all;
  vector<ConnectCCMethod> selected;
  //we may want to get rid of connectCC_command_line_options
  vector<string> connectCC_command_line_options;

  //input string options are put in the following variables
  str_param<char*> defaultFile;
  str_param<char*> mapFile; //roadmap
  n_str_param option_str; //connectCC options
};
