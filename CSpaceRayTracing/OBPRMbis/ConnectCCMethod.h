/* @file ConnectCCMethod.h
 *     	Base class for all the methods to connect CCs
 *	@date 01/21/03
 */

/*Called from the main function to create the connection methods from the command line and defaults*/
class ConnectCCMethodCaller {
 public:
  ConnectCCMethodCaller() {
    connection_methods_selected.clear();
    connection_methods_all.clear();
    //need to fill out the vector of connection_methods 
    //(is this general enough?)
    
  }
  ~ConnectCCMethodCaller() {
    connection_methods_selected.clear();
    connection_methods_all.clear();
  }

  //Fill connectCC_command_line_options with (tag options) from the command line
  int ReadCommandLine(int *argc, char **argv) {
    connection_methods_selected.clear();
    vector<ConnectCCMethod>::iterator current;
    for ( current = all.begin(); current != all.end(); current++ ) {
      if ( current->parseCommandLine(argc, argv) )
	connection_methods_selected.push_back(*current);
    }
    if ( connection_methods_selected.size() == 0)
      selected = ConnectCCMethod::getDefault();
  }

  void CreateConnectionMethods();

 private:
  vector<ConnectCCMethod> connection_methods_all;
  vector<ConnectCCMethod> connection_methods_selected;

  //we may want to get rid of connectCC_command_line_options
  vector<string> connectCC_command_line_options;

  //input string options are put in the following variables
  str_param<char*> defaultFile;
  str_param<char*> mapFile; //roadmap
  n_str_param option_str; //connectCC options


}


ConnectCCMethodCaller::ConnectCCMethodCaller(): 
  defaultFile("-f"), mapFile("-inmapFile"), option_str("-connectCCsMethod") {
  option_str.PutDesc("STRING",
		     "\n\t\t\tPick any combo: default RayTracer targetOriented 1 10000 10000"
		     "\n\t\t\t  RayTracer	STRING	INT	INT	INT	STRING INT INT (bouncingMode:targetOriented maxRays:1 maxBounces:10000 maxRayLength:10000	schedulingMode:largestToSmallest scheduleMaxSize:20 sampleMaxSize:10)"
		     "\n\t\t\t  RRTcomponents  INT INT INT (iter:10 factor:3 cc:3)"
		     );
}

ConnectCCsCmds::~ConnectCCsCmds() { }

int ConnectCCMethodCaller::ReadCommandLine(int *argc, char **argv){  
  vector<char*> cmd; 
  cmd.reserve(*argc);
  
  try {  
    if (*argc == 1)
      throw BadUsage();
    
    cmd.push_back(argv[0]);
    for (int i=1;i<*argc; ++i) {  
      //-- if present then record & keep
      if ( defaultFile.AckCmdLine(&i, *argc, argv) ){
	char tmp[80];
	strcpy(tmp, defaultFile.GetValue() ); 
	strcat(tmp,".map");
	mapFile.PutValue(tmp);
	
	cmd.push_back(argv[i-1]);
	cmd.push_back(argv[i]);
	//	-- if present then record & remove from command line
      } 
      else if ( mapFile.AckCmdLine(&i, *argc, argv) ) {
      } 
      else if ( option_str.AckCmdLine(&i, *argc, argv)) {
	;//-- if unrecognized keep	
      } 
      else {
	cmd.push_back(argv[i]);
      } //endif
      
    } //endfor i
  
    //	-- Do some clean up and final checking	  
    if ( !defaultFile.IsActivated() && !( mapFile.IsActivated() ) && !option_str.IsActivated() )
      throw BadUsage();
    
    //	-- Verify INPUT files exist
    VerifyFileExists(mapFile.GetValue(),EXIT);
  } //	endtry
  catch (BadUsage ) {
    PrintUsage(cout,argv[0]);
    exit(-1);
  } //endcatch
  
  //	-- copy (possibly) modified command line back
  for (int j=0;j<cmd.size(); ++j)
    argv[j] = cmd[j];
  *argc = cmd.size();
  
  //	-- return (possibly) modified argument count
  return (*argc);
}

void ConnectCCMethodCaller::CreateConnectionMethods() {

  istrstream input_stream(option_str.GetValue());
  string cmethod_name;
  while (input_stream >> cmethod_name) {
    if (cmethod_name == string("RayTracer")) {
      connection_methods.push_back(cmethod_name, RayTracerConnectCCMethod(input_stream)); 
    }
    else if ( (cnname == string("RRTcomponents")) || (cnname == string("RRTexpand")) ) {
      connection_methods.push_back(cmethod_name, RRTConnectCCMethod(input_stream));
    }
    else if (cnname == string("components"))  {
      connection_methods.push_back(cmethod_name, ComponentsConnectCCMethod(input_stream));
    }
    
  }

}


class ConnectCCMethod {
 public:
  ConnectCCMethod(Input *,Roadmap *, CollisionDetection*, DistanceMetric*, LocalPlanners*,ConnectMapNodes*);
  ~ConnectCCMethod();

  virtual bool PerformConnectCCs (void) = 0;
  virtual int ParseCommandLine(string options)=0;
 private:

}

class TestConnectionMethod: public ConnectCCMethod {
 public:
  TestConnectionMethod(): ConnectCCMethod { 
    //set defaults

    //parse to overwrite the defaults
    ParseCommandLine("");
  }

  int ParseCommandLine(string options) { }

  bool PerformConnectCCs (void) {

  }

}
