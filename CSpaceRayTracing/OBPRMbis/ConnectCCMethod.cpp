#include "ConnectCCMethod.h"


//Command line examples:
//../growccs.test -f narrow  -outmapFile RRTa.map -cComponents RRTcomponents 200 2000 1  
//../growccs.test -f narrow -outmapFile narrow.ray -cComponents RayTracer targetOriented 1 10 10000 largestToSmallest 10 10


////////////////////////////////////////////////////////////////////////////
// ConnectMapComponents: method definitions

ConnectMapComponents::ConnectMapComponents(): options("-cComponents") {
  selected.clear();
  all.clear();
  //need to fill out the vector of connection_methods 
    
  RRTConnectionMethod* rrt = new RRTConnectionMethod();
  all.push_back(rrt);

  ConnectCCsConnectionMethod* connectccs = new ConnectCCsConnectionMethod();
  all.push_back(connectccs);
  
  RayTracerConnectionMethod* rt = new RayTracerConnectionMethod();
  all.push_back(rt);

  //Command-line-option string
  options.PutDesc("STRING",
   "\n\t\t\tPick any combo: default RayTracer targetOriented 1 10000 10000"
   "\n\t\t\t\tRayTracer \tSTRING \tINT \tINT \tINT \tSTRING \tINT \tINT (bouncingMode:targetOriented maxRays:1 maxBounces:10000 maxRayLength:10000 \tschedulingMode:largestToSmallest scheduleMaxSize:20 sampleMaxSize:10)"
   "\n\t\t\t  RRTcomponents  INT INT INT (iter:10 factor:3 cc:3)" );
}

ConnectMapComponents::ConnectMapComponents(Roadmap * rdmp, CollisionDetection* cd, DistanceMetric* dm, LocalPlanners* lp, ConnectMapNodes* cn) : 
  options("-cComponents") {
  selected.clear();
  all.clear();
  //need to fill out the vector of connection_methods 
  
  RRTConnectionMethod* rrt = new RRTConnectionMethod(rdmp,cd,dm,lp,cn);
  all.push_back(rrt);

  ConnectCCsConnectionMethod* connectccs = new ConnectCCsConnectionMethod(rdmp,cd,dm,lp,cn);
  all.push_back(connectccs);

  RayTracerConnectionMethod* rt = new RayTracerConnectionMethod(rdmp,cd,dm,lp,cn);
  all.push_back(rt);
  
  //Command-line-option string
  options.PutDesc("STRING",
   "\n\t\t\tPick any combo: default RayTracer targetOriented 1 10000 10000"
   "\n\t\t\t\tRayTracer \tSTRING \tINT \tINT \tINT \tSTRING \tINT \tINT (bouncingMode:targetOriented maxRays:1 maxBounces:10000 maxRayLength:10000 \tschedulingMode:largestToSmallest scheduleMaxSize:20 sampleMaxSize:10)"
   "\n\t\t\t  RRTcomponents  INT INT INT (iter:10 factor:3 cc:3)" );
}


ConnectMapComponents::~ConnectMapComponents() {
  selected.clear();
  all.clear();
}

int ConnectMapComponents::ReadCommandLine(int *argc, char **argv) {
  ReadCommandLineCollection(argc, argv); //Review if this is how we want it  
  selected.clear();
  
  //fill out a list of selected methods (one might be there twice)
  vector<ComponentConnectionMethod *>::iterator itr;
  
  //go through each parameter in the command line looking for method names
  istrstream input_stream(options.GetValue());
  string method_name;
  while (input_stream >> method_name) {
    for (itr = all.begin(); itr != all.end(); itr++)
      if (method_name == (*itr)->GetName()) {
	//here we'll need to change this to clone (*itr)
	(*itr)->ParseCommandLine(argc, argv, input_stream);
	selected.push_back(*itr);
      }
  }
  if ( selected.size() == 0 )
    selected = ConnectMapComponents::GetDefault();
}

int ConnectMapComponents::ReadCommandLineCollection (int *argc, char **argv) {
  vector<char*> cmd; 
  cmd.reserve(*argc);
  
  try {  
    if (*argc == 1)
      throw BadUsage();    
    cmd.push_back(argv[0]);
    for (int i = 1; i < *argc; ++i) {  
      if (!options.AckCmdLine(&i, *argc, argv))//-- if unrecognized keep
	cmd.push_back(argv[i]);
    }
    //-- Do some clean up and final checking	  
    if ( !options.IsActivated() )
      throw BadUsage();   
  }
  catch (BadUsage) {
    PrintUsage(cout,argv[0]);
    exit(-1);
  }
  
  //-- copy (possibly) modified command line back
  for (int j=0;j<cmd.size(); ++j)
    argv[j] = cmd[j];
  *argc = cmd.size();  
  //-- return (possibly) modified argument count
  return (*argc);
}

void ConnectMapComponents::PrintUsage(ostream& _os, char *executable_name) {
  _os << "\nUsage: " << executable_name << " [-flag options]\n";
  _os << "\n\tAvailable flags & options are:\n";
  cout.setf(ios::left,ios::adjustfield);  
  _os << "\n\t\t _ADDITIONAL_ ";
  _os << "\n\t"; options.PrintUsage(_os);
  cout.setf(ios::right,ios::adjustfield);  
  _os << "\n\n";

  //add printing usage of options for each method available

}

void ConnectMapComponents::PrintValues(ostream& _os){
  _os <<"\n"<<setw(20)<<"cComponents"<<"\t"<<options.GetValue();
  _os << "\n\n";

  //add printing the values of the methods selected
};

void ConnectMapComponents::PrintDefaults() {
  cout <<"\n"<<setw(20)<<"cComponents"<<"\t"<<options.GetFlag()<<endl;
  cout << "\n\n";

  //add printing the default values for the default method
}


/////////////////////////////////////////////////////////////////////////////////
//   ComponentConnectionMethod: method definitions

///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  RRTConnectionMethod

RRTConnectionMethod::RRTConnectionMethod():ComponentConnectionMethod() { 
  element_name = string ("RRTcomponents"); //in ConnectCCs there is RRTexpand
  cn1 = NULL;
  SetDefault();
}

RRTConnectionMethod::RRTConnectionMethod(Roadmap * rdmp, CollisionDetection* cd, DistanceMetric* dm, LocalPlanners* lp, ConnectMapNodes* cn):
  ComponentConnectionMethod(rdmp, cd, dm, lp, cn) {
  element_name = string ("RRTcomponents"); //in ConnectCCs there is RRTexpand
  cn1 = NULL;
  SetDefault();
}

RRTConnectionMethod::~RRTConnectionMethod() { 
  if (cn1 != NULL)
    delete cn1;
}

int RRTConnectionMethod::ParseCommandLine(int *argc, char **argv, istrstream &input_stream) { 
  int int_rd; //to parse integers
  try {
    if (input_stream >> int_rd) {
      if (int_rd < 0)
	throw BadUsage();
      iterations = int_rd;

      if (input_stream >> int_rd) {
	if (int_rd < 0)
	  throw BadUsage();
	stepFactor = int_rd;

	if (input_stream >> int_rd) {
	  if (int_rd < 0)
	    throw BadUsage();
	  smallcc = int_rd;
	}
      }
    }
  } catch (BadUsage) {
    cerr << "Error in RRT parameters" << endl;//PrintUsage(cout,...);
    exit(-1);
  }

  //update cn1 according to the current values  
  if (cn1 != NULL)
    delete cn1;
  cn1 = new CN(0,smallcc,stepFactor,iterations);
}

void RRTConnectionMethod::SetDefault() {
  iterations = ITERATIONS;
  stepFactor = STEP_FACTOR;
  smallcc    = SMALL_CC;
  if (cn1 != NULL)
    delete cn1;
  cn1 = new CN(0,smallcc,stepFactor,iterations);
}
 
void RRTConnectionMethod::ConnectComponents() {
  cout << "Connecting CCs with method: RRT" << endl;
  ConnectMapNodes::ConnectNodes_RRTConnect(rdmp, &*cd, &*lp, &*dm, *cn1,(*cn).cnInfo);
}

/////////////////////////////////////////////////////////////////////////////////   Connection Method:  ConnectCCsConnectionMethod

ConnectCCsConnectionMethod::ConnectCCsConnectionMethod():ComponentConnectionMethod() { 
  element_name = string ("components"); //in ConnectCCs
  cn1 = NULL;
  SetDefault();
}

ConnectCCsConnectionMethod::ConnectCCsConnectionMethod(Roadmap * rdmp, CollisionDetection* cd, DistanceMetric* dm, LocalPlanners* lp, ConnectMapNodes* cn):
  ComponentConnectionMethod(rdmp, cd, dm, lp, cn) {
  element_name = string ("components"); //in ConnectCCs	
  cn1 = NULL;
  SetDefault();
}
 
ConnectCCsConnectionMethod::~ConnectCCsConnectionMethod() { 
  if (cn1 != NULL)
    delete cn1;
}
 
int ConnectCCsConnectionMethod::ParseCommandLine(int *argc,char **argv,istrstream &input_stream){ 
  int int_rd; //to parse integers
  try {
    if (input_stream >> int_rd) {
      if (int_rd < 0)
	throw BadUsage();
      kpairs = int_rd;

      if (input_stream >> int_rd) {
	if (int_rd < 0)
	  throw BadUsage();
	smallcc = int_rd;
      }
    }
  } catch (BadUsage) {
    cerr << "Error in ConnectCCs parameters" << endl;//PrintUsage(cout,...);
    exit(-1);
  }

  if(kpairs == 0) {  //use defaults if no kpairs:
    SetDefault();
  } 
  else { //update cn1
    if (cn1 !=NULL)
      delete cn1;
    cn1 = new CN(kpairs,smallcc,0,0);
  }
}
 
void ConnectCCsConnectionMethod::SetDefault() {
  smallcc = SMALL_CC;
  kpairs = 4;//What is KPAIRS for then?
  if (cn1 !=NULL)
    delete cn1;
  cn1 = new CN(kpairs,smallcc,0,0);

}

void ConnectCCsConnectionMethod::ConnectComponents() {
  cout << "Connecting CCs with method: ConnectCCs" << endl;
  ConnectMapNodes::ConnectNodes_ConnectCCs(rdmp, &*cd, &*lp, &*dm,
					   (*cn1),(*cn).cnInfo);
}

/////////////////////////////////////////////////////////////////////////////////   Connection Method:  RayTracerConnectionMethod
RayTracerConnectionMethod::RayTracerConnectionMethod(): 
  ComponentConnectionMethod() { 
  element_name = string ("RayTracer");
  SetDefault();
}

RayTracerConnectionMethod::RayTracerConnectionMethod(Roadmap *rdmp, CollisionDetection *cd, DistanceMetric *dm, LocalPlanners *lp, ConnectMapNodes *cn):
  ComponentConnectionMethod(rdmp, cd, dm, lp, cn) {
  element_name = string ("RayTracer");
  SetDefault();
}

RayTracerConnectionMethod::~RayTracerConnectionMethod() { }

int RayTracerConnectionMethod::ParseCommandLine(int *argc, char **argv, istrstream &input_stream) { 
  string str_rd; //to parse strings
  int int_rd; //to parse integers
  try {
    if (input_stream >> str_rd) {
      if (str_rd != string("targetOriented") && str_rd != string("random") && 
	  str_rd != string("heuristic") && str_rd != string("normal"))
	throw BadUsage();
      RayTbouncingMode = str_rd;

      if (input_stream >> int_rd) {
	if (int_rd < 1)
	  throw BadUsage();
	RayTmaxRays = int_rd;
      
	if (input_stream >> int_rd) {
	  if (int_rd < 1)
	    throw BadUsage();
	  RayTmaxBounces = int_rd;
	  
	  if (input_stream >> int_rd) {
	    if (int_rd < 1)
	      throw BadUsage();
	    RayTmaxRayLength = int_rd;
	    
	    if (input_stream >> str_rd) {
	      if (str_rd != string("largestToSmallest") && 
		  str_rd != string("smallestToLargest") &&
		  str_rd != string("closestToFarthest") &&
		  str_rd != string("farthestToClosest"))
		throw BadUsage();
	      if (str_rd == string("largestToSmallest"))
		SchedulingMode = LARGEST_TO_SMALLEST;
	      else if (str_rd == string("smallestToLargest"))
		SchedulingMode = SMALLEST_TO_LARGEST;
	      else if (str_rd == string("closestToFarthest"))
		SchedulingMode = CLOSEST_TO_FARTHEST;
	      else if (str_rd == string("farthestToClosest"))
		SchedulingMode = FARTHEST_TO_CLOSEST;
	      else 
		throw BadUsage();
	      
	      if (input_stream >> int_rd) {
		if (int_rd < 1)
		  throw BadUsage();
		ScheduleMaxSize = int_rd;
		
		if (input_stream >> int_rd) {
		  if (int_rd < 1)
		    throw BadUsage();
		  SampleMaxSize = int_rd;
		}
	      }
	    }
	  }
	}
      }
    }
  } catch (BadUsage) {
    cerr << "Error in RayTracer parameters" << endl;//PrintUsage(cout,...);
    exit(-1);
  }
}

void RayTracerConnectionMethod::SetDefault() {
  
  RayTbouncingMode = string("targetOriented");
  RayTmaxRays = MAX_RAYS;
  RayTmaxBounces = MAX_BOUNCES;
  RayTmaxRayLength = MAX_RAY_LENGTH;
  
  SchedulingMode = LARGEST_TO_SMALLEST;
  ScheduleMaxSize = 20;
  SampleMaxSize = 10;
  
  //get rid of the following two lines as soon as possible
  dmsetid = cn->cnInfo.dmsetid;
  cdsetid = cn->cnInfo.cdsetid;  
}

void RayTracerConnectionMethod::ConnectComponents() {
  cout << "Connecting CCs with method: RayTracer" << endl;
  RayTracer tracer(rdmp, cd, cdsetid, dm, dmsetid, cn);
  tracer.setOptions(RayTbouncingMode, RayTmaxRays, RayTmaxBounces, RayTmaxRayLength);
  tracer.connectCCs(SchedulingMode, ScheduleMaxSize, SampleMaxSize);
}
