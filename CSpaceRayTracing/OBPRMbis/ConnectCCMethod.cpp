
#include "ConnectCCMethod.h"

////////////////////////////////////////////////////////////////////////////
// ConnectMapComponents: method definitions

ConnectMapComponents::ConnectMapComponents(): 
  default_file("-f"), map_file("-inmapFile"), options("-cComponents") {
  
  selected.clear();
  all.clear();
  //need to fill out the vector of connection_methods 
  
  TestConnectionMethod* test= new TestConnectionMethod();
  all.push_back(test);    
  
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

ConnectMapComponents::ConnectMapComponents(Input * input,Roadmap * rdmp, CollisionDetection* cd, DistanceMetric* dm, LocalPlanners* lp, ConnectMapNodes* cn) :
  default_file("-f"), map_file("-inmapFile"), options("-cComponents") {

  selected.clear();
  all.clear();
  //need to fill out the vector of connection_methods 

  TestConnectionMethod* test= new TestConnectionMethod();
  all.push_back(test);    
  
  RRTConnectionMethod* rrt = new RRTConnectionMethod(input,rdmp,cd,dm,lp,cn);
  all.push_back(rrt);

  ConnectCCsConnectionMethod* connectccs = new ConnectCCsConnectionMethod(input,rdmp,cd,dm,lp,cn);
  all.push_back(connectccs);

  RayTracerConnectionMethod* rt = new RayTracerConnectionMethod();
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
    for (int i=1;i<*argc; ++i) {  
      //-- if present then record & keep
      if ( default_file.AckCmdLine(&i, *argc, argv) ){
	char tmp[80];
	strcpy(tmp, default_file.GetValue() ); 
	strcat(tmp,".map");
	map_file.PutValue(tmp);
	
	cmd.push_back(argv[i-1]);
	cmd.push_back(argv[i]);
	//-- if present then record & remove from command line
      } 
      else if ( map_file.AckCmdLine(&i, *argc, argv) ) {
      } 
      else if ( options.AckCmdLine(&i, *argc, argv)) {
	;//-- if unrecognized keep	
      } 
      else {
	cmd.push_back(argv[i]);
      } //endif
    } //endfor i
  
    //-- Do some clean up and final checking	  
    if ( !default_file.IsActivated() && !( map_file.IsActivated() ) && !options.IsActivated() )
      throw BadUsage();
    
    //	-- Verify INPUT files exist
    VerifyFileExists(map_file.GetValue(),EXIT);
  } //endtry
  catch (BadUsage ) {
    PrintUsage(cout,argv[0]);
    exit(-1);
  } //endcatch
  
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
  
  _os << "\n\tMANDATORY:\n";
  _os << "\n\t"; default_file.PrintUsage(_os);
  _os << "\n\t\t _OR_ ";
  _os << "\n\t"; map_file.PrintUsage(_os);
  
  _os << "\n\t\t _ADDITIONAL_ ";
  _os << "\n\t"; options.PrintUsage(_os);
  
  cout.setf(ios::right,ios::adjustfield);  
  _os << "\n\n";

  //add printing usage of options for each method available

}

void ConnectMapComponents::PrintValues(ostream& _os){
  _os <<"\n"<<setw(20)<<"defaultFile"<<"\t"<<default_file.GetValue();
  _os <<"\n"<<setw(20)<<"mapFile"<<"\t"<<map_file.GetValue();
  _os <<"\n"<<setw(20)<<"cComponents"<<"\t"<<options.GetValue();
  _os << "\n\n";

  //add printing the values of the methods selected
};

void ConnectMapComponents::PrintDefaults() {
  cout <<"\n"<<setw(20)<<"defaultFile : no default string for this parameter"<<"\t"<< endl;
  cout <<"\n"<<setw(20)<<"cComponents"<<"\t"<<options.GetFlag()<<endl;
  cout << "\n\n";

  //add printing the default values for the default method
}



/////////////////////////////////////////////////////////////////////////////////
//   ComponentConnectionMethod: method definitions

//   Connection Method:  RRTConnectionMethod

 RRTConnectionMethod::RRTConnectionMethod() { 
    element_name = string ("RRTcomponents"); //in ConnectCCs there is RRTexpand
    //set defaults
    is_default = false;
    //parse to overwrite the defaults
    //ParseCommandLine(0, "");
 }

 RRTConnectionMethod::RRTConnectionMethod(Input * input,Roadmap * rmp, 
                               CollisionDetection* colldetect, 
                               DistanceMetric* distmet, LocalPlanners* localp, 
                               ConnectMapNodes* cmn){
    element_name = string ("RRTcomponents"); //in ConnectCCs there is RRTexpand
  rdmp = rmp; cd = colldetect; 
  dm = distmet; lp = localp; 
  cn = cmn;
  cn1 = new CN(0,SMALL_CC,STEP_FACTOR,ITERATIONS);
  }
 RRTConnectionMethod::~RRTConnectionMethod() { }
//../growccs.test -f narrow  -outmapFile RRTa.map \
//        -cComponents RRTcomponents 200 2000 1  
 int RRTConnectionMethod::ParseCommandLine(int *argc, char **argv, istrstream &input_stream) { 

       if ( input_stream >> iterations) { //get value, if any
          if ( iterations < 0 ) {
            cout << endl << "INVALID: iterations = " << iterations;
            exit(-1);
          } else {
            if ( input_stream >> stepFactor) { //get value, if any
               if ( stepFactor < 0 ) {
                  cout << endl << "INVALID: stepFactor = " << stepFactor;
                  exit(-1);
               } else {
                 if (input_stream >> smallcc) { //get value, if any
                    if ( smallcc < 0 ) {
                       cout << endl << "INVALID: smallcc = " << smallcc;
                       exit(-1);
                    }
                 } else {
                       smallcc = SMALL_CC;  // default
                 }
               }
            } else {
               stepFactor = STEP_FACTOR;  // default
               smallcc    = SMALL_CC;  // default
            }
          }
       } else {
          iterations = ITERATIONS;  // default
          stepFactor = STEP_FACTOR;  // default
          smallcc    = SMALL_CC;  // default
       }

       cn1 = new CN(0,smallcc,stepFactor,iterations);

 }

 void RRTConnectionMethod::SetDefault() {
    is_default = true;
 }
 void RRTConnectionMethod::ConnectComponents(/*parameters for CC connection*/) {
        //Roadmap * _rm,CollisionDetection* cd,
        //LocalPlanners* lp,DistanceMetric * dm,
        //CN& _cn, CNInfo& info
    cout << "Connecting CCs with method: RRT" << endl;
    ConnectMapNodes::ConnectNodes_RRTConnect(rdmp, &*cd, &*lp, &*dm,
    						(*cn1),(*cn).cnInfo);
 }

//   Connection Method:  ConnectCCsConnectionMethod

 ConnectCCsConnectionMethod::ConnectCCsConnectionMethod() { 
    element_name = string ("components"); //in ConnectCCs
    //set defaults
    is_default = false;
    //parse to overwrite the defaults
    //ParseCommandLine(0, "");
 }

 ConnectCCsConnectionMethod::ConnectCCsConnectionMethod(Input * input,Roadmap * rmp, 
                               CollisionDetection* colldetect, 
                               DistanceMetric* distmet, LocalPlanners* localp, 
                               ConnectMapNodes* cmn){
    element_name = string ("components"); //in ConnectCCs
  rdmp = rmp; cd = colldetect; 
  dm = distmet; lp = localp; 
  cn = cmn;
  cn1 = new CN(KPAIRS,SMALL_CC,0,0);
  }
 ConnectCCsConnectionMethod::~ConnectCCsConnectionMethod() { }
 
 int ConnectCCsConnectionMethod::ParseCommandLine(int *argc,char **argv,istrstream &input_stream){ 
    if ( input_stream >> kpairs) { //get kpairs value
       if ( kpairs < 0 ) {
         cout << endl << "INVALID: kpairs = " << kpairs;
         exit(-1);
       } else {
         if (input_stream >> smallcc) { //get smallcc value, if any
            if ( smallcc < 0 ) { 
               cout << endl << "INVALID: smallcc = " << smallcc;
               exit(-1);
            } 
          } else {
            smallcc = SMALL_CC;  // default
         }
       }
    }
    if(kpairs == 0) {  //use defaults if no kpairs: kpairs=4,smallcc=SMALL_CC
         smallcc = SMALL_CC;
         kpairs = 4;
         }
    cn1 = new CN(kpairs,smallcc,0,0);
}//end ParseCommandLine
 void ConnectCCsConnectionMethod::SetDefault() {
    is_default = true;
 }
 void ConnectCCsConnectionMethod::ConnectComponents(/*parameters for CC connection*/) {

    cout << "Connecting CCs with method: ConnectCCs" << endl;
    ConnectMapNodes::ConnectNodes_ConnectCCs(rdmp, &*cd, &*lp, &*dm,
    						(*cn1),(*cn).cnInfo);
 }

