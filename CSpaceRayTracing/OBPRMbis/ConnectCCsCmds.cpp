/////////////////////////////////////////////////////////////////////
//  ConnectCCsCmds.cpp
//
//  Created  09/06/02 Marco A. Morales, based on QueryCmds.cpp
/////////////////////////////////////////////////////////////////////

#include "ConnectCCsCmds.h"

#include <iostream.h>
#include <iomanip.h>
#include <string.h>
#include <vector.h>
#include "util.h"

//===================================================================
//  Constructors and Destructor
//===================================================================
ConnectCCsCmds::ConnectCCsCmds():
  defaultFile    ("-f"),
  mapFile        ("-inmapFile"),
  //pathFile	       ("-pathFile"),
  //queryFile      ("-queryFile")
  option_str("-connectCCsMethod") {
  option_str.PutDesc("STRING",
      "\n\t\t\tPick any combo: default RayTracer targetOriented 1 10000 10000"
		     "\n\t\t\t  RayTracer	STRING	INT	INT	INT	STRING INT INT (bouncingMode:targetOriented maxRays:1 maxBounces:10000 maxRayLength:10000	schedulingMode:largestToSmallest scheduleMaxSize:20 sampleMaxSize:10)"
      "\n\t\t\t  RRTcomponents  INT INT INT (iter:10 factor:3 cc:3)"
		     );
}

ConnectCCsCmds::~ConnectCCsCmds() {
}


//===================================================================
int
ConnectCCsCmds::
ReadCommandLine(int *argc, char **argv){
  
  vector<char*> cmd; cmd.reserve(*argc);
  
  try {
    
    if (*argc == 1)
      throw BadUsage();
    
    cmd.push_back(argv[0]);
    for (int i=1;i<*argc; ++i) {
      
      
      //-- if present then record & keep
      
      if ( defaultFile.AckCmdLine(&i, *argc, argv) ){
	char tmp[80];
	//      		      strcpy(tmp, defaultFile.GetValue() ); strcat(tmp,".path");
	//            pathFile.P		utValue(tmp);
	strcpy(tmp, defaultFile.GetValue() ); strcat(tmp,".map");
	mapFile.PutValue(tmp);
	//            strcpy(tmp, defa	ultFile.GetValue() ); strcat(tmp,".query");
	//            queryFile.PutValue(tmp);	
	
	cmd.push_back(argv[i-1]);
	cmd.push_back(argv[i]);
	
	//	-- if present then record & remove from command line
	
      } 
      else if ( mapFile.AckCmdLine(&i, *argc, argv) ) {
	//    	      } else if ( pathFile.AckCmdLine(&i, *argc, argv) ) {
	//          } else if ( queryFile.AckCmdLine(&i, *argc, argv) ) {
      } 
      else if ( option_str.AckCmdLine(&i, *argc, argv)) {
	//-- if unrecognized keep	
	;
      } 
      else {
	cmd.push_back(argv[i]);
      } //endif
      
    } //endfor i
    
    //	-- Do some clean up and final checking
    
    if ( !defaultFile.IsActivated() 
	 &&
	 !( mapFile.IsActivated() 
	    //  	    &	& 
	    //  		pathFile.IsActiv	ated() &&
	    //  		queryFile.IsActivated()  )	
	    )
	 && !option_str.IsActivated()
	 )
      throw BadUsage();
    
    //	-- Verify INPUT files exist
    
    VerifyFileExists(mapFile.GetValue(),EXIT);
    //      	VerifyFileExists(queryFile.GetValue(),EXIT);
    
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
  
};

void
ConnectCCsCmds::
PrintUsage(ostream& _os,char *executablename){

        _os << "\nUsage: " << executablename << " [-flag options]\n";
        _os << "\n    Available flags & options are:\n";

    cout.setf(ios::left,ios::adjustfield);

        _os << "\n    MANDATORY:\n";
        _os << "\n\t"; defaultFile.PrintUsage(_os);
        _os << "\n\t\t _OR_ ";
        _os << "\n\t"; mapFile.PrintUsage(_os);
//          _os << "\n\t"; pathFile.PrintUsage(_os);
//          _os << "\n\t"; queryFile.PrintUsage(_os);

        _os << "\n\t\t _ADDITIONAL_ ";
	_os << "\n\t"; option_str.PrintUsage(_os);

    cout.setf(ios::right,ios::adjustfield);

        _os << "\n\n";

};

void
ConnectCCsCmds::
PrintValues(ostream& _os){

  _os <<"\n"<<setw(20)<<"defaultFile"<<"\t"<<defaultFile.GetValue();
  _os <<"\n"<<setw(20)<<"mapFile"<<"\t"<<mapFile.GetValue();
//    _os <<"\n"<<setw(20)<<"pathFile"<<"\t"<<pathFile.GetValue();
//    _os <<"\n"<<setw(20)<<"queryFile"<<"\t"<<queryFile.GetValue();
  _os <<"\n"<<setw(20)<<"connectCCsMethod"<<"\t"<<option_str.GetValue();
  _os << "\n\n";
};

void ConnectCCsCmds::PrintDefaults() {
  cout <<"\n"<<setw(20)<<"defaultFile : no default string for this parameter"<<"\t"<< endl;
  cout <<"\n"<<setw(20)<<"connectCCsMethod"<<"\t"<<option_str.GetFlag()<<endl;
  cout << "\n\n";

}
