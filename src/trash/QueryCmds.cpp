// $Id$
/////////////////////////////////////////////////////////////////////
//  QueryCmds.c
//
//  Created  10/19/98 Lucia K. Dale
/////////////////////////////////////////////////////////////////////

#include "QueryCmds.h"

#include "util.h"

//===================================================================
//  Constructors and Destructor
//===================================================================
QueryCmds::QueryCmds():
        defaultFile    ("-f"),

        mapFile        ("-inmapFile"),
        pathFile       ("-pathFile"),
        queryFile      ("-queryFile")
	{
}

QueryCmds::~QueryCmds() {
}


//===================================================================
int
QueryCmds::
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
          strcpy(tmp, defaultFile.GetValue() ); strcat(tmp,".path");
          pathFile.PutValue(tmp);
          strcpy(tmp, defaultFile.GetValue() ); strcat(tmp,".map");
          mapFile.PutValue(tmp);
          strcpy(tmp, defaultFile.GetValue() ); strcat(tmp,".query");
          queryFile.PutValue(tmp);

          cmd.push_back(argv[i-1]);
          cmd.push_back(argv[i]);

      //-- if present then record & remove from command line

        } else if ( mapFile.AckCmdLine(&i, *argc, argv) ) {
        } else if ( pathFile.AckCmdLine(&i, *argc, argv) ) {
        } else if ( queryFile.AckCmdLine(&i, *argc, argv) ) {

      //-- if unrecognized keep

        } else {
		cmd.push_back(argv[i]);
        } //endif

    } //endfor i

    //-- Do some clean up and final checking

    if ( !defaultFile.IsActivated() 
		&&
         !( mapFile.IsActivated() && 
		pathFile.IsActivated() &&
		queryFile.IsActivated()  )
       )
        throw BadUsage();

    //-- Verify INPUT files exist

    VerifyFileExists(mapFile.GetValue(),PMPL_EXIT);
    VerifyFileExists(queryFile.GetValue(),PMPL_EXIT);

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

};

void
QueryCmds::
PrintUsage(ostream& _os,char *executablename){

        _os << "\nUsage: " << executablename << " [-flag options]\n";
        _os << "\n    Available flags & options are:\n";

    cout.setf(ios::left,ios::adjustfield);

        _os << "\n    MANDATORY:\n";
        _os << "\n\t"; defaultFile.PrintUsage(_os);
        _os << "\n\t\t _OR_ ";
        _os << "\n\t"; mapFile.PrintUsage(_os);
        _os << "\n\t"; pathFile.PrintUsage(_os);
        _os << "\n\t"; queryFile.PrintUsage(_os);

    cout.setf(ios::right,ios::adjustfield);

        _os << "\n\n";

};

void
QueryCmds::
PrintValues(ostream& _os){

  _os <<"\n"<<setw(20)<<"defaultFile"<<"\t"<<defaultFile.GetValue();
  _os <<"\n"<<setw(20)<<"mapFile"<<"\t"<<mapFile.GetValue();
  _os <<"\n"<<setw(20)<<"pathFile"<<"\t"<<pathFile.GetValue();
  _os <<"\n"<<setw(20)<<"queryFile"<<"\t"<<queryFile.GetValue();

  _os << "\n\n";
};
