/////////////////////////////////////////////////////////////////////
//  MyQueryCmds.c
//
//  Created  07/23/2002  Shawna Thomas
/////////////////////////////////////////////////////////////////////

#include "MyQueryCmds.h"

#include <iostream.h>
#include <iomanip.h>
#include <string.h>
#include <vector.h>
#include "util.h"


MyQueryCmds::MyQueryCmds() : QueryCmds(), 
                             checkAllNodes("-checkAllNodes", 1, 0, 1), 
                             pathValidationFlag("-validatePaths","sequential") {}


MyQueryCmds::~MyQueryCmds() {}


int
MyQueryCmds::
ReadCommandLine(int *argc, char **argv){

  vector<char*> cmd; cmd.reserve(*argc);

  try {

    if (*argc == 1)
      throw BadUsage();

    cmd.push_back(argv[0]);
    for (int i=1;i<*argc; ++i) {

      //-- if present then record & remove from command line

        if ( checkAllNodes.AckCmdLine(&i, *argc, argv) ) {
	} else if ( pathValidationFlag.AckCmdLine(&i, *argc, argv) ) {

      //-- if unrecognized keep

        } else {
		cmd.push_back(argv[i]);
        } //endif

    } //endfor i

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
  return QueryCmds::ReadCommandLine(argc, argv);
}


void
MyQueryCmds::
PrintUsage(ostream& _os,char *executablename){
  QueryCmds::PrintUsage(_os, executablename);

  cout.setf(ios::left,ios::adjustfield);

  _os << "\n    OPTIONAL:\n";
  _os << "\n\t"; checkAllNodes.PrintUsage(_os);
  _os << "\n\t"; pathValidationFlag.PrintUsage(_os);

  cout.setf(ios::right,ios::adjustfield);

  _os << "\n\n";
}


void
MyQueryCmds::
PrintValues(ostream& _os){
  QueryCmds::PrintValues(_os);
  _os <<"\n"<<setw(20)<<"checkAllNodes"<<"\t"<<checkAllNodes.GetValue();
  _os <<"\n"<<setw(20)<<"validatePaths"<<"\t"<<pathValidationFlag.GetValue();
  _os <<"\n\n";
}
