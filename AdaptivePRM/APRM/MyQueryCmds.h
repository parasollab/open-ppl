////////////////////////////////////////////////////////////////////////////////////////////

/**@file MyQueryCmds.h
  *@date 07/23/2002
  *@author Shawna Thomas
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MyQueryCmds_h
#define MyQueryCmds_h


#include "QueryCmds.h"


class MyQueryCmds : public QueryCmds {
public:
 
  MyQueryCmds();
  ~MyQueryCmds();

  int ReadCommandLine(int *argc, char **argv);

  void PrintUsage(ostream& _os,char *executablename);

  void PrintValues(ostream& _os);

  num_param<int> checkAllNodes;
  str_param<char*> pathValidationFlag;

protected:

private:

};

#endif
