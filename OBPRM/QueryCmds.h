// $Id$
/////////////////////////////////////////////////////////////////////
//  QueryCmds.h
//
//  Created  10/19/98 Lucia K. Dale 
/////////////////////////////////////////////////////////////////////

#ifndef QueryCmds_h
#define QueryCmds_h

#include "Input.h"

class QueryCmds {
public:
    static const int FILENAME_LENGTH  =  80;

    //-----------------------------------------------------------
    //  Constructors and Destructor
    //-----------------------------------------------------------
    QueryCmds();
    ~QueryCmds();

    //-----------------------------------------------------------
    //  Methods
    //-----------------------------------------------------------
    int ReadCommandLine(int *argc, char **argv);
    void PrintUsage(ostream& _os,char *executablename);
    void PrintValues(ostream& _os);

    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------

    str_param<char*> defaultFile;

    str_param<char*> mapFile;      // INPUT  filename
    str_param<char*> queryFile;    // INPUT  filename
    str_param<char*> pathFile;     // OUTPUT filename

protected:
private:
};

#endif
