// $Id$

/**@file QueryCmds.h
   @date 10/19/98
   @author Lucia K. Dale
*/

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

    /// INPUT  filename
    str_param<char*> mapFile;
    /// INPUT  filename
    str_param<char*> queryFile;
    /// OUTPUT filename
    str_param<char*> pathFile;

protected:
private:
};

#endif
