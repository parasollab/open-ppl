////////////////////////////////////////////////////////////////////////////////////////////

/**@file ConnectCCsCmds.h
  *@date 09/06/02
  *@author Marco Morales
  * Based on QueryCmds.h
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ConnectCCsCmds_h
#define ConnectCCsCmds_h

////////////////////////////////////////////////////////////////////////////////////////////
#include "Parameters.h"

////////////////////////////////////////////////////////////////////////////////////////////
class ConnectCCsCmds {
public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor */
    //@{

        ///Default Constructor. 
        ConnectCCsCmds();

        ///Destructor. Do nothing.
        ~ConnectCCsCmds();

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  I/O (Display, Input, Output)
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O*/
    //@{

        /**Analysis command argument list.
          *This method analysis the argument list of query command
          *and fill values in to data members of this class.
          *@param argc Number of arguments in argv.
          *@param argv An array of arguments of obprm command.
          *@exception BadUsage This exception is throwed whenever bad usages is found
          *in command arguments.
          *@see param::AckCmdLine.
          */
        int ReadCommandLine(int *argc, char **argv);

        ///Ouput usage of query command.
        void PrintUsage(ostream& _os,char *executablename);

        ///Ouput values of parameters of query command.
        void PrintValues(ostream& _os);

	///Output default values of connectCCs command
	void PrintDefaults();

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Public Data
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Use one filename for roamap, query, and path. (extensions are different)
    str_param<char*> defaultFile;

    /// INPUT  filename for road map
    str_param<char*> mapFile;
    /// INPUT  filename for query configurations
    //str_param<char*> queryFile;
    /// OUTPUT filename for path(s)
    //str_param<char*> pathFile;

    // Connection method to connect CCs
    n_str_param option_str;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Protected Data members and Member Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
protected:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Private Data members and Member Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
private:
};

#endif
