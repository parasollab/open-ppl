////////////////////////////////////////////////////////////////////////////////////////////
/**@file  Input.h
  * The header file for handling input of obprm.
  *
  *General Description:
  *
  * - This set of classes supports a command line interface and 
  *         environment reading file.
  *
  * - The classes in the set are:
  *     -# param<TYPE>       -- abstract class (single value w/ default)
  *     -# num_param<TYPE>   -- implements a single numerical value 
  *          adhering to a [min,max] range
  *     -# str_param<TYPE>   -- implements a single value w/ a few
  *         more methods to check directory paths etc
  *                     acknowledges 1 field from argv
  *     -# n_str_param<TYPE> -- implements a char* value
  *                     acknowledges n fields from argv
  *
  *@author  Lucia K. Dale
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Input_h
#define Input_h
////////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "OBPRMDef.h"
#include "Cfg.h"

//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//  Class Input
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////

class Input {
public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name  Constructors and Destructor*/
    //@{

        /**Default constructor.
          *This method initializes data member by setting defaul values, max/min values
          *, and descriptions for their usages.
          *Memories for environment information are also allocated here.
          *@see param, num_param, str_param, and n_str_param
          */
        Input();
        ///Destructor. 
        virtual ~Input();

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Read/Write Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Read/Write Methods*/
    //@{

        /**Analysis command argument list.
          *This method analysis the argument list of obprm command
          *and fill values in to data members of this class.
          *@param argc Number of arguments in argv.
          *@param argv An array of arguments of obprm command.
          *@exception BadUsage This exception is throwed whenever bad usages is found
          *in command arguments.
          *@see main this method is called in main.
          */
	virtual 
	void ReadCommandLine(int argc, char** argv);

        /**Do nothing (cited from Input.cpp file).
          *Read data from preamble setion and throw them away.
          *@param _myisrean The source of input.
          */
        void ReadPreamble(istream& _myistream);

        /**Write Preamble.
          *Write command argument list to given output stream.
          *@param _myostream Output stream
          */
        void WritePreamble(ostream& _myostream) const;

        /**Read filename of enviroment file to Input::envFile.
          *This method is used to read roadmap file which
          *contains filename of environment.
          *
          *@param _myisrean The source of input.
          *@sa Read
          *@sa Read(istream & _is)
          */
        void ReadEnvFile(istream& _myistream);

        /**Write filename of environment to output stream
          *This method is used to write filename of environment
          *to roadmap file which is generated accroding to this 
          *environment.
          *
          *@param _myostream Output stream
          */
        void WriteEnvFile(ostream& _myostream);
    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Print Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Print Methods.
      *Print out some usefull information, such as parameter usages.
      */
    //@{ 

        /**Print program usage.
          *This method prints out program name and its usage.
          *Actually, this method calls PrintUsage method of class data members.
          *@param _os Output stream of usage messages.
          *@param executablename The name of executable file. Ex: obprm and query.
          *@see param::Print_Usage
          */
	virtual
        void PrintUsage(ostream& _os,char *executablename);

        /**Print values of data members.
          *This method calls GetValue of data members and send these value to
          *output stream.
          *@param _os Output stream for data values.
          *@see param::GetValue
          */
	virtual
        void PrintValues(ostream& _os);

        /**Print default values of data members.
          *This method calls GetDefault of data members and send these value to
          *standard output.
          *@see param::GetDefault
          */
	virtual
        void PrintDefaults();   // for the parameter "-defaults"
    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Public DATA
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  File/Directory Information
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    /**@name File/Directory Information*/
    //@{
        str_param<char*> envFile;               ///< input environmental filename
        str_param<char*>  mapFile;              ///< output roadmap filename
        str_param<char*> inmapFile;             ///< input roadmap filename
	str_param<char*> inmapFile2;             ///< input roadmap filename2
        str_param<char*>  defaultFile;        ///< Default input and output filenames
        str_param<char*>  descDir;            ///< env desc files here (*.dat,*.g,etc)
	str_param<char*> partitionType;
	str_param<char*> integrationType;
    //@}

    /// command line arguments
    char commandLine[8*ARGSTRING_LENGTH];    // store command line

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Parameters for Motion Planning
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Parameters for Motion Planning.
      *These data members are used to configure planning
      *components, like node generator, node connector, local planner....
      */
    //{@

        num_param<int>    addPartialEdge;     ///< Add failed path to roadmap. (For node connection)
	num_param<int>    addAllEdges;        ///< Add all edges, valid or not, to roadmap. (For node connection)
	num_param<int>    seedByChunk;        ///< Seed for each chunk. (For node generation)

	num_param<long> seed;

        n_str_param       collPair;             ///< Collosion Pair for OBPRM
        n_str_param       freePair;             ///< Free Pair for OBPRM

        num_param<double> posres;               ///< Position Resolution
        num_param<double> orires;               ///< Orientation Resolution
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
	num_param<double> rdres;
	num_param<double> gamma;
#endif

        n_str_param       bbox;                 ///< Environment Bounding Box
        num_param<double> bbox_scale;           ///< Environment Bounding Box scale factor

        n_str_param      *GNstrings[MAX_GN];    ///< mapnode generators
        int numGNs;
        n_str_param      *CNstrings[MAX_CN];    ///< mapnode connectors
        int numCNs;
        n_str_param      *LPstrings[MAX_LP];    ///< local planners
        int numLPs;
        n_str_param      *CDstrings[MAX_CD];    ///< collision detectors
        int numCDs;
        n_str_param      *DMstrings[MAX_DM];    ///< distance metrics
        int numDMs;
	n_str_param *MEstrings[MAX_GN];
	int numMEs;

        /// choose collision detection (cstk or vclip) from beginning
        cd_predefined  cdtype;

        /**This is used when USE_CSTK is specified in cdtype.
          *@see Body::buildCDstructure
          */
        int nprocs;

        /**CfgManager's name.
          *Could be "Cfg_free_rigid", "Cfg_fixed_PRR", 
          *"Cfg_free_serial", "Cfg_fixed_tree".
          *@see ReadCfgType
          */
        char cfgName[100];

        /**The joints number for articulated robots.
          *stored so as to be easily used by derived class.
          *@see ReadCfgType
          */
	num_param<int> numofJoints; ///< number of joints
        bool cfgSet;

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Private Data Members and Member Functions
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
private:

};

#endif
