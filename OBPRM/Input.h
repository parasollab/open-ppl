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
#include "Parameters.h"
#include "DHparameters.h"
#include "Vectors.h"
#include "Orientation.h"

#include "Cfg.h"

//////////////////////////////////////////////////////////////////////////////////
class Environment;

//////////////////////////////////////////////////////////////////////////////////
/**@name Format version for environment (*.env) files
  *      The number breaks down as YearMonthDay so numerical
  *      comparisons can be more easily made.
  *@warning Be consistent.  It should be YYYYMMDD
  *      Inconsistent conversions can be misleading.
  *      For example, comparing 200083  to 20000604.
  */
//@{

//defined by Xinyu Tang, 03/27/2002
//Objective: To enable the obprm to distinguish the external & internal obstacles, 
//           so it would not try to generate nodes on the surfaces of the internal
//           obstacles, which might save a lot of time for obprm;
#define ENV_VER_20020327                   20020327

#define ENV_VER_20001022                   20001022
#define ENV_VER_LEGACY                     0
//@}

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

#ifdef _WIN32
  #define FILENAME_LENGTH    80
  #define MAX_MULTIBODY      50
  #define MAX_CONNECTION     500
  #define MAX_FIXEDBODY      50
  #define MAX_FREEBODY       500
#else
  //@{
  ///Used by GMS
  static const int FILENAME_LENGTH  =  80;  // GMS stuff uses these
  static const int MAX_MULTIBODY    =  50;
  static const int MAX_CONNECTION   =  500;
  static const int MAX_FIXEDBODY    =  50;
  static const int MAX_FREEBODY     =  500;
  //@}

#endif

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
        ~Input();

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

        /**Read data from Environment file and check version.
          *This method reads data from file whose filename 
          *is saved in #envFile.
          *This method will also check if this given file is or not a valid
          *environment file. If not and action=EXIT, process will be terminated.
          *
          *@param action What action will be performed, if file processing
          *error was found.
          *@see Read(istream & , int ,int ) and ReadCfgType.
          */
        void Read(int action);

        /**Read environmental data from specified input stream.
          *Reads information about how many multibodys, how many bodys for each
          *multibody, fixed or free, positions and orientations, filenames for
          *geometric data, connection (joint) information....
          *
          *@param _is Input stream which contains enviromental data.
          */
        void Read(istream & _is, int envFormatVersion,int action);

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

        /**Ouput filenames for free bodys and fixed bodys.
          *@note for debugging purposes
          */
        void PrintBodyNames();

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

        n_str_param       collPair;             ///< Collosion Pair for OBPRM
        n_str_param       freePair;             ///< Free Pair for OBPRM

        num_param<double> posres;               ///< Position Resolution
        num_param<double> orires;               ///< Orientation Resolution

        n_str_param       bbox;                 ///< Environment Bounding Box
	n_str_param       bbox_ref;                 ///< Environment Bounding Box
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
    //  For Enviroment
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**Environment Related Data. */
    //@{

        int multibodyCount;                                          ///< How many multibodys in environment
        int BodyCount[MAX_MULTIBODY];                                ///< How many (fixed  and free) bodys for each multibody
	// addded by Xinyu Tang, 03/28/2002
	// Objective: to show whether this Obstacle is Internal;
	//            by default it's external, so they should be false;
        int bBodyInternal[MAX_MULTIBODY];                                ///< How many (fixed  and free) bodys for each multibody

        int FixedBodyCount[MAX_MULTIBODY];                           ///< How many fixed bodys for each multibody
        int FreeBodyCount[MAX_MULTIBODY];                            ///< How many free bodys for each multibody
        char *fixedbodyFileName[MAX_MULTIBODY][MAX_FIXEDBODY];       ///< Geometric data filename for each fixed body.
        char *freebodyFileName[MAX_MULTIBODY][MAX_FREEBODY];         ///< Geometric data filename for each free body.


        int connectionCount[MAX_MULTIBODY];                          ///< Number of connection (joint) for MultiBody

        int isFree[MAX_MULTIBODY][MAX_FIXEDBODY + MAX_FREEBODY];     ///< 1 if this specified Body is free. 0 is fixed.
        int BodyIndex[MAX_MULTIBODY][MAX_FIXEDBODY + MAX_FREEBODY];  ///< Index for fixed (or free) body in fixed (or free) Body's array.

        int previousBodyIndex[MAX_MULTIBODY][MAX_CONNECTION];        ///< The index for Body before this connection (joint).
        int nextBodyIndex[MAX_MULTIBODY][MAX_CONNECTION];            ///< The index for Body after this connection (joint).

        Vector3D fixedbodyPosition[MAX_MULTIBODY][MAX_CONNECTION];       ///< Position of fixed body
        Orientation fixedbodyOrientation[MAX_MULTIBODY][MAX_CONNECTION]; ///< Orientation of fixed body

        Vector3D bodyOrientation[MAX_MULTIBODY][MAX_CONNECTION];         ///< Orientation of body (for free and fixed)

        /**Not used elsewhere, only to read in something.
          *so that the previous version of .env files could still be used. 
          */
        Vector3D freebodyPosition[MAX_MULTIBODY];                        ///< Position of free body
        Orientation freebodyOrientation[MAX_MULTIBODY];                  ///< Orientation of free body

        int connectionType[MAX_MULTIBODY][MAX_CONNECTION];               ///< Which type of this join is. "Revolute" or "Prismatic"
        Vector3D transformPosition[MAX_MULTIBODY][MAX_CONNECTION];       ///< Translation Transform (from DHframe to next link's frame)
        Orientation transformOrientation[MAX_MULTIBODY][MAX_CONNECTION]; ///< Orientation Tranform (from DHframe to next link's frame)
        DHparameters dhparameters[MAX_MULTIBODY][MAX_CONNECTION];        ///< DH-Parameter for joint.
        Vector3D positionToDHFrame[MAX_MULTIBODY][MAX_CONNECTION];       ///< Translation Transform (from current link's frame to DHframe)
        Orientation orientationToDHFrame[MAX_MULTIBODY][MAX_CONNECTION]; ///< Orientation Tranform (from current link's frame to DHframe)
        vector <char *> comments[MAX_MULTIBODY][MAX_CONNECTION];         ///< A string (FixedBody or FreeBody) for each body in environment.

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
