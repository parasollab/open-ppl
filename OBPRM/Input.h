// $Id$
/////////////////////////////////////////////////////////////////////
/**@file Input.h
        This set of classes supports a command line interface and 
  	environment reading file.
  
        The classes in the set are:
          o param<TYPE>       -- abstract class (single value w/ default)
          o num_param<TYPE>   -- implements a single numerical value 
  				 adhering to a [min,max] range
          o str_param<TYPE>   -- implements a single value w/ a few
  				 more methods to check directory paths etc
                                 acknowledges 1 field from argv
          o n_str_param<TYPE> -- implements a char* value
                                 acknowledges n fields from argv
  
    @author Lucia K. Dale
    @date   8/27/98
*/
/////////////////////////////////////////////////////////////////////

#ifndef Input_h
#define Input_h

#ifdef _WIN32
  #include <iostream.h>
#endif

#include "Parameters.h"
#include "DHparameters.h"
#include "Vectors.h"
#include "Orientation.h"
#include <string.h>
#ifdef _WIN32
  #include <vector.h>
  #include <strstrea.h>
#else
  #include <vector.h>
  #include <strstream.h>
#endif

#include <stdlib.h>
#include <math.h>

/** Format version for environment (*.env) files
        The number breaks down as YearMonthDay so numerical
        comparisons can be more easily made.
   @warning Be consistent.  It should be YYYYMMDD
        Inconsistent conversions can be misleading.
        For example, comparing 200083  to 20000604.
*/
#define ENV_VER_20001022                   20001022
#define ENV_VER_LEGACY                     0


class Environment;

//---------------------------
//  Input
//---------------------------
class Input {
public:
#ifdef _WIN32
  #define ARGSTRING_LENGTH  256
  #define MAX_CN             10
  #define MAX_GN             10
  #define MAX_LP             10
  #define MAX_CD             10
  #define MAX_DM             10
  #define MAX_CFG	     10
  #define FILENAME_LENGTH    80
  #define MAX_MULTIBODY      50
  #define MAX_CONNECTION     50
  #define MAX_FIXEDBODY      50
  #define MAX_FREEBODY       50
#else
  static const int ARGSTRING_LENGTH = 256;	// used by motion planning
  static const int MAX_CN           =  10;
  static const int MAX_GN           =  10;
  static const int MAX_LP           =  10;
  static const int MAX_CD           =  10;
  static const int MAX_DM           =  10;
  static const int MAX_CFG	    =  10;

  static const int FILENAME_LENGTH  =  80;	// GMS stuff uses these
  static const int MAX_MULTIBODY    =  50;
  static const int MAX_CONNECTION   =  50;
  static const int MAX_FIXEDBODY    =  50;
  static const int MAX_FREEBODY     =  50;
#endif

    //-----------------------------------------------------------
    //  Constructors and Destructor
    //-----------------------------------------------------------
    Input();
    ~Input();
    //-----------------------------------------------------------
    //  Methods
    //-----------------------------------------------------------

    void Read(int action);
    void Read(istream & _is, int envFormatVersion,int action);
    virtual void ReadCommandLine(int argc, char** argv);

    void ReadPreamble(istream& _myistream);
    void WritePreamble(ostream& _myostream) const;
    void ReadEnvFile(istream& _myistream);
    void WriteEnvFile(ostream& _myostream);

    void PrintBodyNames();
    void PrintUsage(ostream& _os,char *executablename);
    void PrintValues(ostream& _os);
    void PrintDefaults();	// for the parameter "-defaults"

    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------
public:

    str_param<char*> envFile;               // input filename
    str_param<char*>  mapFile;              // output filename
    str_param<char*> inmapFile;             // input filename

    //
    // choose collision detection (cstk or vclip) from beginning
    //
    cd_predefined  cdtype;
    int nprocs;

public:

    //
    // command line arguments
    //
    char commandLine[4*ARGSTRING_LENGTH];    // store command line

    //
    // motion planning stuff
    //
    str_param<char*>  defaultFile;
    str_param<char*>  descDir;            // env desc files here (*.dat,*.g,etc)
    num_param<int>    numShells;
    num_param<double> proportionSurface;
    num_param<int>    lineSegment;        // used for LocalPlanner.
    num_param<int>    usingClearance;
    num_param<int>    addPartialEdge;

    n_str_param       collPair;
    n_str_param       freePair;

    num_param<int>    calcClearance;

    num_param<double> posres;             // Position Resolution
    num_param<double> orires;             // Orientation Resolution

    n_str_param       bbox;               // Bounding Box
    num_param<double> bbox_scale;         // Bounding Box scale factor

    n_str_param      *GNstrings[MAX_GN];  // mapnode generators
    int numGNs;
    n_str_param      *CNstrings[MAX_CN];  // mapnode connectors
    int numCNs;
    n_str_param      *LPstrings[MAX_LP];  // local planners
    int numLPs;
    n_str_param      *CDstrings[MAX_CD];  // collision detectors
    int numCDs;
    n_str_param      *DMstrings[MAX_DM];  // distance metrics
    int numDMs;
    n_str_param      *CFGstrings[MAX_CFG];  // Cfg type
    int numCFGs;

    char cfgName[100];
    int numofJoints; // the joints number for articulated robots.
                     // stored so as to be easily used by derived class.

    //
    // data fields from 'init' file (given as command line argument)
    //
    int multibodyCount;
    int BodyCount[MAX_MULTIBODY];
    int FixedBodyCount[MAX_MULTIBODY];
    int FreeBodyCount[MAX_MULTIBODY];
    char *fixedbodyFileName[MAX_MULTIBODY][MAX_FIXEDBODY];
    char *freebodyFileName[MAX_MULTIBODY][MAX_FREEBODY];


    int connectionCount[MAX_MULTIBODY];

    int isFree[MAX_MULTIBODY][MAX_FIXEDBODY + MAX_FREEBODY];
    int BodyIndex[MAX_MULTIBODY][MAX_FIXEDBODY + MAX_FREEBODY];

    int previousBodyIndex[MAX_MULTIBODY][MAX_CONNECTION];
    int nextBodyIndex[MAX_MULTIBODY][MAX_CONNECTION];

    Vector3D fixedbodyPosition[MAX_MULTIBODY][MAX_CONNECTION];
    Orientation fixedbodyOrientation[MAX_MULTIBODY][MAX_CONNECTION];


    Vector3D bodyOrientation[MAX_MULTIBODY][MAX_CONNECTION];

    // these two arrays are not used elsewhere, only to read in something.
    // so that the previous version of .env files could still be used. 
    Vector3D freebodyPosition[MAX_MULTIBODY];
    Orientation freebodyOrientation[MAX_MULTIBODY];

    int connectionType[MAX_MULTIBODY][MAX_CONNECTION];
    Vector3D transformPosition[MAX_MULTIBODY][MAX_CONNECTION];
    Orientation transformOrientation[MAX_MULTIBODY][MAX_CONNECTION];
    DHparameters dhparameters[MAX_MULTIBODY][MAX_CONNECTION];
    Vector3D positionToDHFrame[MAX_MULTIBODY][MAX_CONNECTION];
    Orientation orientationToDHFrame[MAX_MULTIBODY][MAX_CONNECTION];
    vector <char *> comments[MAX_MULTIBODY][MAX_CONNECTION];
private:

};

#endif
