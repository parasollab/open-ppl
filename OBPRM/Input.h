// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Input.h
//
//  General Description
//      This set of classes supports a command line interface and 
//	environment reading file.
//
//      The classes in the set are:
//        o param<TYPE>       -- abstract class (single value w/ default)
//        o num_param<TYPE>   -- implements a single numerical value 
//				 adhering to a [min,max] range
//        o str_param<TYPE>   -- implements a single value w/ a few
//				 more methods to check directory paths etc
//                               acknowledges 1 field from argv
//        o n_str_param<TYPE> -- implements a char* value
//                               acknowledges n fields from argv
//
//  Created
//     10/24/98  Lucia K. Dale
//
/////////////////////////////////////////////////////////////////////

#ifndef Input_h
#define Input_h

#ifdef _WIN32
  #include <iostream.h>
#endif

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

// Format version for environment (*.env) files
//      The number breaks down as YearMonthDay so numerical
//      comparisons can be more easily made.
// Warning: Be consistent.  It should be YYYYMMDD
//      Inconsistent conversions can be misleading.
//      For example, comparing 200083  to 20000604.
#define ENV_VER_LEGACY                     0
#define ENV_VER_20001022                   20001022

class Environment;

class BadUsage{};

//----------------------------------------
//  abstract parameter (pure virtual)
//----------------------------------------
template <class TYPE> class param {
public:
        param ();
        param (char *_flag);

        char* GetFlag();
        TYPE GetDefault();

        void PutDesc(                 char *_desc);
        void PutDesc(char *_typedesc, char *_desc);
        char* GetDesc();
        char* GetTypeDesc();

        TYPE GetValue();
        void PutValue(TYPE _val);

        bool IsActivated(void);
        void PrintUsage(ostream& _os,int width=18);
        bool AckCmdLine(int *i, int argc, char** argv, bool nfields=false);
protected:
        virtual void SetValue(TYPE _val)=0;
        virtual void SetDefault(TYPE _val)=0;
        virtual bool VerifyValidValue(TYPE _val)=0;

        TYPE tvalue, tdefault;
        char flag[80];
        bool activated;
        char *typedesc;
        char *desc;
};

//----------------------------------------
//  numeric parameter (ie, w/ value range)
//----------------------------------------
template<class TYPE>
class num_param : public param <TYPE> {
public:
        num_param ();
        num_param(char *_flag, TYPE _default, TYPE _min, TYPE _max);
        void PrintUsage(ostream& _os,int width=18);
protected:
        void SetValue(TYPE _val);
        void SetDefault(TYPE _val);
        bool VerifyValidValue(TYPE _val);

        TYPE rangeMin,rangeMax;
};


//----------------------------------------
//  string (mostly) parameter ( 1 field acknowledged from argv )
//----------------------------------------
template<class TYPE>
class str_param : public param <TYPE> {
public:
        str_param();
        str_param(char *_flag);
        str_param(char *_flag, char *_initialValue);
        void VerifyValidDirName();
protected:
        void SetValue(TYPE _val);
        void SetDefault(TYPE _val);
        bool VerifyValidValue(TYPE _val);

        void VerifyLastCharIsA(char *ch);
};

class n_str_param : public str_param <char*> {
public:
        n_str_param();
                //:str_param<char*>(){};
        n_str_param(char *_flag);
                //:str_param<char*>(_flag){};
        n_str_param(char *_flag,char* _initialValue);
                //:str_param<char*>(_flag,_initialValue){};
        bool AckCmdLine(int *i, int argc, char** argv);
protected:
        bool VerifyValidValue(char* _val);
};


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
    static bool VerifyFileExists(char *_fname);

    void Read();
    void Read(istream & _is, int envFormatVersion);
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
private:

};

template<class TYPE> param<TYPE>::
param(){
    activated = false;
    desc = NULL;
    typedesc = NULL;
};
template<class TYPE> param<TYPE>::
param(char *_flag){
    strcpy(flag, _flag);
    activated = false;
    desc = NULL;
    typedesc = NULL;
};
template<class TYPE> char* param<TYPE>::
GetFlag(){
    return flag;
};
template<class TYPE> TYPE param<TYPE>::
GetDefault(){
    return tdefault;
};
template<class TYPE> void param<TYPE>::
PutDesc(char *_desc){
	desc=strdup(_desc);
	typedesc=strdup("LKD-VALUE");
};
template<class TYPE> void param<TYPE>::
PutDesc(char *_typedesc,char *_desc){
    desc=strdup(_desc);
    typedesc=strdup(_typedesc);
};
template<class TYPE> char* param<TYPE>::
GetDesc(){
    return desc;
};
template<class TYPE> char* param<TYPE>::
GetTypeDesc(){
    return typedesc;
};
template<class TYPE> TYPE param<TYPE>::
GetValue(){
    return tvalue;
};
template<class TYPE> void param<TYPE>::
PutValue(TYPE _val){
    if (VerifyValidValue(_val))
        SetValue(_val);
    else
        cout << "\n       Value is NOT changed.\n";
};
template<class TYPE> bool param<TYPE>::
IsActivated(void){
    return activated;
};
template<class TYPE> void param<TYPE>::
PrintUsage(ostream& _os, int width){
    _os << setw(width) << flag ;
    _os << GetTypeDesc();
    _os << GetDesc();
};
template<class TYPE> bool param<TYPE>::
AckCmdLine(int *i, int argc, char** argv,bool nfields){

    if (  strlen(flag)==strlen(argv[*i]) &&
        !strncmp(argv[*i],flag,strlen(flag))  ) {

          if (++(*i) < argc) {

                istrstream  is(argv[*i]);
                is >> tvalue;

                if (VerifyValidValue(tvalue)){
                        activated = true;
                        return true;
                } else {
                        throw BadUsage();
                }

          } else {
                cout << "\nERROR: "
                        << flag << "  missing a VALUE";
                throw BadUsage();
          }
    }
    return false;
};


//===================================================================
//  numeric parameter (ie, w/ value range)
//  Constructors  & other methods
//===================================================================
template<class TYPE> num_param<TYPE>::
num_param():param<TYPE>(){};

template<class TYPE> num_param<TYPE>::
num_param(char *_flag, TYPE _default, TYPE _min, TYPE _max)
        :param<TYPE>(_flag),
                rangeMin(_min),rangeMax(_max)
                {
                SetValue(_default);
                SetDefault(_default);
};
template<class TYPE> void num_param<TYPE>::
PrintUsage(ostream& _os, int width){
    _os << setw(width) << flag << GetTypeDesc() << " (default, " ;

    _os.setf(ios::right,ios::adjustfield);
      _os << setw(width/2) << tdefault << ")";
    _os.setf(ios::left,ios::adjustfield);

    _os << GetDesc();
};
template<class TYPE> void num_param<TYPE>::
SetValue(TYPE _val){
    tvalue = _val;
};
template<class TYPE> void num_param<TYPE>::
SetDefault(TYPE _val){
    tdefault = _val;
};
template<class TYPE> bool num_param<TYPE>::
VerifyValidValue(TYPE _val){
    if ((rangeMin <= _val) && (_val <= rangeMax)) {
        return true;
    } else {
        cout << "\nERROR: "
                <<flag<<" "<<_val
                <<" is out of range ("
                << rangeMin <<","<<rangeMax<<")";
        return false;
    }
};
//===================================================================
//  string (mostly) parameter ( 1 field acknowledged from argv )
//  Constructors  & other methods
//===================================================================
template<class TYPE> str_param<TYPE>::
str_param():param<TYPE>(){
    tvalue = new char[300];
};
template<class TYPE> str_param<TYPE>::
str_param(char *_flag)
        :param<TYPE>(_flag){
    tvalue = new char[300];
    SetValue("");
};
template<class TYPE> str_param<TYPE>::
str_param(char *_flag, char *_initialValue)
        :param<TYPE>(_flag){
    tvalue = new char[300];
    SetValue(_initialValue);
};
template<class TYPE> void str_param<TYPE>::
VerifyValidDirName(){
    VerifyLastCharIsA("/");
};
template<class TYPE> void str_param<TYPE>::
SetValue(TYPE _val){
    strcpy(tvalue, _val);
};
template<class TYPE> void str_param<TYPE>::
SetDefault(TYPE _val){
    strcpy(tdefault, _val);
};
template<class TYPE> bool str_param<TYPE>::
VerifyValidValue(TYPE _val){
    return true;
};
template<class TYPE> void str_param<TYPE>::
VerifyLastCharIsA(char *ch){
    if (strlen(tvalue) > 0 )
        if ( tvalue[strlen(tvalue)-1] != *ch )
                strcat(tvalue,ch);
};



#endif
