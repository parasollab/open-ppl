// $Id$
/////////////////////////////////////////////////////////////////////
/**@file Parameters.h
        This set of classes supports a command line interface. 
  
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

#ifndef Parameters_h
#define Parameters_h

#ifdef _WIN32
  #include <iostream.h>
#endif

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

class BadUsage{};

//----------------------------------------
/** abstract parameter (used to be pure virtual)
        This is the base class for all our "parameter types"
        currently used to communicate between user and program
        via the command line.
*/
//----------------------------------------
template <class TYPE> class param {
public:

  /**@name Constructors and Destructor */
  //@{
        /// Default constructor.
        param ();
        /// Most used constructor specifies command line flag.
        param (char *_flag);
  //@}

  /**@name Access Methods for various data fields */
  //@{
        char* GetFlag();
        TYPE GetDefault();

        void PutDesc(                 char *_desc);
        void PutDesc(char *_typedesc, char *_desc);
        char* GetDesc();
        char* GetTypeDesc();

        TYPE GetValue();
        void PutValue(TYPE _val);
  //@}

  /// @return True when parameter exists in current command line
        bool IsActivated(void);
  /// Print proper command line usage
        void PrintUsage(ostream& _os,int width=18);
  /** Data fields are updated according to what was specified on 
      the command line.
      @return True when parameter exists in current command line.
  */
        bool AckCmdLine(int *i, int argc, char** argv, bool nfields=false);
protected:
  /**@name Helper Methods for verifying and accessing major data field
  These were originally "virtual" but Burchan changed this without 
  explanation during the platform ports. 
  */
  //@{
        virtual void SetValue(TYPE _val)=0;
        virtual void SetDefault(TYPE _val)=0;
        virtual bool VerifyValidValue(TYPE _val)=0;
  //@}

  /**@name Data */
  //@{
        TYPE tvalue, tdefault; /// major data field and its default
        char flag[80];         /// command line flag
        bool activated;        /// currently active on command line?
        char *typedesc;        /// type of major data field for human eyes
        char *desc;            /// meaning of major data field to humans
  //@}
};

//----------------------------------------
/** numeric parameter (ie, w/ value range)
    Numeric derived parameter will have max and min values.
*/
//----------------------------------------
template<class TYPE>
class num_param : public param <TYPE> {
public:
  /**@name Constructors and Destructor */
  //@{
        /// Default constructor.
        num_param ();
        /// max & min set upon construction
        num_param(char *_flag, TYPE _default, TYPE _min, TYPE _max);
  //@}
        /// Proper usage info for user to read...
        void PrintUsage(ostream& _os,int width=18);
protected:
  /**@name Helper Methods for verifying and accessing major data field
  These were originally "virtual" in the base class and *had* to be
  specified in every derived class such as this one.
  */
  //@{
        void SetValue(TYPE _val);
        void SetDefault(TYPE _val);
        bool VerifyValidValue(TYPE _val);
  //@}

        TYPE rangeMin,rangeMax;
};


//----------------------------------------
/** string (mostly) parameter ( 1 field acknowledged from argv ) */
//----------------------------------------
template<class TYPE>
class str_param : public param <TYPE> {
public:
  /**@name Constructors and Destructor */
  //@{
        /// Default constructor.
        str_param();
        /// Construct w/ command line flag specfied.
        str_param(char *_flag);
        /// Construct w/ command line flag and initial value specfied.
        str_param(char *_flag, char *_initialValue);
  //@}
        /// Verify value given is of a valid format for a directory
        void VerifyValidDirName();
protected:
  /**@name Helper Methods for verifying and accessing major data field
  These were originally "virtual" in the base class and *had* to be
  specified in every derived class such as this one.
  */
  //@{
        void SetValue(TYPE _val);
        void SetDefault(TYPE _val);
        bool VerifyValidValue(TYPE _val);
  //@}

  /// Verify last character in input is as specified by "ch"
        void VerifyLastCharIsA(char *ch);
};

class n_str_param : public str_param <char*> {
public:
  /**@name Constructors and Destructor */
  //@{
        /// Default constructor.
        n_str_param();
                //:str_param<char*>(){};
        /// Construct w/ command line flag and initial value specfied.
        n_str_param(char *_flag);
                //:str_param<char*>(_flag){};
        /// Verify value given is of a valid format for a directory
        n_str_param(char *_flag,char* _initialValue);
                //:str_param<char*>(_flag,_initialValue){};
  //@}

  /** Data fields are updated according to what was specified on 
      the command line.
      @return True when flag & parameter values exist in command line.
  */
        bool AckCmdLine(int *i, int argc, char** argv);

  /**@name Since this derived type can have a variable number of 
           white-space separated substrings, these methods will
           allow you to keep track of how many.  This is useful
           for some validation procedures.  For example when 
           exactly "x" substrings must be present.  */
  //@{
        int GetNumStrings();
        void PutNumStrings(int _n);
  //@}

protected:
        bool VerifyValidValue(char* _val);
	int numStrings;
};


//*********************************************************
//
//      Template implementations of all the above...
//
//*********************************************************

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
