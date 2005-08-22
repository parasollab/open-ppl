////////////////////////////////////////////////////////////////////////////////////////////
/**@file  Parameters.h
  * This set of classes supports a command line interface.
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
  *@date   8/27/98
  */
////////////////////////////////////////////////////////////////////////////////////////////
#ifndef Parameters_h
#define Parameters_h

////////////////////////////////////////////////////////////////////////////////////////////
#include <sstream>
#include "Defines.h"

////////////////////////////////////////////////////////////////////////////////////////////
class BadUsage{};

//////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//  Class param
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////
/**abstract parameter (used to be pure virtual).
  *This is the base class for all our "parameter types"
  *currently used to communicate between user and program
  *via the command line.
  */
template <class TYPE> class param {
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

        /**Default constructor.
          *Following is the status if this instance after constructed:
          * -# #activated FALSE
          * -# #desc NULL
          * -# #typedesc NULL
          */
        param ();
        /**Most used constructor specifies command line flag.
          *Following is the status if this instance after constructed:
          * -# #activated FALSE
          * -# #desc NULL
          * -# #typedesc NULL
          * -# #falg _flag
          */
        param (char *_flag);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Access Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Access Methods for various data fields */
    //@{

        ///Get #flag.
        char* GetFlag();

        ///Get #tdefault.
        TYPE GetDefault() const;

        /**Set description of this parameter, such as usage.
          *Set #desc as _desc
          */
        void PutDesc(                 char *_desc);

        /**Set description of this parameter , such as usage
          *and the type of this parameter, such as integer, string....
          *Set #desc as _desc and #typedesc as _typedesc
          */
        void PutDesc(char *_typedesc, char *_desc);

        ///Get Description about this parameter. (#desc)
        char* GetDesc() const;

        ///Get Description about the type this parameter. (#typedesc)
        char* GetTypeDesc() const;

        ///Get parameter value. (#tvalue)
        TYPE GetValue() const;

        ///Validate and set parameter value. (#tvalue)
        void PutValue(TYPE _val);

        ///Validate and set default value.
        void PutDefault(TYPE _val);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Helper Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Helper Methods */
    //@{

        /// @return True when parameter exists in current command line
        bool IsActivated(void) const;

        /// Print proper command line usage
        void PrintUsage(ostream& _os,int width=18) const;

        /** Data fields are updated according to what was specified on 
          * the command line.
          * @param i index for argv. This method will check parameter value(s)
          * starting from i and update i to the end of read value(s).
          * @param argc the length of argv
          * @param argv A list of parmeters from user input.
          *
          * @return True when parameter exists in current command line.
          */
        bool AckCmdLine(int *i, int argc, char** argv, bool nfields=false);

    //@}
	///\todo clean me up.
 virtual void SetValue(TYPE _val)=0;
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Protected Data Members and Member Functions
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
protected:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Protected Helper Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Helper Methods for verifying and accessing major data field.
      *These were originally "virtual" but Burchan changed this without 
      *explanation during the platform ports. 
      */
    //@{
          ///Abstract
         
          ///Abstract
          virtual void SetDefault(TYPE _val)=0;
          ///Abstract
          virtual bool VerifyValidValue(TYPE _val)=0;
    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Data
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    TYPE tvalue,           ///< major data field
         tdefault;         ///< major data field's default value
    char flag[80];         ///< command line flag
    bool activated;        ///< currently active on command line?
    char *typedesc;        ///< type of major data field for human eyes
    char *desc;            ///< meaning of major data field to humans
};

//////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//  Class num_param
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////
/** numeric parameter (ie, w/ value range)
  * Numeric derived parameter will have max and min values.
  */
template<class TYPE>
class num_param : public param <TYPE> {

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

        /// Default constructor.
        num_param ();

        /**Max & min & default set upon construction.
          *@see SetDefault and SetValue
          */
        num_param(char *_flag, TYPE _default, TYPE _min, TYPE _max);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Helper Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Helper Methods */
    //@{       

        /** Proper usage info for user to read...
          * @param width used to format output.
          */
        void PrintUsage(ostream& _os,int width=18) const;

    //@}
 ///Set value of this parameter (#tvalue)
	///\todo clean me up.
        void SetValue(TYPE _val);

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Protected Data Members and Member Functions
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
protected:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Protected Helper Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Helper Methods for verifying and accessing major data field
    These were originally "virtual" in the base class and *had* to be
    specified in every derived class such as this one.
    */
    //@{

       
        ///Set default value of this parameter (#tdefault)
        void SetDefault(TYPE _val);

        ///Check if given value is between #rangeMin and #rangeMax or not.
        bool VerifyValidValue(TYPE _val);
    //@}

    TYPE rangeMin,  ///< Minimum acceptable value for this parameter
         rangeMax;  ///< Maximum acceptable value for this parameter
};

//////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//  Class str_param
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////
/** string (mostly) parameter ( 1 field acknowledged from argv ) */
template<class TYPE>
class str_param : public param <TYPE> {

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

        /// Default constructor. Allocate memory.
        str_param();
        /// Construct w/ command line flag specfied.
        str_param(char *_flag);
        /// Construct w/ command line flag and initial value specfied.
        str_param(char *_flag, char *_initialValue);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Helper Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Helper Methods */
    //@{

        /// Verify value given is of a valid format for a directory
        void VerifyValidDirName();

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Protected Data Members and Member Functions
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
///\todo clean me up.
 ///Set value of this parameter (#tvalue)
        void SetValue(TYPE _val);
protected:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Protected Helper Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Helper Methods for verifying and accessing major data field
    These were originally "virtual" in the base class and *had* to be
    specified in every derived class such as this one.
    */
    //@{

       

        ///Set default value of this parameter (#tdefault)
        void SetDefault(TYPE _val);

        ///Alway return true.
        bool VerifyValidValue(TYPE _val);

    //@}

    /// Verify last character in input is as specified by "ch"
    void VerifyLastCharIsA(char *ch);
};


//////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//  Class str_param
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////
class n_str_param : public str_param <char*> {

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

        /// Default constructor.
        n_str_param();

        /// Construct w/ command line flag and initial value specfied.
        n_str_param(char *_flag);

        /// Verify value given is of a valid format for a directory
        n_str_param(char *_flag,char* _initialValue);

	n_str_param(const n_str_param& tmp);
    //@}

    /** Data fields are updated according to what was specified on 
        the command line.
        @return True when flag & parameter values exist in command line.
    */
    bool AckCmdLine(int *i, int argc, char** argv);

    /**@name String Number.
      *      Since this derived type can have a variable number of 
      *      white-space separated substrings, these methods will
      *      allow you to keep track of how many.  This is useful
      *      for some validation procedures.  For example when 
      *      exactly "x" substrings must be present.  
      */
    //@{

          ///Get number of substrings. (#numStrings)
          int GetNumStrings() const;

          ///Set number of substrings. (#numStrings)
          void PutNumStrings(int _n);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //  Protected Data Members and Member Functions
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
protected:

    /**Validate this paramete's value
      *@return True if this is not a empty string. Otherwise false will be returned.
      */
    bool VerifyValidValue(char* _val);

    int numStrings; ///< How many substrings are in the value of this parameter.
};


///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  Template implementations of param Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

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
GetDefault() const {
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
GetDesc() const {
    return desc;
};
template<class TYPE> char* param<TYPE>::
GetTypeDesc() const {
    return typedesc;
};
template<class TYPE> TYPE param<TYPE>::
GetValue() const {
    return tvalue;
};
template<class TYPE> void param<TYPE>::
PutValue(TYPE _val){
    if (VerifyValidValue(_val))
        SetValue(_val);
    else
        cout << "\n       Value is NOT changed.\n";
};
template<class TYPE> void param<TYPE>::
PutDefault(TYPE _val){
    if (VerifyValidValue(_val))
        SetDefault(_val);
    else
        cout << "\n       Default is NOT changed.\n";
};
template<class TYPE> bool param<TYPE>::
IsActivated(void) const {
    return activated;
};
template<class TYPE> void param<TYPE>::
PrintUsage(ostream& _os, int width) const {
    _os << setw(width) << flag ;
    _os << GetTypeDesc();
    _os << GetDesc();
};

template<class TYPE> bool param<TYPE>::
AckCmdLine(int *i, int argc, char** argv,bool nfields){

    if (  strlen(flag)==strlen(argv[*i]) &&
        !strncmp(argv[*i],flag,strlen(flag))  ) {

          if (++(*i) < argc) {

                std::istringstream  is(argv[*i]);
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


///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  Template implementations of num_param Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

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
PrintUsage(ostream& _os, int width) const {
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

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  Template implementations of str_param Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//===================================================================
//  string (mostly) parameter ( 1 field acknowledged from argv )
//  Constructors  & other methods
//===================================================================
template<class TYPE> str_param<TYPE>::
str_param():param<TYPE>(){
    tvalue = new char[300];
    SetValue("");
    tdefault = new char[300];
    SetDefault("");
};
template<class TYPE> str_param<TYPE>::
str_param(char *_flag)
        :param<TYPE>(_flag){
    tvalue = new char[300];
    SetValue("");
    tdefault = new char[300];
    SetDefault("");
};
template<class TYPE> str_param<TYPE>::
str_param(char *_flag, char *_initialValue)
        :param<TYPE>(_flag){
    tvalue = new char[300];
    SetValue(_initialValue);
    tdefault = new char[300];
    SetDefault(_initialValue);
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
