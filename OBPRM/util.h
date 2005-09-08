/**@file util.h
  *
  *This is a file containing many kinds of methods.
  *
  *@date 7/17/98
  *@author Daniel Vallejo
  */

#ifndef util_h
#define util_h

/////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers

#ifdef HPUX
#include <sys/io.h>
#endif

#include <ctype.h>

/////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "OBPRMDef.h"
#include "tinyxml.h"

/////////////////////////////////////////////////////////////////////////////////////////
class Cfg;
class Environment;
/////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////
//
//
//
// Basic Utility.
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////

/**@name Basic Utility */
//@{

/**Calculate the minimum DIRECTED angular distance 
  *between two angles normalized to 1.0 
  */
double DirectedAngularDistance(double a,double b);

///Return minimun between a and b.
inline double min(double a, double b){
    return a < b ? a : b;
}

///Return maximun between a and b.
inline double max(double a, double b){
    return a > b ? a : b;
}

/// Return the square of a.
inline double sqr(double a)
{
    return a*a;
}
//@}

/////////////////////////////////////////////////////////////////////////////////////////
//
//
//
// Random Number Generator
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////


  //return non-negative double-prevision floating-point values 
  //uniformly distributed over the interval [0.0, 1.0)
  //call drand48()
  double OBPRM_drand();
  
  //return non-negative long integers uniformly distributed over the interval [0, 2**31)
  //call lrand48()
  long OBPRM_lrand();

  //return signed long integers uniformly distributed over the interval [-2**31, 2**31)
  //call mrand48()
  long OBPRM_mrand();

  // normally(gaussian) distributed random number generator.
  // when reset is 1, it reset the internal static variable and return 0.0
  double OBPRM_grand(bool reset = false);
  
  /* use seedval as the seed
   */
  long OBPRM_srand(long seedval = 0x1234ABCD);
  
  /* "baseSeed" is a static variable in this function
     we use baseSeed, methodName and nextNodeIndex to generate a deterministic seed,
     then call seed48()
     when reset is 1, baseSeed will be reset
   */
  long OBPRM_srand(std::string methodName, int nextNodeIndex, long base = 0x1234ABCD, bool reset = false);


/////////////////////////////////////////////////////////////////////////////////////////
//
//
//
// Cfgs input & output from/to files.
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////
/**@name Cfgs I/O Utility*/
//@{

    /**Write a list of Cfgs in a give path and path size 
      *to file with given filename.
      *
      *This method writes number of links of robot,
      *path size to output_file. Then Cfgs for
      *each link in robot will be output to file 
      *named "output_file".cfg
      *
      *
      *@param filename Filename for Cfg data.
      *@param path All Gfgs will be written to file.
      *@param env is used to get "robot link" information.
      *@note if file couldn't be opened, error message will be post 
      *and process will be terminated.
      *
      *@see Cfg::printLinkConfigurations
      */
    void WritePathLinkConfigurations(char output_file[80], 
                                     vector<Cfg*>& path, 
                                     Environment *env);
    template <class CFG>
    void WritePathLinkConfigurations(char output_file[80], 
                                     vector<CFG>& path, 
                                     Environment *env);

    /**Write a list of Cfgs in a give path and path size 
      *to file with given filename.
      *@param filename Filename for Cfg data.
      *@param path All Gfgs will be written to file.
      *@param env is not used.
      *@note if file couldn't be opened, error message will be post 
      *and process will be terminated.
      */   
    void WritePathConfigurations(char output_file[80], 
                                 vector<Cfg*>& path, 
                                 Environment *env);  
    template <class CFG>
    void WritePathConfigurations(char output_file[80], 
				 vector<CFG>& path, 
                                 Environment *env);

    /**Read a list of Cfgs from file with given filename.
      *@param filename Filename for Cfg data.
      *@param cfgs All new Gfg will be inserted to this list.
      *@note if file couldn't be opened, error message will be post.
      *@see Cfg::Read
      */
    template <class CFG>
    void ReadCfgs(char *filename, vector<CFG>& cfgs);

//@}

/////////////////////////////////////////////////////////////////////////////////////////
//
//
//
// Files input & output from/to files.
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////
/**@name File I/O Utility*/
//@{

    #define EXIT 1      ///< Actions for VerifyFileExists
    #define RETURN 2    ///< Actions for VerifyFileExists

    /**Check if or not this given filename exists.
      *@param _fname File name that is going to be checked.
      *@action What should be done if file not found.
      *Its value should be EXIT or RETURN.
      *
      *@return true if file exists. If file dosen't exist 
      *and action is EXIT, process will be terminated.
      *Otherwise false will be returned.
      */
    bool VerifyFileExists(const char *_fname,int action);


    /**Read data for element from input stream.
      *This method throws away comments starting from "#", and
      *find out read data for element.
      *
      *@param element An element which will read data from file.
      *@note type T should have istream>> overloading.
      *@note >> overloading should return 0 (NULL or false)
      *if data couldn't be read correctly.
      */
    template <class T> bool readfield (istream &_is, T *element);

//@}

/////////////////////////////////////////////////////////////////////////////////////////
//
//
//
// implementation for the template function.
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////

///The character to distinguish commnets.
#define COMMENT_DELIMITER '#'
///The maximum number of characters in each line.
#define LINEMAX 256
    
template <class T> bool readfield (istream &_is, T *element,vector <char *> &comment) {
    char c;
    char ThrowAwayLine[LINEMAX];
    
    while ( _is.get(c) ) {
        if (c == '#') {
            _is.getline(ThrowAwayLine,LINEMAX,'\n');
            comment.push_back(strdup(ThrowAwayLine));
        }
        else if (! isspace(c) ) {
            _is.putback(c);
            if (_is >> *element) {
              return true;
            } else {
            break;
            }
        }
    }
    
    // could not read correctly ...
    cout << "Error in reading!!! at util::readfield. " << endl;
    return false;
}

template <class T> bool readfield (istream &_is, T *element) {
    vector <char  *> comment;
    bool ret=readfield(_is,element,comment);
    comment.clear();
    return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////
//Modified for VC
/////////////////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32

////////////////////////////////////////////////////////////////////////////////////////
// Following functions define M_PI and drand48, which are not starndard c library and 
// definitions. In addition, rint used to round off float points to int is also here.
/////////////////////////////////////////////////////////////////////////////////////////

#define M_PI PI //reference PI above

extern "C" {
//Implementation of these functions are located in util.cpp
double drand48();
double erand48(register unsigned short *xsubi);
long irand48(register unsigned short m);
long krand48(register unsigned short *xsubi, unsigned short m);
long lrand48();
long mrand48();
static void next();
void srand48(long seedval);
unsigned short * seed48(unsigned short seed16v[3]);
void lcong48(unsigned short param[7]);
long nrand48(register unsigned short *xsubi);
long jrand48(register unsigned short *xsubi);

/**Round to closest integer.
  *The rint() function rounds x to an integer value according
  *to the prevalent rounding mode.  The default rounding mode
  *is to round to the nearest integer.
  *@return The  rint() function returns the integer value as a float-
  *ing-point number.
  */
double rint(double x);

} //end extern "C"

#endif //_WIN32
/////////////////////////////////////////////////////////////////////////////////////////

template <class CFG>
void WritePathLinkConfigurations(char output_file[80], 
				 vector<CFG>& path, 
				 Environment *env) {
  vector<Cfg*> ppath;
  for(int i=0; i<path.size(); i++)
    ppath.push_back(&path[i]);
  WritePathLinkConfigurations(output_file, ppath, env);
}


template <class CFG>
void WritePathConfigurations(char output_file[80],
			     vector<CFG>& path,
			     Environment *env) {
  vector<Cfg*> ppath;
  for(int i=0; i<path.size(); i++) 
    ppath.push_back(&path[i]);
  WritePathConfigurations(output_file, ppath, env);
}


// read cfgs from a file into a vector.
template <class CFG>
void ReadCfgs(char * filename,  vector<CFG>& cfgs) {
  ifstream  is(filename);
  if (!is) {
    cout << "\nWarning: in util::ReadCfgs: can't open infile: " << filename << endl;
    return;
  }
  
  CFG tempCfg;
  while (1) {
    tempCfg.Read(is);
    if(!is) break;
    cfgs.push_back(tempCfg);
  }
}

/////////////////
//
//Logging + XML stuff
//
/////////////////
//#define DEBUG_MSG 1
//#define WARNING_MSG 2
//#define ERROR_MSG 3

#ifdef _LOG

#define LOG_DEBUG_MSG( msg ) \
{ \
        cout << "DEBUG: " << msg << endl ; \
}

#define LOG_WARNING_MSG( msg ) \
{ \
        cout << "WARNING: " << msg << endl ; \
}

#define LOG_ERROR_MSG( msg ) \
{ \
        cout << "ERROR: " << msg << endl ; \
}

#else 

#define LOG_DEBUG_MSG( msg ) { }
#define LOG_WARNING_MSG( msg ) { }
#define LOG_ERROR_MSG( msg ) { }
#endif //_LOG






class MessageLogs {

  public:
    MessageLogs() {
      bOutput = false;
      level = VERBOSE;
    };
    
    inline int GetLevel() { return level; };
    inline void SetLevel(int in_level) { level = in_level; };
    inline void operator << (ostream& io_os) { 
      if(!bOutput)
        cout << io_os << endl;;
    };
    
  private:
    int level;
    bool bOutput;
    

};

///\todo{ XML wrapper, things needed.
///isElement == name
///isChild 
///getChild
///isAttribute
///findAttribute


class MPProblem;
class MPBaseObject {

  public: 
    MPBaseObject(){ m_pProblem = NULL;};
    MPBaseObject(MPProblem* in_pProblem){ m_pProblem = in_pProblem;};
    MPBaseObject(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
         m_strLabel("") { 
      m_pProblem = in_pProblem;
      ParseXML(in_pNode); 
    };
    virtual void ParseXML(TiXmlNode* in_pNode) {
      if(in_pNode->Type() == TiXmlNode::ELEMENT) {
        const char* carLabel = in_pNode->ToElement()->Attribute("Label");
        if(carLabel) {
          m_strLabel = string(carLabel);
        }
      } 
    };
    
    inline MessageLogs& GetMessageLog() { return m_message_log; };
    inline MPProblem* GetMPProblem() { return m_pProblem;}
    virtual void PrintOptions(ostream& out_os) { };
    inline string& GetLabel() { return m_strLabel; };
  private:
    MessageLogs m_message_log;
    MPProblem* m_pProblem;
    string m_strLabel;
    /// want to add string m_strName;
    //      remove from below ... use in future
      //if(level >= GetMessageLog().GetLevel()) \

};
/*

class MPFileIO : public MPBaseObject {
public:
  MPFileIO(string& in_strFileName) {
    constructed = true;
    m_strFileName = in_strFilename;
  };
    
  MPFileIO() { constructed = false;}
    
  //May need to add reference counting through
  //copy constrctor later;
    
  string& GetFileName() {return m_strFileName; };
  
  ostream& GetFileStream() {
    if(!constructed) {
      LOG_ERROR_MSG("file_name_io::I dont have a filename"); exit(-1);
    }
  
    if(!fileOpened) {
      m_fstream = new fstream(m_strFileName);
    }
    return *m_fstream; 
  };

private:
  bool fileOpened, constructed; 
  string m_strFileName;
  fstream* m_fstream;
};

*/





#endif

