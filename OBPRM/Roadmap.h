// $Id$
/////////////////////////////////////////////////////////////////////
//
/**@file Roadmap.h
  *
  * General Description
  *    This is the main OBPRM class which contains data and methods
  *    to manipulate the environment with specified moving bodies
  *    (ie, robot(s)) and the corresponding roadmap.
  *
  *    This file contains the prototype declarations for the class. 
  *    Definitions are in the file "Roadmap.cpp".
  *
  * @date 07/16/98  
  * @author Lucia K. Dale
  */
/////////////////////////////////////////////////////////////////////

#ifndef Roadmap_h
#define Roadmap_h

/////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "OBPRMDef.h"              // Cfg type defined here
#include "RoadmapGraph.h"       // graph class

class Input;              ///< init info (command line, env initfile)
class Environment;        ///< Environment classes
template <class CFG, class WEIGHT> class LocalPlanners;      ///< Local Planner       Algobase
class DistanceMetric;     ///< Distance Metrics    Algobase
class CollisionDetection; ///< Collision Detection Algobase
class InfoCfg;

/////////////////////////////////////////////////////////////////////

/**@name Constants for Roadmap Version LEGACY Infomation
  *Format version for roadmap (*.map) files.
  *The number breaks down as MonthDayYear (mmddyy).
  *For future versions, name them according to
  *YearMonthDay(yyyymmdd) so numerical
  *comparisons can be more easily made.
  *
  *@warning Be consistent.  It should be YYYYMMDD
  *         Inconsistent conversions can be misleading.
  *         For example, comparing 200083  to 20000604.
  */

//@{
#define RDMPVER_LEGACY                      0
#define RDMPVER_LEGACY_CFG_FIELDS           0            // none
#define RDMPVER_LEGACY_EDGEWT_FIELDS        1            // lp
//@}

/////////////////////////////////////////////////////////////////////
/**@name Constants for Roadmap Version 62000 Infomation*/ 
//@{

//Modified for VC

//This was changed to this because the VC reads numbers 
//starting with 0 (zero) as octal number (not decimal)
#if defined(_WIN32) || defined(__HP_aCC)
#define RDMPVER_62000                      062000
#else
#define RDMPVER_62000                      62000
#endif

#define RDMPVER_62000_CFG_FIELDS           2            ///< obst, tag
#define RDMPVER_62000_EDGEWT_FIELDS        2            ///< lp, ticks (or weight for DBL)

//@}

/////////////////////////////////////////////////////////////////////
/**@name Constants for Roadmap Version 61100 Infomation*/ 
//@{

//Modified for VC
#if defined(_WIN32) || defined(__HP_aCC)
#define RDMPVER_061100                      061100
#else
#define RDMPVER_061100                      61100
#endif

#define RDMPVER_061100_CFG_FIELDS           2            ///< obst, tag
#define RDMPVER_061100_EDGEWT_FIELDS        2            ///< lp, ticks (or weight for DBL)

//@}

/////////////////////////////////////////////////////////////////////
/**@name Constants for Roadmap Version 61300 Infomation*/ 
//@{

//Modified for VC
#if defined(WIN32) || defined(__HP_aCC)
#define RDMPVER_061300                      061300
#else
#define RDMPVER_061300                      61300
#endif

#define RDMPVER_061300_CFG_FIELDS           3            ///< obst, tag, clearance
#define RDMPVER_061300_EDGEWT_FIELDS        2            ///< lp, ticks (or weight for DBL)

//@}

/////////////////////////////////////////////////////////////////////
/**@name Constants for Roadmap Current Version Infomation*/ 
//@{

//Modified for VC
#if defined(WIN32) || defined(__HP_aCC)
#define RDMPVER_CURRENT                     061300 
#else
#define RDMPVER_CURRENT                     61300 
#endif

#define RDMPVER_CURRENT_STR                "061300"      ///< Current version # in string format
#define RDMPVER_CURRENT_CFG_FIELDS          3            ///< obst, tag, clearance
#define RDMPVER_CURRENT_EDGEWT_FIELDS       2            ///< lp, ticks (or weight for DBL)

//@}

/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class Roadmap {
public:
	
  /////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  /////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{
  
  ///Default Constrcutor. Do nothing.
  Roadmap();
  
  
  /** *Preferred* Constrcutor, 
   * fills input & inits.
   * This method initalizes it data members by
   * calling InitRoadmap, and set Roadmap version
   * as current version, RDMPVER_CURRENT.
   */
  Roadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm, 
	  LocalPlanners<CFG,WEIGHT>* lp, Environment* env = NULL);
  
  /**Delete all the elements of RoadMap. 
   *Actually do nothing currently.
   */
  ~Roadmap();
  
  //@}
  
  /////////////////////////////////////////////////////////////////////
  //
  //
  //    Init functions
  //
  //
  /////////////////////////////////////////////////////////////////////
  
  /**@name Initialization Methods*/
  //@{
  /**Initialize Roadmap (parse command line & read in data).
   *Initialize roadmap according to command line arguments.
   *reserve space for nodes/edges in roadmap graph.
   *read roadmap's environment.
   *@param ExistingMap if ExistingMap is not null, then
   *roadmap will read roadmap data from file 
   *named ExistingMap.
   *
   *@note this method calls InitEnvironment(Input*)
   *to initialize Environmentin this instance.
   *
   *@see InitEnvironment
   */
  void InitRoadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm, 
		   LocalPlanners<CFG,WEIGHT>* lp, char* ExistingMap=NULL, Environment* env = NULL); 
  
  /**Initialize roadmap's environment from given Input instance.
   *
   *@note Not necessary if constructor "Roadmap(&input)" or
   *      method "InitRoadmap(input)" is used. 
   *@see Environment::Get and Read(int action)
   */
  void InitEnvironment(Input* input);
  
  /**Copy given Environment pointer to this roadmap's environment.
   */
  void InitEnvironment(Environment* env);
  
  //@}
  
  /////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Method (Getting Statistics)
  //
  //
  /////////////////////////////////////////////////////////////////////
  /**@name Access Method.
   *Getting Statistics.
   */
  //@{
  
  /// Get a pointer to roadmap's environment 
  Environment* GetEnvironment();
  
  //@}
  
  
  /////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O (Display, Input, Output)
  //
  //
  /////////////////////////////////////////////////////////////////////
  /**@name I/O.
   *Display, Input, Output.
   */
  //@{
  
  /**Read data from roadmap file.
   *Usually, a readmap file is called *.map, which 
   *provide information like roadmap version, 
   *associated enviroment filename, what local planners are used, 
   *what collision detectors are used, and what distace metrics
   *are used when this roadmap was generated.
   *
   *@param _filename Filename of roadmap data. Usually *.map.
   *
   *@see CheckVersion, Input::ReadPreamble, Input::ReadEnvFile, 
   *LocalPlanners::ReadLPs, CollisionDetection::ReadCDs, 
   *DistanceMetric::ReadDMs, and WeightedMultiDiGraph::ReadGraph.
   *These methods are called in this function to help 
   *reading from *.map file.
   */
  void ReadRoadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm,
		   LocalPlanners<CFG,WEIGHT>* lp, const char* _fname);
  
  void ReadRoadmap(const char* _fname);
  
  /**Write data to roadmap file.
   *Usually, a readmap file is called *.map, which 
   *provide information like roadmap version, 
   *associated enviroment filename, what local planners are used, 
   *what collision detectors are used, and what distace metrics
   *are used when this roadmap was generated.
   *
   *@param _filename Filename of roadmap data. Usually *.map.
   *
   *@see CheckVersion, Input::WritePreamble, Input::WriteEnvFile, 
   *LocalPlanners::WriteLPs, CollisionDetection::WriteCDs, 
   *DistanceMetric::WriteDMs and WeightedMultiDiGraph::WriteGraph. 
   *These methods are called in this function to help 
   *writing to *.map file.
   */
  void WriteRoadmap(Input* input, CollisionDetection *cd, DistanceMetric* dm,
		    LocalPlanners<CFG,WEIGHT>* lp, const char* _fname = NULL);
  
  void ReadRoadmapGRAPHONLY(const char* _fname);
  
  /**Check version of roadmap file.
   *@param The file which contain roadmap data and version info.
   *@return True if the version of this is current version, 
   *RDMPVER_CURRENT. Otherwise this method will call 
   *ConvertToCurrentVersion and return what it returnes.
   *@see RDMPVER_CURRENT for current version number and
   *ConvertToCurrentVersion
   */
  bool CheckVersion(const char* _fname);
  
  /**utilities to automatically convert to current roadmap version
   *This method reads version number in specified file,
   *and convert file accroding to this number.
   *Before converting, this method will
   *copy current file _fname to "_fname.XXX" where XXX=thisVersion.
   *
   *@see Roadmap.h for more information about formats for versions
   *and ConvertGraph for roadmap graph conversion, SaveCurrentVersion
   *for backup data in old version.
   */
  bool ConvertToCurrentVersion(const char* _fname, int thisVersion);
	
  /**Convert format for old graph to newest verion.
   *This is done by filling null values to new fields which
   *is not defined in old version.
   */
  void ConvertGraph(istream& myifstream, ostream& myofstream, 
		    int presentCfgFields, int presentEdgeWtFields);
  
  /**Backup old version roadmap file to another file.
   *@param _fname File name for old version roadmap file.
   *@param infile this method will copy _fname to infile.
   *client should allocate memory for infile.
   *@param outfile this method will be _fname.oldversion to
   *outfile. client should allocate memory for outfile.
   *@param thisVersion The version in _fname
   *@note this method copy every thing in infile to outfile.
   */
  void SaveCurrentVersion(const char* _fname, int thisVersion, 
			  char* infile, char* outfile);
  
  
  /**Write roadmap's environment to an output stream.
   *This method calls Environment::Write to do this.
   *@see Environment::Write(ostream&)
   */
  void WriteEnvironment(ostream& _ostream);
  
  //@}
  
  /////////////////////////////////////////////////////////////////////
  //
  //
  //    Data
  //
  //
  /////////////////////////////////////////////////////////////////////
  
  Environment* environment;          ///< Environment.
  
  ///< Roadmap built for environment. WEIGHT defined in OBPRM.h.
  RoadmapGraph<CFG,WEIGHT>* m_pRoadmap; //an interface pointer
  
  int RoadmapVersionNumber;           ///< Newest version number.
  
  /////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data members and Member methods
  //
  //
  /////////////////////////////////////////////////////////////////////
 protected:
  
  /////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data members and Member methods
  //
  //
  /////////////////////////////////////////////////////////////////////
 private:
};


#include "BasicDefns.h"
#include "Environment.h"
#include "DistanceMetrics.h"
#include "util.h"

//===================================================================
//  Roadmap class Methods: Constructors and Destructor
//===================================================================
template <class CFG, class WEIGHT>
Roadmap<CFG, WEIGHT>::
Roadmap() {
  m_pRoadmap = new RoadmapGraph<CFG, WEIGHT>;
}


template <class CFG, class WEIGHT>
Roadmap<CFG, WEIGHT>::
Roadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm,
	LocalPlanners<CFG,WEIGHT>* lp, Environment* env) {
  m_pRoadmap = new RoadmapGraph<CFG, WEIGHT>;
  
  InitRoadmap(input, cd, dm, lp, NULL, env); 
  RoadmapVersionNumber = RDMPVER_CURRENT;
}


template <class CFG, class WEIGHT>
Roadmap<CFG, WEIGHT>::
~Roadmap() {
  if( m_pRoadmap != NULL ) 
    delete m_pRoadmap;
  if( environment != NULL ) 
    delete environment;
  m_pRoadmap = NULL;
  environment = NULL;
}


//////////////////////////////////////////////////////////////////////
//  Initialize roadmap according to command line arguments
//  -- reserve space for nodes/edges in roadmap graph
//  -- read roadmap's environment 
//////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void
Roadmap<CFG, WEIGHT>::
InitRoadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm,
	    LocalPlanners<CFG,WEIGHT>* lp, char* ExistingMap, Environment* env) {
  //---------------------------------------------------------
  // initialize roadmap, from scratch or previously built map
  //---------------------------------------------------------
  if ( ExistingMap != NULL ) {           // start from previous map
    ReadRoadmap( input, cd,dm,lp, ExistingMap );
  }
  
  //-----------------------------------------------
  // initialize problem environment
  //-----------------------------------------------
  if(env == NULL) {
    environment = new Environment;
    InitEnvironment( input );
  } else {
    InitEnvironment(env);      
  }
};


//////////////////////////////////////////////////////////////////////
//  Initialize roadmap's environment from a file
//
//  **NOTE** Not necessary if constructor "Roadmap(&input)" or
//           method "InitRoadmap(input)" is used. 
//////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void
Roadmap<CFG, WEIGHT>::
InitEnvironment(Input* input) {
  // open 'init' file, read it into Input object, & close it
  input->Read(EXIT);
  // read environment files & put them in Environment object
  environment->Get(input);
};


template <class CFG, class WEIGHT>
void
Roadmap<CFG, WEIGHT>::
InitEnvironment(Environment* env) {
  environment = env;
};



//========================================================================
// Roadmap class Methods: Getting Data & Statistics
//========================================================================

//
// Get a pointer to roadmap's environment
//
template <class CFG, class WEIGHT>
Environment*
Roadmap<CFG, WEIGHT>::
GetEnvironment() { 
  return environment;
};


template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
ReadRoadmap(const char* _fname) {
  ifstream  myifstream(_fname);
  if (!myifstream) {
    cout << endl << "In ReadRoadmap: can't open infile: " << _fname ;
    return;
  }  
};


//========================================================================
// Roadmap class Methods: Display, Input, Output
//========================================================================
template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
ReadRoadmapGRAPHONLY(const char* _fname) {
  cout << endl << "getting nodes from Read: " << _fname << endl;
  
  ifstream  myifstream(_fname);
  if (!myifstream) {
    cout << endl << "In ReadRoadmap: can't open infile: " << _fname ;
    return;
  }
  
  // skip over stuff up to and including DMSTOP, next line should contain GRAPHSTART
  char tagstring[400]; bool moreFile=true;
  while(moreFile) {
    myifstream  >> tagstring;
    if ( strstr(tagstring,"DMSTOP") ) 
      moreFile = false;
  }
  
  m_pRoadmap->ReadGraph(myifstream);           // reads verts & adj lists
  
  myifstream.close();
}


//
// Read roadmap from a file
//
template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
ReadRoadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm,
	    LocalPlanners<CFG,WEIGHT>* lp, const char* _fname) {
  if ( !CheckVersion(_fname) ) {
    cout << endl << "In ReadRoadmap: don't recognize format in: " << _fname ;
    return;
  }
  
  ifstream  myifstream(_fname);
  if (!myifstream) {
    cout << endl << "In ReadRoadmap: can't open infile: " << _fname ;
    return;
  }
  
  // Read throwaway tag strings..."Roadmap Version Number"
  char tagstring[30];
  myifstream >> tagstring >> tagstring >> tagstring; 
  myifstream >> RoadmapVersionNumber;
  
  input->ReadPreamble(myifstream); // do nothing now (could init defaults)
  input->ReadEnvFile(myifstream);    
  lp->ReadLPs(myifstream);
  cd->collisionCheckers.ReadCDs(myifstream);
  dm->distanceMetrics.ReadDMs(myifstream);
  
  m_pRoadmap->ReadGraph(myifstream);           // reads verts & adj lists
  
  myifstream.close();
};


//
// Write roadmap to a file
//
template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
WriteRoadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm,
	     LocalPlanners<CFG,WEIGHT>* lp, const char* _fname) {
  char outfile[200];
  
  if ( _fname == NULL ) {
    strcpy(outfile,input->mapFile.GetValue() );
  } else {
    strcpy( outfile,_fname );
  }
  
  ofstream  myofstream(outfile);
  if (!myofstream) {
    cout << endl << "In WriteRoadmap: can't open outfile: " << outfile ;
  }
  
  myofstream << "Roadmap Version Number " << RDMPVER_CURRENT_STR;
  
  input->WritePreamble(myofstream); // for now just write commandline
  input->WriteEnvFile(myofstream);
  lp->WriteLPs(myofstream);
  cd->collisionCheckers.WriteCDs(myofstream);
  dm->distanceMetrics.WriteDMs(myofstream);
  
  m_pRoadmap->WriteGraph(myofstream);         // writes verts & adj lists
  myofstream.close();
};


//
// Write roadmap's environment to an output stream
//
template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
WriteEnvironment(ostream& _os) {
  environment->Write(_os);
};


//////////////////////////////////////////////////////////////////////
// Roadmap Version Check and Convertors
//////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
bool 
Roadmap<CFG, WEIGHT>::
CheckVersion(const char* _fname) {
  ifstream  myifstream(_fname);
  if (!myifstream) {
    cout << endl << "In CheckVersion: can't open infile: " << _fname ;
    return false;
  }
  
  int  thisVersion; 
  char tagstring[30];
  
  myifstream  >> tagstring; 
  if ( strstr(tagstring,"PREAMBLESTART") ) {
    thisVersion = RDMPVER_LEGACY;
  } else if ( strstr(tagstring,"Roadmap") ) {
    myifstream  >>tagstring >> tagstring;  // read "Version" and "Number"
    myifstream >> thisVersion;                 
  }
  
  myifstream.close();
  
  if ( thisVersion == RDMPVER_CURRENT  ) {
    return true;
  } else {
    return ConvertToCurrentVersion(_fname,thisVersion);
  }  
} 


//
// Convert To Current Roadmap Version 
//
template <class CFG, class WEIGHT>
bool 
Roadmap<CFG, WEIGHT>::
ConvertToCurrentVersion(const char* _fname, int thisVersion) {
  char infile[200], outfile[200];
  
  // copy current file _fname to "_fname.XXX"  where XXX=thisVersion
  // construct infile=_fname.XXX  and outfile=_fname names
  SaveCurrentVersion(_fname, thisVersion, infile, outfile);
  
  ifstream  myifstream(infile);
  if (!myifstream) {
    cout << endl << "In ConvertToCurrentVersion: can't open infile: " << infile ;
    return false;
  }
  
  ofstream  myofstream(outfile);
  if (!myofstream) {
    cout << endl << "In ConvertToCurrentVersion: can't open outfile: " << outfile ;
    return false;
  }
  
  char tagstring[500];
  
  if ( thisVersion != RDMPVER_LEGACY ) {
    // read tags "Roadmap Version Number XXX" -- don't exist in LEGACY VERSION
    myifstream >> tagstring >> tagstring >> tagstring >> tagstring;
  }
  myofstream << "Roadmap Version Number " << RDMPVER_CURRENT_STR; 
  
  // copy myifstream into myofstream until get to graph...
  while ( !strstr(tagstring,"DMSTOP") ) {
    myifstream.getline(tagstring,499);
    myofstream << tagstring << endl;
  }
  
  if ( thisVersion == RDMPVER_LEGACY) {
    // legacy: 0 cfg fields, 1 wt field (lp)
    ConvertGraph(myifstream, myofstream, 
		 RDMPVER_LEGACY_CFG_FIELDS, RDMPVER_LEGACY_EDGEWT_FIELDS);
    myofstream << endl << "Converted from ROADMAP VERSION LEGACY";
    
  } else if (thisVersion == RDMPVER_62000) {
    // 62000: 2 cfg fields (obst, tag), 2 wt fields (lp, ticks/weight)
    ConvertGraph(myifstream, myofstream, 
		 RDMPVER_62000_CFG_FIELDS, RDMPVER_62000_EDGEWT_FIELDS);
    myofstream << endl << "Converted from ROADMAP VERSION 62000";
    
  } else if (thisVersion == RDMPVER_061100) {
    // 061100: 2 cfg fields (obst, tag), 2 wt fields (lp, ticks/weight)
    ConvertGraph(myifstream, myofstream, 
		 RDMPVER_061100_CFG_FIELDS, RDMPVER_061100_EDGEWT_FIELDS);
    myofstream << endl << "Converted from ROADMAP VERSION 061100";
    
  } else if (thisVersion == RDMPVER_061300) {
    // 061300: 3 cfg fields (obst, tag, clearance), 2 wt fields (lp, ticks/weight)
    ConvertGraph(myifstream, myofstream,
		 RDMPVER_061300_CFG_FIELDS, RDMPVER_061300_EDGEWT_FIELDS);
    myofstream << endl << "Converted from ROADMAP VERSION 061300";
    
  } else {
    cout << "In ConvertToCurrentVersion: unknown roadmap version in " << _fname << endl;
    return false;
  }
  
  myifstream.close();
  myofstream.close();
  return true;
}


//
// copies file _fname to "_fname.XXX" where XXX==thisVersion
// and constructs infile = _fname.XXX and outfile = _fname
//
template <class CFG, class WEIGHT>
void
Roadmap<CFG, WEIGHT>::
SaveCurrentVersion(const char* _fname, int thisVersion, 
		   char* infile, char* outfile) {
  char tagstring[500];
  
  // construct names for infile (_fname) and outfile (_fname.XXX)
  strcpy( infile, _fname );
  strcpy( outfile, _fname );
  strcat( outfile, ".RdmpVer-" );
  if ( thisVersion == RDMPVER_LEGACY ) {
    strcat ( outfile, "LEGACY" );
  } else if ( thisVersion == RDMPVER_62000 ) {
    strcat ( outfile, "62000" );
  } else if ( thisVersion == RDMPVER_061100 ) {
    strcat ( outfile, "061100" );
  } else if ( thisVersion == RDMPVER_061300 ) {
    strcat ( outfile, "061300" );
  } else {
    strcat ( outfile, "UNKNOWN_VERSION" );
  }
  
  // open infile and outfile
  ifstream  myifstream(infile);
  if (!myifstream) {
    cout << endl << "In SaveCurrentVersion: cannot open infile: " << infile;
    return;
  }
  ofstream  myofstream(outfile);
  if (!myofstream) {
    cout << endl << "In SaveCurrentVersion: cannot open outfile: " << outfile;
    return;
  }
  
  // copy infile into outfile
  while ( myifstream.getline(tagstring,499) ){
    myofstream << tagstring << endl;
  }
  
  // now, swap the names so we'll read from _fname.XXX and write to _fname
  strcpy( infile, outfile ); 
  strcpy( outfile, _fname ); 
}


//
// This function will read and convert the lines of the graph file relating
// to the adjacency lists *ONLY*.
//
// ASSUMPTION: the old version is a subset of the new version, missing only
//             some values in either the InfoCfg field or in the Edge Weights
// REMEDY: the missing fields will be padded with some "null" value - note
//         this is not a robust approach and may yield trouble later...
//
template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
ConvertGraph(istream&  myifstream, ostream& myofstream, 
	     int presentCfgFields, int presentEdgeWtFields) {
  char tagstring[200];
  int v, i, j, nverts, nedges, vids;
  
  // GRAPHSTART TAG
  myifstream >> tagstring;
  myofstream << tagstring << endl;
  // GRAPH info (#verts #edges #vids) 
  myifstream >> nverts >> nedges >> vids;
  myofstream << nverts << " " << nedges << " " << vids << endl;
  
  CFG c;
  int dofs = c.DOF();
  int missingCfgFields     = RDMPVER_CURRENT_CFG_FIELDS - presentCfgFields;
  int missingEdgeWtFields  = RDMPVER_CURRENT_EDGEWT_FIELDS - presentEdgeWtFields;
  
  for (v=0; v < nverts; v++) {
    myifstream >> vids; 
    myofstream << vids << " "; 
    for (i=0; i < dofs; i++) {
      myifstream >> tagstring;
      myofstream << tagstring << " "; 
    }
    for (i=0; i < presentCfgFields; i++) {
      myifstream >> tagstring;
      myofstream << tagstring << " "; 
    }
    for (i=0; i < missingCfgFields; i++) {
      myofstream << InfoCfg::NULL_INFO << " "; 
    }
    
    int adjedges;
    myifstream >> adjedges; 
    myofstream << adjedges << " "; 
    for (i=0; i < adjedges; i++) {
      myifstream >> tagstring;         // vid of adj vertex for this edge
      myofstream << tagstring << " "; 
      for (j=0; j < presentEdgeWtFields; j++) {
	myifstream >> tagstring;
	myofstream << tagstring << " "; 
      }
      for (j=0; j < missingEdgeWtFields; j++) {
	myofstream << NULL_WT_INFO << " "; 
      }
    }
    myofstream << endl;
  }
  
  // GRAPHSTOP TAG
  myifstream >> tagstring;
  myofstream << tagstring << endl;
  
  return;
}



#endif
