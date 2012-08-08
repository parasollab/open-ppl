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

#include "RoadmapGraph.h"       // graph class
#include "Environment.h"


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
/**@name Constants for Roadmap Version 10604 Infomation*/ 
//@{

//Modified for VC
#if defined(WIN32) || defined(__HP_aCC)
#define RDMPVER_010604                      010604
#else
#define RDMPVER_010604                      10604
#endif

#define RDMPVER_010604_CFG_FIELDS           3            ///< obst, tag, clearance
#define RDMPVER_010604_EDGEWT_FIELDS        2            ///< lp, ticks (or weight for DBL)

/////////////////////////////////////////////////////////////////////
/**@name Constants for Roadmap Version 10604 Infomation*/ 
//@{

//Modified for VC
#if defined(WIN32) || defined(__HP_aCC)
#define RDMPVER_041805                      041805
#else
#define RDMPVER_041805                      41805
#endif

#define RDMPVER_041805_CFG_FIELDS           3            ///< obst, tag, clearance
#define RDMPVER_041805_EDGEWT_FIELDS        2            ///< lp, ticks (or weight for DBL)

//@}

/////////////////////////////////////////////////////////////////////
/**@name Constants for Roadmap Current Version Infomation*/ 
//@{

//Modified for VC
#if defined(WIN32) || defined(__HP_aCC)
#define RDMPVER_CURRENT                     041805 
#else
#define RDMPVER_CURRENT                     41805
#endif

#define RDMPVER_CURRENT_STR                "041805"      ///< Current version # in string format
#define RDMPVER_CURRENT_CFG_FIELDS          3            ///< obst, tag, clearance
#define RDMPVER_CURRENT_EDGEWT_FIELDS       2            ///< lp, ticks (or weight for DBL)
#define INVALID_RNGSEED_STR                "-999"        ///< invalid RNG seed string: -999

//@}

/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class Roadmap {
public:
  typedef CFG CfgType;
  typedef WEIGHT WeightType;
  typedef RoadmapGraph<CFG, WEIGHT> RoadmapGraphType;
  typedef typename RoadmapGraphType::vertex_descriptor VID;  
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

  //Copy Constructor. Create a new roadmap from another roadmap.
  Roadmap(Roadmap<CFG,WEIGHT> &from_rdmp);  
  
  /** *Preferred* Constrcutor, 
   * fills input & inits.
   * This method initalizes it data members by
   * calling InitRoadmap, and set Roadmap version
   * as current version, RDMPVER_CURRENT.
   */
  //Roadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm, 
  //  LocalPlanners<CFG,WEIGHT>* lp, long RNGseedValue, Environment* env);

  
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

    
  /**Copy given Environment pointer to this roadmap's environment.
   */
  void SetEnvironment(Environment* env);
  
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
  //void ReadRoadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm,
  //     LocalPlanners<CFG,WEIGHT>* lp, const char* _fname);
  
  /* @todo: void ReadRoadmap(const char* _fname); */
  
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
  //void WriteRoadmap(Input* input, CollisionDetection *cd, DistanceMetric* dm,
  //      LocalPlanners<CFG,WEIGHT>* lp, const char* _fname = NULL);
  
  void ReadRoadmapGRAPHONLY(const char* _fname);
  

  /**Append nodes and edges from one roadmap (from_rdmp) into 
   * another roadmap (to_rdmp)
   */
  vector<VID> AppendRoadmap(Roadmap<CFG,WEIGHT> &from_rdmp);

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
  void ConvertGraph(istream& myifstream, ostream& myofstream, int thisVersion, 
        int presentCfgFields, int presentEdgeWtFields);
  
  /**Backup old version roadmap file to another file.
   *@param _fname File name for old version roadmap file.
   *@param infile this method will copy _fname to infile.
   *client should allocate memory for infile.
   *@param outfile this method will be _fname.oldversion to
   *outfile. client should allocate memory for outfile.
   *@param thisVersion The version in _fname
   *@note this method copy every thing in infile to outfile.
   * @todo make conversion perl scripts and remove conversion code from roadmap.
   */
  void SaveCurrentVersion(const char* _fname, int thisVersion, 
        char* infile, char* outfile);

  /**Read information about RNGseed from file.
    *@param _fname filename for data file.
    *@see ReadRNGseed(istream& _myistream)
    */
  void ReadRNGseed(const char* _fname);
  
  /**Read information about RNGseed from input stream.
    *@see WriteRNGseed for data format
    *@note if it is not a value, then error message will be post to 
    *standard output.
    */
  void ReadRNGseed(istream& _myistream);
  
  /**Ouput RNGseed to file.
    *@param _fname filename for data file.
    *@see WriteRNGseed(ostream& _myostream)
    */
  void WriteRNGseed(const char* _fname) const;
  
  /**Ouput information about RNGseed output stream.
    *@note format: long
    *@note if a map is converted from older version map, we put "-999"
    *
    */
  void WriteRNGseed(ostream& _myostream) const;


  /////////////////////////////////////////////////////////////////////
  //
  //
  //    Acess
  //
  //
  /////////////////////////////////////////////////////////////////////

  void SetRNGseed(long seedval) {RNGseed = seedval;}
  long GetRNGseed() const {return RNGseed;}


   bool IsCached(VID _v1, VID _v2) {
    if(m_lpcache.count(make_pair(min(_v1,_v2), max(_v1,_v2))) > 0)
      return true;
    else
      return false;
  }

  bool GetCache(VID _v1, VID _v2) {
//    if(IsCached(_v1,_v2)) {
      return m_lpcache[make_pair(min(_v1,_v2), max(_v1,_v2))];
//    } else {
//      cout << "LocalPlannerMethod::GetCache -- Cache Error " << endl;
//      exit(-1);
//    }
  }

  void SetCache(VID _v1, VID _v2, bool _b) {
    m_lpcache[make_pair(min(_v1,_v2),max(_v1,_v2))] = _b;
  }
  
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

  long RNGseed;                      ///seed for random number generator
  
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
  std::map<std::pair<VID,VID>,bool> m_lpcache;
};


//===================================================================
//  Roadmap class Methods: Constructors and Destructor
//===================================================================
template <class CFG, class WEIGHT>
Roadmap<CFG, WEIGHT>::
Roadmap() 
  :  environment(NULL), 
     RoadmapVersionNumber(RDMPVER_CURRENT),
     RNGseed(0) {
  m_pRoadmap = new RoadmapGraph<CFG, WEIGHT>;
}

/*
template <class CFG, class WEIGHT>
Roadmap<CFG, WEIGHT>::
Roadmap(Input* input, CollisionDetection* cd, DistanceMetric* dm,
  LocalPlanners<CFG,WEIGHT>* lp, long RNGseedValue, Environment* env)
  : RoadmapVersionNumber(RDMPVER_CURRENT) {
  m_pRoadmap = new RoadmapGraph<CFG, WEIGHT>;
  
  InitRoadmap(input, cd, dm, lp, NULL, env); 
  RNGseed = RNGseedValue;
}
*/
template <class CFG, class WEIGHT>
Roadmap<CFG, WEIGHT>:: 
Roadmap(Roadmap<CFG, WEIGHT> &from_rdmp)
  :  RoadmapVersionNumber(from_rdmp.RoadmapVersionNumber),
     RNGseed(from_rdmp.RNGseed) {
  environment = from_rdmp.GetEnvironment();
  m_pRoadmap = new RoadmapGraph<CFG, WEIGHT>;
  AppendRoadmap(from_rdmp);
}

template <class CFG, class WEIGHT>
vector<typename Roadmap<CFG, WEIGHT>::VID>
Roadmap<CFG, WEIGHT>::
AppendRoadmap(Roadmap<CFG, WEIGHT> &from_rdmp) {
  vector<VID> from_vid, to_vids;
  from_rdmp.m_pRoadmap->GetVerticesVID(from_vid); // get vertices
  typename vector<VID>::iterator vid_itrt;
  //copy vertices
  for (vid_itrt = from_vid.begin(); vid_itrt < from_vid.end(); vid_itrt++) {
    CFG cfg = (*(from_rdmp.m_pRoadmap->find_vertex(*vid_itrt))).property();
    to_vids.push_back(m_pRoadmap->AddVertex(cfg));
  }
  vector< pair<pair<VID,VID>,WEIGHT> > edges;
  typename vector< pair<pair<VID,VID>,WEIGHT> >::iterator edge_itrt;
  for (vid_itrt = from_vid.begin(); vid_itrt < from_vid.end(); vid_itrt++) {
    edges.clear();
    //from_rdmp.m_pRoadmap->GetOutgoingEdges(*vid_itrt, edges); //get edges fix_lantao
    //use iterator to traverse the adj edges and then put the data into edges
    typename RoadmapGraph<CFG, WEIGHT>::vertex_iterator vi = from_rdmp.m_pRoadmap->find_vertex(*vid_itrt);
    for(typename RoadmapGraph<CFG, WEIGHT>::adj_edge_iterator ei =(*vi).begin(); ei!=(*vi).end(); ei++ ){
	pair<pair<VID,VID>,WEIGHT> single_edge;
	single_edge.first.first=(*ei).source();
	single_edge.first.second=(*ei).target();
	single_edge.second = (*ei).property();
	edges.push_back(single_edge); //put the edge into edges
    } 
    for (edge_itrt = edges.begin(); edge_itrt < edges.end(); edge_itrt++) {
      if (!m_pRoadmap->IsEdge((*edge_itrt).first.first, (*edge_itrt).first.second)) { //add an edge if it is not yet in m_pRoadmap
  CFG cfg_a = (*(from_rdmp.m_pRoadmap->find_vertex((*edge_itrt).first.first))).property();
  CFG cfg_b = (*(from_rdmp.m_pRoadmap->find_vertex((*edge_itrt).first.second))).property();
  m_pRoadmap->AddEdge(cfg_a,cfg_b,(*edge_itrt).second);
      }
    } //endfor edge_itrt  
  } //endfor vid_itrt
  return to_vids;
}

template <class CFG, class WEIGHT>
Roadmap<CFG, WEIGHT>::
~Roadmap() {
  if( m_pRoadmap != NULL ) 
    delete m_pRoadmap;
  m_pRoadmap = NULL;
}


//////////////////////////////////////////////////////////////////////
//  Initialize roadmap according to command line arguments
//  -- reserve space for nodes/edges in roadmap graph
//  -- read roadmap's environment 
//////////////////////////////////////////////////////////////////////
/*template <class CFG, class WEIGHT>
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
    CFG test_cfg;
    environment = new Environment(test_cfg.DOF(),test_cfg.posDOF(), input);
  } else {
    SetEnvironment(env);      
  }
};
*/

template <class CFG, class WEIGHT>
void
Roadmap<CFG, WEIGHT>::
SetEnvironment(Environment* env) {
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


//========================================================================
// Roadmap class Methods: Display, Input, Output
//========================================================================
//read_graph not implemented in parallel yet, the ifdef will be removed when we have
//read_graph algo for p_graph

template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
ReadRoadmapGRAPHONLY(const char* _fname) {
  #ifndef _PARALLEL
  cout << endl << "getting nodes from Read: " << _fname << endl;

  ifstream  myifstream(_fname);
  if(!myifstream) 
  {
    cout << endl << "In ReadRoadmap: can't open infile: " << _fname ;
    return;
  }
  char tagstring[400]; 
  bool moreFile = true;
  int count = 0;
  while(moreFile && (myifstream >> tagstring)) 
  {
    count++;
    if(strstr(tagstring,"GRAPHSTART")) 
      moreFile = false;
  }
  myifstream.close();
  
  if(!moreFile)
  {
    ifstream myifstream2(_fname);
    if(!myifstream2)
    {
      cout << endl << "In ReadRoadmap: can't open infile: " << _fname;
      return;
    }
    for(int i=0; i<count-1; ++i)
      myifstream2 >> tagstring;
    //m_pRoadmap->ReadGraph(myifstream2);
    stapl::sequential::read_graph(*m_pRoadmap, myifstream2);
    myifstream2.close();
  }
  else
  {
    cout << endl  << "In ReadRoadmap: didn't read GRAPHSTART tag right";
    return;
  }
  #endif
}




//
// Read roadmap from a file
//
/*
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
  
  input->ReadPreamble(myifstream);
  input->ReadEnvFile(myifstream);    
  lp->ReadLPs(myifstream);
  cd->ReadCDs(myifstream);
  dm->ReadDMs(myifstream);
  ReadRNGseed(myifstream);

  m_pRoadmap->ReadGraph(myifstream);           // reads verts & adj lists
  
  myifstream.close();
};
*/

//
// Write roadmap to a file
//
/*
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
  cd->WriteCDs(myofstream);
  dm->WriteDMs(myofstream);
  WriteRNGseed(myofstream);

  m_pRoadmap->WriteGraph(myofstream);         // writes verts & adj lists
  myofstream.close();
};
*/


//////////////////////////////////////////////////////////////////////
// Read/Write RNGseed
//////////////////////////////////////////////////////////////////////

template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
ReadRNGseed(const char* _fname) {
  ifstream  myifstream(_fname);
  if (!myifstream) {
    cout << endl << "In ReadRNGseed: can't open infile: " << _fname ;
    return;
  }
  ReadRNGseed(myifstream);
  myifstream.close();
}


template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
ReadRNGseed(istream& _myistream) {
  char tagstring[100];
  char rngdesc[100];
  int  RNGseed;
  
  _myistream >> tagstring;
  if ( !strstr(tagstring,"RNGSEEDSTART") ) {
    cout << endl << "In ReadRNGseed: didn't read RNGSEEDSTART tag right";
    return;
  }
  
  _myistream >> RNGseed;
  _myistream.getline(rngdesc,100,'\n');  // throw out rest of this line

  _myistream >> tagstring;
  if ( !strstr(tagstring,"RNGSEEDSTOP") ) {
    cout << endl << "In ReadRNGseed: didn't read RNGSEEDSTOP tag right";
    return;
  }
}

template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
WriteRNGseed(const char* _fname) const {
  ofstream  myofstream(_fname);
  if (!myofstream) {
    cout << endl << "In WriteRNGseed: can't open outfile: " << _fname ;
  }
  WriteRNGseed(myofstream);
  myofstream.close();
}


template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
WriteRNGseed(ostream& _myostream) const {
  _myostream << endl << "#####RNGSEEDSTART#####";
  _myostream << endl << RNGseed;  // seed for RNG
  _myostream << endl <<"#####RNGSEEDSTOP#####"; 
}



//////////////////////////////////////////////////////////////////////
// Roadmap Version Check and Convertors
//////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
bool 
Roadmap<CFG, WEIGHT>::
CheckVersion(const char* _fname) {
  cout<<"Roadmap::CheckVersion is called"<<endl;
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
  cout<<"Roadmap::ConvertToCurrentVersion is called"<<endl;
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
  std::string graphLine;

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
  
  //add RNGseed section
  myofstream <<"#####RNGSEEDSTART#####"<<endl;
  myofstream << INVALID_RNGSEED_STR<<endl;  // INVALID SEED
  myofstream << "#####RNGSEEDSTOP#####"<<endl; 

  if ( thisVersion == RDMPVER_LEGACY) {
    // legacy: 0 cfg fields, 1 wt field (lp)
    ConvertGraph(myifstream, myofstream, thisVersion,
     RDMPVER_LEGACY_CFG_FIELDS, RDMPVER_LEGACY_EDGEWT_FIELDS);
    myofstream << endl << "Converted from ROADMAP VERSION LEGACY";
    
  } else if (thisVersion == RDMPVER_62000) {
    // 62000: 2 cfg fields (obst, tag), 2 wt fields (lp, ticks/weight)
    ConvertGraph(myifstream, myofstream, thisVersion,
     RDMPVER_62000_CFG_FIELDS, RDMPVER_62000_EDGEWT_FIELDS);
    myofstream << endl << "Converted from ROADMAP VERSION 62000";
    
  } else if (thisVersion == RDMPVER_061100) {
    // 061100: 2 cfg fields (obst, tag), 2 wt fields (lp, ticks/weight)
    ConvertGraph(myifstream, myofstream, thisVersion,
     RDMPVER_061100_CFG_FIELDS, RDMPVER_061100_EDGEWT_FIELDS);
    myofstream << endl << "Converted from ROADMAP VERSION 061100";
    
  } else if (thisVersion == RDMPVER_061300) {
    // 061300: 3 cfg fields (obst, tag, clearance), 2 wt fields (lp, ticks/weight)
    ConvertGraph(myifstream, myofstream, thisVersion,
     RDMPVER_061300_CFG_FIELDS, RDMPVER_061300_EDGEWT_FIELDS);
    myofstream << endl << "Converted from ROADMAP VERSION 061300";
    
  } else if (thisVersion == RDMPVER_010604) {
    // 010604: 3 cfg fields (obst, tag, clearance), 2 wt fields (lp, ticks/weight)
    cout<<"copy each line for a graph"<<endl;
    while ( graphLine.find("GRAPHSTOP") == string::npos ) {
      getline(myifstream, graphLine);
      myofstream << graphLine << endl;
    } 
    myofstream << endl << "Converted from ROADMAP VERSION 010604";
    
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
  //char tagstring[500];
  string tagstring;
  cout<<"Roadmap::SaveCurrentVersion is called"<<endl;
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
  } else if ( thisVersion == RDMPVER_010604 ) {
    strcat ( outfile, "010604" );
  }  else {
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
  //while ( myifstream.getline(tagstring,499) ){
  //  myofstream << tagstring << endl;
  //}
  
  while ( getline(myifstream, tagstring) ){
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
//             some values in either the Cfg fields or in the Edge Weights
// REMEDY: the missing fields will be padded with some "null" value - note
//         this is not a robust approach and may yield trouble later...
//
template <class CFG, class WEIGHT>
void 
Roadmap<CFG, WEIGHT>::
ConvertGraph(istream&  myifstream, ostream& myofstream, int thisVersion,
       int presentCfgFields, int presentEdgeWtFields) {
  char tagstring[200];
  int v, i, j, nverts, nedges, vids;
  
  // GRAPHSTART TAG
  myifstream >> tagstring;
  myofstream << tagstring << endl;
  // GRAPH info (#verts #edges #vids) 
  myifstream >> nverts >> nedges >> vids;
  if((thisVersion == RDMPVER_061300) || 
     (thisVersion == RDMPVER_061100) ||
     (thisVersion == RDMPVER_62000) || 
     (thisVersion == RDMPVER_LEGACY)) //old graph was undirected
    myofstream << nverts << " " << nedges*2 << " " << vids << endl;
  else
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
      myofstream << -1 << " "; 
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
