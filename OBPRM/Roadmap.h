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
//#include "Defines.h"
#include "OBPRM.h"              // Cfg type defined here
#include "RoadmapGraph.h"       // graph class
#include "ConnectMapNodes.h"    // Map Node Connectors Algobase

class Input;              ///< init info (command line, env initfile)
class Environment;        ///< Environment classes
class LocalPlanners;      ///< Local Planner       Algobase
class GenerateMapNodes;   ///< Map Node Generators Algobase
class DistanceMetrics;    ///< Distance Metrics    Algobase
class CollisionDetection; ///< Collision Detection Algobase

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
	Roadmap(Input*, CollisionDetection*, 
		DistanceMetric*, LocalPlanners*, 
		Environment* env = NULL);
	
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
	void InitRoadmap(Input*, CollisionDetection*,
		DistanceMetric*,LocalPlanners*, char *ExistingMap=NULL, 
		Environment* env = NULL); 
	
		/**Initialize roadmap's environment from given Input instance.
		*
		*@note Not necessary if constructor "Roadmap(&input)" or
		*      method "InitRoadmap(input)" is used. 
		*@see Environment::Get and Read(int action)
	*/
	void InitEnvironment(Input*);
	
	/**Copy given Environment pointer to this roadmap's environment.
	*/
	void InitEnvironment(Environment*);
	
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
	Environment * GetEnvironment();
	
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
	void ReadRoadmap(Input*, CollisionDetection *cd,
		DistanceMetric *dm,
		LocalPlanners *lp,
		const char* _filename);
	
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
	void WriteRoadmap(Input*, CollisionDetection *cd,
		DistanceMetric *dm,
		LocalPlanners *lp,
		const char* _filename = NULL);
	
	void ReadRoadmapGRAPHONLY(const char* _filename);
	
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
	void ConvertGraph
		(istream&  myifstream, ostream& myofstream, int presentCfgFields, int presentEdgeWtFields);
	
	/**Backup old version roadmap file to another file.
	*@param _fname File name for old version roadmap file.
	*@param infile this method will copy _fname to infile.
	*client should allocate memory for infile.
	*@param outfile this method will be _fname.oldversion to
	*outfile. client should allocate memory for outfile.
	*@param thisVersion The version in _fname
	*@note this method copy every thing in infile to outfile.
	*/
	void SaveCurrentVersion
		(const char* _fname, int thisVersion, char* infile, char* outfile);
	
	
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
	
	Environment * environment;          ///< Environment.
	
	///< Roadmap built for environment. WEIGHT defined in OBPRM.h.
	RoadmapGraph<Cfg,WEIGHT> * m_pRoadmap; //an interface pointer

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

#endif
