// $Id$

//////////////////////////////////////////////////////////////////////////////////////////
/**@file GenerateMapNodes.h
  *This set of classes supports a "RoadMap Node Generation Algobase".
  *Generate roadmap nodes and enter each as a graph node.
  *
  *The classes in the set are:
  *	-# GN            -- info related to an individual method of node
  *                        generation
  *	-# GNSets        -- contains 'master' list of all generation methods
  *                      and maintains sets of generation methods
  *                      (derived from BasicSets<GN>)
  *	-# GenerateMapNodes
  *                   -- has GNSets as data element, and its methods
  *                      include the actual generation algorithms.
  *                      A 'wrapper' method cycles thru sets of gn's.
  *
  *  Each GN element is given a unique id, which is used to compose
  *         hmmm, I don't know what...
  *
  *@author Lucia K. Dale
  *@date   8/27/98
  */
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef GenerateMapNodes_h
#define GenerateMapNodes_h

//////////////////////////////////////////////////////////////////////////////////////////
// Include OBPRM headers
#include "Input.h"
#include "CollisionDetection.h"

//////////////////////////////////////////////////////////////////////////////////////////
class Cfg;
class Body;
//////////////////////////////////////////////////////////////////////////////////////////

#define MAX_NODES          5000000
#define MAX_NODES_PER_OBST 5000

/** Pre-defined Algobase Sets
  * @warning DO NOT CHANGE THE ENUMERATION ORDERS w/o CHANGING
  *          SET DEFN's in "GenerateMapNodes.cpp"
  */
enum gn_predefined {
        BASICPRM,       ///< Probabalistic roadmap
        BASICOBPRM,     ///< Simple Binary Search to surface
        GN_USER1        ///< first user defined gn set, if any
};

//////////////////////////////////////////////////////////////////////////////////////////
//
//
// Algo base information data structures
//
//
//////////////////////////////////////////////////////////////////////////////////////////

struct GNInfo {

	/**
	  */
    SID gnsetid; 

    /**
	  */
    SID cdsetid;

    /**
	  */
    SID dmsetid;

	/**@name OBPRM Parameter*/
	//@{
	    /**Number of free Cfgs will be generated along a sample ray.
		  *A binary seacrh function, GenerateMapNodes::GenerateSurfaceCfg,
		  *generates a list of free Cfgs along a sample ray
		  *shooting from inside of C-Obstacle.
		  *This variable defines how many Cfgs will be extracted 
		  *from this list.
		  *@see GenerateMapNodes::Shells 
		  */
		int numShells;

		/**Used in OBPRM to generate Cfgs in C-Obstacle.
		  *This pair contains values which tell node generation
		  *methods what strategies will be used to find point in
		  *Robot and point in Obstacle. The first element in this 
		  *pair is for Robor and the second one is for Obstacle.
		  *Each element represents a way to find point.
		  *@see PairOptions for all possible ways and 
		  *GenerateMapNodes::GenerateSeeds for generating 
		  *Cfgs in C-Obstacles.
		  */
		n_str_param collPair;

		/**Used in OBPRM to generate collision free Cfgs.
		  *This pair contains values which tell node generation
		  *methods what strategies will be used to find point in
		  *Robot and point in Obstacle. The first element in this 
		  *pair is for Robor and the second one is for Obstacle.
		  *Each element represents a way to find point.
		  *@see PairOptions for all possible ways and
		  *GenerateMapNodes::GenFreeCfgs4Obst for generating free Cfgs.
		  */
		n_str_param freePair;

		/**Proportion of generated Cfgs are on Surface.
		  *This varible decides how many Cfgs will be 
		  *created from GenSurfaceCfgs4Obst and how many 
		  *Cfgs will be generated from GenFreeCfgs4Obst.
		  *
		  *@note this vaule should between [0,1]
		  *@see GenerateMapNodes::GenSurfaceCfgs4Obst,
		  *GenerateMapNodes::GenFreeCfgs4Obst and 
		  *GenerateMapNodes:OBPRM.
		  */
		double proportionSurface;
	//@}

	/**Will or won't generated clearance information
	  *for Cfgs in nodes.
	  *The calculated Cfg will put in InfoCfg::clearance.
	  *@see Cfg::info
	  */
    int calcClearance;

	/**Will or won't generated Cfgs add to Roadmap graph.
	  *If True, GenerateMapNodes::GenerateNodes
	  *will add generated free Cfgs to roadmap graph.
	  *Other generated Cfgs will be left in GNInfo::nodes.
	  */
    bool addNodes2Map; 

	/**Node generation functions in GenerateMapNodes add
	  *generated Cfgs in this list.
	  */
    vector<Cfg> nodes;

	///user indicated tag???
    int tag;

	/**Used for record collsion deteciotion inforamtion.
	  *This is used in like Cfg::ApproxCSpaceClearance
	  *, Cfg::GetFreeRandomCfg, and Cfg::isCollision
	  */
    CDInfo cdInfo;
};

/////////////////////////////////////////////////////////////////////
/**@class GN
  *
  *This class contains information relevant to a particular
  *map node generator (e.g., name, ptr to function, etc).
  */
/////////////////////////////////////////////////////////////////////

class GN;
class GNSets;
class GenerateMapNodes;

class Input;
class Roadmap;
class Environment;
class DistanceMetric;

///Pointer to map-node generating function
typedef void (*GNF) (Environment*, CollisionDetection*,DistanceMetric*,GN&, GNInfo&);

class GN {
  friend class GNSets;
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Constructors and Destructor*/
   //@{

	  /**Default constructor. 
	    *Initialize its data member to invalid values.
		*Setup descriptions for usage of GN.
		*
		*@note Default values for some parameters are setup in this method:
		*	-# GN::numNodes  falg="nodes"  default_value=10 MIN=1 MAX=5000000
        *	-# GN::numShells falg="shells" default_value=3  MIN=1 MAX=50
        *	-# GN::proportionSurface falg="pctSurf" default_value=1.0  MIN=0 MAX=1.0
        *	-# GN::collPair  falg="collPair" default_value="cM rT"
        *	-# GN::freePair  falg="freePair" default_value="cM rV"
        *	-# GN::clearanceFactor  falg="clearFact" default_value=1.0  MIN=0 MAX=1.0
        *	-# GN::gauss_d  falg="d" default_value=0  MIN=0 MAX=5000000
		*
		*@see num_param and n_str_param.
	    */
	  GN();
	  ///Destructor. Currently do nothing.
	  ~GN();

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Operator Overloading
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Operator Overloading*/
   //@{

	  /**Compare name of given GN with name of this instance.
		*@param _gn the name of this GN is compared with name of this instance.
		*Possible _gn's name are 
		*	-# BasicPRM
		*	-# BasicOBPRM
		*	-# OBPRM
		*	-# GaussPRM
		*	-# BasicMAPRM
                *       -# CSpaceMAPRM
		*@return True, if names of these two instance are the same. Otherwise False.
		*If this method could not recognize _gn's name then false will be returned.
		*/ 
	  bool operator==(const GN&) const;

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods.
    *All of these functions used for getting values of datamembers.
    */
  //@{

	  /**Get name of this GN instance.
		*@see GN::name
		*/
	  char*  GetName() const;

      ///Get generating map-node function.
	  GNF    GetGenerator();

	  ///Get elemet in of this GN element in universe.
	  EID    GetID() const;

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Usage Documentation Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Usage Documentation Methods.
    *Methods are used to print out incrementally useful documentation
    *messages about proper usage based on the type of "bad" usage observed.
    */
  //@{

	  /**Print out all information about 
	    *usage of GN.
		*This method posts information about how to
		*use, customize, or create a node generation function
		*and its parameters.
		*For example how to create a GN for Basic PRM or 
		*for MAPRM and how to setup correct parameters for these
		*specific GNs.
		*@see This method calls following methods:
		*	-# PrintUsage_BasicPRM
		*	-# PrintUsage_BasicOBPRM
		*	-# PrintUsage_OBPRM
		*	-# PrintUsage_GaussPRM
		*	-# PrintUsage_BasicMAPRM
                *       -# PrintUsage_CSpaceMAPRM
		*/ 
	  void  PrintUsage_All(ostream& _os);

	  /**Print out Basic PRM Usage to given output stream.
	    *@see GN::numNodes and num_param::PrintUsage
		*for underline PrintUsage
	    */
	  void  PrintUsage_BasicPRM(ostream& _os);

	 /**Print out Basic OBPRM Usage to given output stream.
	    *@see GN::numNodes, GN::numShells and 
		*num_param::PrintUsage for underline PrintUsage
	    */
	  void  PrintUsage_BasicOBPRM(ostream& _os);

	 /**Print out OBPRM Usage to given output stream.
	    *@see GN::numNodes, GN::numShells, GN::proportionSurface
		*, GN::collPair, GN::freePair, and GN::clearanceFactor.
		*num_param::PrintUsage and n_str_param::PrintUsage for
		*underline PrintUsage
	    */
	  void  PrintUsage_OBPRM(ostream& _os);

	 /**Print out Gaussian PRM Usage to given output stream.
	    *@see GN::numNodes, GN::gauss_d and 
		*num_param::PrintUsage for underline PrintUsage
	    */
	  void  PrintUsage_GaussPRM(ostream& _os);

	 /**Print out Basic MAPRM Usage to given output stream.
	    *@see GN::numNodes, GN::numShells and 
		*num_param::PrintUsage for underline PrintUsage
	    */
	  void  PrintUsage_BasicMAPRM(ostream& _os);

	 /**Print out CSpace MAPRM Usage to given output stream.
	    *@see GN::numNodes and
	        *num_param::PrintUsage for underline PrintUsage
	    */
	  void  PrintUsage_CSpaceMAPRM(ostream& _os);
	  
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
public:

	///Number of nodes to generate. For all node genration methods.
    num_param<int>    numNodes;

	/**@name BasicOBPRM and OBPRM Parameters*/
	//@{

	/**Number of shells to retain.
	  *@see GNInfo::numShells for detail.
      */
    num_param<int>    numShells;

	/**Proportion of free nodes to surface.
	  *@see GNInfo::proportionSurface for detail.
      */
    num_param<double> proportionSurface;

	/**(surface nodes) collision pair.
	  *@see GNInfo::collPair for detail.
      */
    n_str_param       collPair;

	/**(free nodes) free pair.
	  *@see GNInfo::freePair for detail.
      */
    n_str_param       freePair;

	/**Clearance is clearanceFactor*Position_Resoultion.
	  *Tell Node genration functions to generate free 
	  *Cfgs which have certain clearance.
	  *@see GenerateMapNodes::GenerateSurfaceCfg
	  */
    num_param<double> clearanceFactor; 
	//@}

	/**Distance from surface to retain Gausian
	  *@see GenerateMapNodes::GaussPRM
	  */
    num_param<double> gauss_d;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:
	char   name[80];		///< method name in character format
	GNF    generator;       ///< ptr to generator code
	EID    gnid;			///< Element id of this GN instance in univserse. (in GNSets)

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Private Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:

};

///Output name and type of given GN instance. Currently do nothing.
ostream& operator<< (ostream&, const GN& );

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//	class GNSets
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

/**This class is derived from BasicSets<GN>.
  *This class manages/contains:
  *        o a 'master' list of all generators, and
  *        o sets of generators
  *
  *     Each generator and set is given a unique id. The gn ids (EIDs)
  *     are used to compose labels encoding generator sets (used, e.g.,
  *     to record
  *
  */

class GNSets : public BasicSets<GN> {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destrcutor.
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor */
  //@{

	  ///Default constructor. Currently do nothing.
	  GNSets();
	  ///Destructor. Currently do nothing.
	  ~GNSets();

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Adding GNs, Making & Modifying GN sets
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Adding GNs, Making & Modifying GN sets */
  //@{

	   /**Add node generation info to universe.
	     *@note this method just adds GN instances to universe 
		 *, and no set will be created.
	     *@see MakeGNSet(const char*)
	     */
	   int AddGN(const char*);     // add gn(s) to universe

	   /**Add an element to (ordered) GN set.
	     *@see BasicSets::AddElementToOSet
	     */
	   int AddGNToSet(const SID, const EID);

	   /**Remove element from (ordered) GN set.
	     *@see BasicSets::DeleteElementFromOSet
	     */
	   int DeleteGNFromSet(const SID, const EID);

	   /**Read node generation info from a given string.
	     *@return INVALID_SID if istrstream for this given
		 *string could not be created.
		 *@see MakeGNSet(istream& _myistream)
	     */
	   SID MakeGNSet(const char*);

	   /**Read node generation info from input stream,
	     *create GN instances for these info, and make an (ordered) GN set for these.
		 *Accroding to read-in GN "names", following rules are applied.
		 *
		 *	-# BasicPRM
		 *		-# GN::numNodes = user spcified value 
		 *         (if no user spcified value, default value is 10 )
		 *
		 *	-# GaussPRM
		 *		-# GN::numNodes = user spcified value 
		 *         (if no user spcified value, default value is 10 )
		 *		-# GN::gauss_d = user spcified value
		 *         (if no user spcified value, default value is 0 )
		 *
		 *	-# BasicOBPRM
		 *		-# GN::numNodes = user spcified value 
		 *         (if no user spcified value, default value is 10 )
		 *		-# GN::numShells = user spcified value
		 *         (if no user spcified value, default value is 3 )
		 *
		 *	-# OBPRM
		 *		-# GN::numNodes = user spcified value 
		 *         (if no user spcified value, default value is 10 )
		 *		-# GN::numShells = user spcified value
		 *         (if no user spcified value, default value is 3 )
		 *		-# GN::proportionSurface = user spcified value
		 *         (if no user spcified value, default value is 1.0 )
		 *		-# GN::freePair = user spcified value
		 *         (if no user spcified value, default value is cM rV )
		 *		-# GN::collPair = user spcified value
		 *         (if no user spcified value, default value is cM rT )
		 *		-# GN::clearanceFactor = user spcified value
		 *         (if no user spcified value, default value is 1.0 )
		 *
		 *	-# BasicMAPRM
		 *		-# GN::numNodes = user spcified value 
		 *         (if no user spcified value, default value is 10 )
		 *
		 *      -# CSpaceMAPRM
		 *              -# GN::numNodes = user specified value
		 *         (if no user specified value, default value is 10 )
		 *
		 *@return SID of new set if every thing is OK. Otherwise, process will be terminiated.
	     *@see BasicSets::MakeOSet 
	     */ 
	   SID MakeGNSet(istream&);     

	   /**Make a new (ordered) GN set with element _eid.
	     *@see BasicSets::MakeOSet(const EID _eid)
	     */
	   SID MakeGNSet(const EID _eid);

	   /**Make a new (ordered) GN cd set with a list of elements in _eidvector.
		 *@see BasicSets::MakeOSet 
	     */
	   SID MakeGNSet(const vector<EID> _eidvector);

	   /**Remove a (ordered) GN set from universe.
	     *@see BasicSets::DeleteOSet
	     */
	   int DeleteGNSet(const SID _sid);

  //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Access Method (Getting Data & Statistics)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods.
	*Getting Data & Statistics 
	*/
   //@{
	   
	   /**Get a GN instance from universe.
	     *@note _gnid should be added to universe before.
	     *@see BasicSets::GetElement
	     */
	   GN GetGN(const EID _gnid ) const;

	   /**Get all GN instances in universe.
	     *@return a list of GN instances.
		 *@see BasicSets::GetElements
	     */
	   vector<GN> GetGNs() const;

	   /**Get a (ordered) GN set from universe.
	     *@note _sid should be created and added to universe before.
		 *@see BasicSets::GetOSet(const SID _sid)
	     */
	   vector<GN> GetGNSet(const SID _sid ) const;

	   /**Get all (ordered) GN set in universe.
	     *@return a list of (ordered) GN sets and their SIDs.
		 *Each (ordered) GN set contains a list of GN instances.
	     *@see BasicSets::GetOSets
	     */
	   vector<pair<SID,vector<GN> > > GetGNSets() const;

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	I/O Method (Display, Input, Output)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name I/O Methods.
     *Display, Input, Output 
     */
   //@{ 

	   /**Output all GN instances info in universe
	     *to standard output.
	     *@see BasicSets::DisplayElements.
	     */
	   void DisplayGNs() const;

	   /**Output GN instance info to standard output.
	     *@param _lpid Specify which GN instance should be printed.
	     *@see BasicSets::DisplayElement.
	     */
	   void DisplayGN(const EID ) const;

       /**Output information of all (ordered) GN sets in universe.
	     *@see BasicSets::DisplayOSets
	     */
	   void DisplayGNSets() const;

       /**Output information of (ordered) GN set with _sid.
	     *@see BasicSets::DisplayOSet
	     */
	   void DisplayGNSet(const SID ) const;

	   /**Ouput information about all GN instances to file.
	     *@param _fname filename for data file.
		 *@see WriteGNs(ostream& _myostream)
	     */
	   void WriteGNs(const char* _fname) const;

	   /**Ouput information about all GN instances to output stream.
	     *@note format: GN_NAME (a string) GN_PARMS (double, int, etc) 
		 *for each GN instance.
		 *	-# if GaussPRM is encountered, then GN::gauss_d will be 
		 *     printed.
		 *
		 *@see GetGNs 
	     */
	   void WriteGNs(ostream& ) const;

	   /**Read information about GN instances from file.
	     *@param _fname filename for data file.
		 *@see ReadGNs(istream& _myistream)
	     */
	   void ReadGNs(const char* _fname);

	   /**Read information about GN instances from input stream.
	     *This method reads and adds GN instances to universe.
		 *@see AddGN for add GN instances to universe and WriteGNs 
		 *for data format
		 *@note if this method could not be able to understand
		 *input file format, then error message will be post to 
		 *standard output.
	     */
	   void ReadGNs(istream& );

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Helper Method.
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Helper Methods.*/
   //@{ 

	  ///Setting up any default values for sets. Do nothing currnetly.
	  void   PutDefaults(Environment *_env);

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Private Data members and Member methods
  //
  //
  /////////////////////////////////////////////////////////////////////////////////////////
private:
};

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//	class GenerateMapNodes
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
/**This is the main generator class. It has an GNSets object
  *as a data element (list of gns and gn sets), and its methods
  *include the actual map node generator algorithms:
  *	- probabalistic roadmap (PRM)
  *	- simple binary search
  *
  * 'Wrapper' methods cycle thru sets of gns, stopping upon 1st
  * success or trying all.
  */

class GenerateMapNodes {

   friend SID MakeGNSet(istream&);

public:


  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{

	  ///Default Constructor. Call DefaultInit().
	  GenerateMapNodes();
	  ///Destructor. Currently do nothing.
	  ~GenerateMapNodes();

  //@}


  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Initialization functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Initialization Methods*/
  //@{

	  /**Initialize a datat member,GNInfo instance.
	    *Following rule are applied:
		*	-# GNInfo::gnsetid = BASICPRM
		*	-# GNInfo::cdsetid = CSTK or RAPID 
		*      (depends on USE_CSTK defined or not)
		*	-# GNInfo::dmsetid = S_EUCLID9
		*@see GNInfo
	    */
	  void DefaultInit();

	  /**Initialize this instance accroding to user input.
	    *Following GN Sets are created:
        *	- BasicPRM
		*	- BasicOBPRM
		*
		*In addition, Sets will be created according to user 
		*input strings. (one string for one set).
		*
		*GNInfo instance are also initailized here as following
		*	-# GNInfo::numShells = Input::numShells
        *	-# GNInfo::proportionSurface = Input::proportionSurface
		*	-# GNInfo::collPair = Input::collPair
		*	-# GNInfo::freePair = Input::freePair
		*	-# GNInfo::calcClearance = Input::calcClearance
		*	-# GNInfo::addNodes2Map = True
		*	-# GNInfo::tag = InfoCfg::NULL_INFO
		*
		*@note if user input strings are not empty, 
		*gnInfo.gnsetid will be set to GN_USER1
		*
		*@see GN::MakeGNSet(istream&) and ValidateParameters
        */
	  void UserInit(Input * input, Environment *_env);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Node Generation functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Node Generation.
    *High level node gernation methods.
	*may call more than one "Actual Node Generation Heuristic"
	*in sets 
	*/
  //@{

	  /**Generate nodes according to all GNs in the set.
	    *This is a high level view of GenerateNodes, which 
		*retrives actural node generation funtions from speicified
		*GNSet.
		*In this speicified GNSet, GN(s) contains pointer to 
		*a low level node generation funtion and parameters for
		*these low level funtions.
		*The job of this method is to:
		*	-# Get speicified GNSet
		*	-# Call low level node generation funtions in
		*      GNs in this speicified GNSet.
		*	-# if GNinfo::calcClearance is true
		*      Clearance for each generated Cfg is computed.
		*      otherwise Clearance for each generated Cfg is set
		*      to -1.
		*	-# if GNinfo::addNodes2Map is true
		*      Gfgs are inserted in Roadmap, _rm.
		*
		*@param _rm New created nodes will be added to this roadmap.
		*@param _gnsetid Which set of GN will be used in node generation time.
		*@param info Record node generation process infomation.
	    */
	  void GenerateNodes(Roadmap* _rm, CollisionDetection*,DistanceMetric*, SID, GNInfo&);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Staic Methods: CORE Node Generation Heuristics
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name CORE Node Generation Heuristics.
    *These fucntion provides specific node generations.
	*/
  //@{

	  /**Basic Randomized (probabilistic) Node Generation.
        *This method generates GN::numNodes collision-free Cfgs
		*and insert there Cfgs to GNInfo::nodes.
		*@param _env Used to get free Cfg.
		*@param cd Used to get free Cfg
		*@param _gn use _gn.numNodes to know how many free Cfgs will be 
		*           generated.
		*@param _info All generated free Cfg will be inserted to _info.nodes.
		*@see See Cfg::GetFreeRandomCfg to know how to generate "one" free Cfg.
		*@note If INTERMEDIATE_FILES is defined WritePathConfigurations will be
		*called.
	    */
	  static
	  void BasicPRM(Environment* _env,CollisionDetection* cd,DistanceMetric* dm, GN& _gn, GNInfo& );

	  /** Basic, no frills,  Obstacle Based Node Generation.
	    * 
		* Algorithm:
		*	-# For each Obstacle in Environement.
		*		-# Generate Cfg inside C-Obstacle
		*          (using GenerateInsideCfg)
		*		-# Generate a ray shooting from Cfg in 1.
		*          (using Cfg::GetRandomRay)
		*		-# Generate a Free Cfg alone the direction of ray in 2.
		*          (using GenerateOutsideCfg)
		*		-# Generate Surface Cfgs using Binary Search Procedure.
		*		-# Generate Shells (using Shells)
		*	-# End For
		*
		* when there is no Obstacle (i.e. only object in the Enviroment is 
		* Robot) GenCfgsFromCObst will be called instead of alg above.
		* @note number of free Cfgs genertated in this algorithm
		* will be GN::numNodes/(number_of_obstacle)/GN::numShells
		*
		* @param _env Environment for getting geometric information.
		* @param cd Used to get free Cfg (checking collision).
		* @param _gn use _gn.numNodes to know how many free Cfgs will be 
		*           generated and _gn.numShells to know how many free cfgs
		*           will be generated along a shooting ray.
		* @param _info All generated free Cfg will be inserted to _info.nodes.
		*
		* @see GN for parameter for BasicOBPRM, GenerateInsideCfg for generating
		* Cfg inside C-Obstacle, GenerateOutsideCfg for
		* generating Free Cfg, and GenerateSurfaceCfg
		* for Binary Search Procedure.
		*
		* @bug if number_of_obstacle is zero, above "equaltion" will cause
		* "divided by zero" run time error?!
	    */
	  static
	  void BasicOBPRM(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );

	  /** Obstacle Based Node Generation with some nifty enhancements such
	    * shells, alternative seed picking heuristics, and some free nodes.
		* 
		* Algorithm:
		*	-# For each Obstacle in Environement.
		*		-# Generate Surface Cfgs using Binary Search Procedure
		*          (using GenSurfaceCfgs4Obst)
		*		-# Generate Free Cfgs using Ad Hoc procedure
		*          (using GenFreeCfgs4Obst)
		*		-# Collect Free Cfgs found in 1 and 2.
		*	-# End For
		*
		* when there is no Obstacle (i.e. only object in the Enviroment is 
		* Robot) GenCfgsFromCObst will be called instead of alg above.
		* @note number of free Cfgs genertated in algorithm line 2
		* will be (P * GN::numNodes/(number_of_obstacle)/GN::numShells)
		* and number of free Cfgs genertated in algorithm line 3 will
		* be ((1-P) * GN::numNodes/(number_of_obstacle))
		* Here P is GN::proportionSurface.
		*
		* @param _env Environment for getting geometric information.
		* @param cd Used to get free Cfg (checking collision).
		* @param _gn use _gn.numNodes to know how many free Cfgs will be 
		*           generated and more.
		* @param _info All generated free Cfg will be inserted to _info.nodes.
		*
		* @see GN for parameter for OBPRM, GenSurfaceCfgs4Obst for generating
		* Surface Cfgs using Binary Search Procedure, GenFreeCfgs4Obst for
		* generating Free Cfgs using Ad Hoc procedure, and GenCfgsFromCObst
		* for robot only environment.
		*
		* @bug if number_of_obstacle is zero, above "equaltion" will cause
		* "divided by zero" run time error?!
	    */
	  static
	  void OBPRM(Environment* _env,CollisionDetection* cd,DistanceMetric* dm, GN& _gn, GNInfo& _info );

	  /**Filters randomly generated nodes in such a way that a "Gaussian" 
	    *distribution on obstacle surfaces is retained.
		*Brief Alg is:
		*	-# for i = 1 to n
        *		-# randomly generate cfg1
        *		-# randomly generate cfg2 distance of "d" away from cfg1
        *		-# if one of (cfg1,cfg2) is in collision and the other is not
        *			-# add the free one to the roadmap
        *		-# endif
		*	-# endfor
		*@param _env Used to get free Cfg.
		*@param cd Used to get free Cfg
		*@param _gn use _gn.numNodes to know how many free Cfgs will be 
		*           generated. _gn.gauss_d is "d" in algorithm above.
		*@param _info All generated free Cfg will be inserted to _info.nodes.
		*
		*@see See Cfg::GetRandomCfg to know how to generate "one" random Cfg.
		*See CollisionDetection::isCollision to check collsion.
		*
		*@note If INTERMEDIATE_FILES is defined WritePathConfigurations will be
		*called.
	    */
	  static
	  void GaussPRM(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );

	  /// Moves randomly generated nodes to some point on the medial axis
	  static
	  void BasicMAPRM(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );

	  static 
          void CSpaceMAPRM(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );
	  
  //@}


  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Staic Methods: Helper Methods for Heuristics 
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods for Heuristics */
  //@{

	  /**validate values specified for collision and free pairs.
		*@see ValidatePairs.
		*/
	  static
	  bool ValidateParameters(Input*);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Staic Methods: Helper Methods for MAPRM 
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods for MAPRM */
  //@{

	  static
	  void MoveToMedialAxis
	  (Cfg &cfg, vector<Cfg> *path, Environment *_env, 
	   CollisionDetection* cd, DistanceMetric *dm, GN& _gn, GNInfo &_info);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Staic Methods: Helper Methods for Baic OBPTM 
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods for MAPRM */
  //@{

	  /**Generate Cfg in C-Free but near Obstacle with clearanceFactor=1.
		*This method calls 
		*GenerateSurfaceCfg(Environment *,CollisionDetection *,
		*DistanceMetric *,GNInfo&,int,int,Cfg,Cfg, double)
		*with clearanceFactor=1.
		*/
	  static vector<Cfg>
	  GenerateSurfaceCfg(Environment *env,CollisionDetection *cd, DistanceMetric * dm,GNInfo& info,
					   int rob, int obst, Cfg insideCfg, Cfg outsideCfg);

	  /**Generate Cfg in C-Free but near Obstacle.
		*These Gfgs are created alone the line made by
		*insideCfg and outsideCfg. Algorithm is :
		*	-# do {
		*		-# mid = midpoint between in and out
		*		-# if mid is in C-Free then out=mid
		*		-# else in=mid
		*	-# } while dist(in,out) > clearance
		*Here dist is ditance between in and out.
		*clearance is clearanceFactor * PositionRes.
		*All midpoints which are in C-Free will be recorded.
		*@param rob index for robot in Environment.
		*@param obst index for obstacle in Environment.
		*@param insideCfg Cfg inside C-Obstacle
		*@param outsideCfg Cfg outside C-Obstacle.
		*@param clearanceFactor used to calculate cleanance between Cfg and Obstacle.
		*@return A list of midpoints which are in C-Free.
		*@see FirstFreeCfgs, GenerateInsideCfg, and GenerateOutsideCfg
		*@see called by BasicOBPRM, GenSurfaceCfgs4ObstVERTEX.
		*/
	  static vector<Cfg>
	  GenerateSurfaceCfg(Environment *env,CollisionDetection *cd, DistanceMetric * dm,GNInfo& info,
					   int rob, int obst, Cfg insideCfg, Cfg outsideCfg, double clearanceFactor);

  //@}


  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Staic Methods: Helper Methods for Advanced OBPTM 
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods for MAPRM */
  //@{

	  /**Generate Free Cfgs near the surface of Obstacle with clearanceFactor=1.
		*This method calls 
		*GenSurfaceCfgs4Obst(Environment *,CollisionDetection *, 
		*DistanceMetric *, int, int, GNInfo&, double) with clearanceFactor=1.
		*/
	  static vector<Cfg>
	  GenSurfaceCfgs4Obst(Environment * env,CollisionDetection *,
								DistanceMetric* dm, int obstacle, int nCfgs, 
								GNInfo &_info);

	  /**Generate Free Cfgs near the surface of Obstacle.
		*Get CollPair infomation by calling ValidatePairs for info.collPair.
		*	-# if Both element of CollPair is N_rT.
		*		-# then Cfg::GenSurfaceCfgs4ObstNORMAL will be called.
		*	-# othewise, GenSurfaceCfgs4ObstVERTEX will be called.
		*@return Return values returned by the function called by this method.
		*/
	  static vector<Cfg>
	  GenSurfaceCfgs4Obst(Environment * env,CollisionDetection *,
								DistanceMetric* dm, int obstacle, int nCfgs, 
								GNInfo &_info, double clearanceFactor);

	  /**Generate Free Cfgs near the surface of Obstacle by overlapping
		*point on the robot and point on the obstacle and 
		*with clearance 1.
		*This method calls 
		*GenSurfaceCfgs4ObstVERTEX(Environment *,CollisionDetection *,
		*DistanceMetric *, int, int,GNInfo&, double)
		*with clearanceFactor=1.
		*/
	  static vector<Cfg>
	  GenSurfaceCfgs4ObstVERTEX(Environment * env,CollisionDetection *,
								DistanceMetric* dm, int obstacle, int nCfgs, 
								GNInfo &_info);

	  /**Generate Free Cfgs near the surface of Obstacle by overlapping
		*point on the robot and point on the obstacle.
		*
		*This method uses different method to generate Cfg in C-Obs
		*instead of using GenerateInsideCfg. See GenerateSeeds for
		*more infomation about how to ues another way to generate Cfgs
		*inside CObstacle.
		*
		*After getting nCfgs Cfgs in side C-Obs, like BasicOBPRM,
		*Free Cfgs in around a obstacle are generated by calling 
		*GenerateSurfaceCfg and Shells.
		*@param obstacle index for obstacle in Environment.
		*@param nCfgs number of Free Cfgs near the surface
		*       will be generated.
		*@param clearanceFactor used to calculate cleanance between Cfg and Obstacle.
		*
		*@return a list of Free Cfgs near the surface of Obstacle.
		*
		*@see GenerateSurfaceCfg for binary seacrh of Cfgs, and GenerateSeeds
		*for generating Cfgs in C-Obstacle.
		*/
	  static vector<Cfg>
	  GenSurfaceCfgs4ObstVERTEX(Environment * env,CollisionDetection *,
								DistanceMetric* dm, int obstacle, int nCfgs, 
								GNInfo &_info, double clearanceFactor);
  //@}

protected:

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Staic Methods : Helper Methods for Basic OBPRM
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods for Basic OBPRM*/
  //@{
	  /** returns the first n free cfgs from a given vector of cfgs.  
		* Returns all free cfgs if there are less than n. 
		*@param n how many free Cfg in cfgs will be returned.
		*@param cfgs a list of Cfgs which will be examed in this method
		*to eact Free Cfgs.
		*/
	  static vector<Cfg>
	  FirstFreeCfgs(Environment *env,CollisionDetection *cd, vector<Cfg> cfgs, GNInfo &info, int n);

	  /**Return all free cfgs in the given vector of cfgs.
		*@see FirstFreeCfgs(Environment *,CollisionDetection *, vector<Cfg>, GNInfo&, int)
		*/
	  static vector<Cfg>
	  FirstFreeCfgs(Environment *env,CollisionDetection *cd, vector<Cfg> cfgs, GNInfo &info);

	    /**Get nshells Cfgs from given Cfg list.
		*@param nshells number of Cfgs that are going to
		*bereturned.
		*@note if the size of given list is smaller than nshells
		*then all element in the list will be returned.
		*@note if the size of given list is larger than nshells,
		*then one elemet will be return in every (size/nshells)
		*elemets along the given list.
		*/
	  static vector<Cfg> Shells(vector<Cfg> cfgs, int nshells);

	  /**Get Cfg that is in side C-Obstacle.
		*This is done by following:
		*	-# Move the center point of robot to the
		*      center point of the obststacle.
		*	-# if first step does not generate
		*      Cfg in side C-Obs then, the center point of 
		*      robot will be moved to any point on the
		*      surface of obstacle.
		*@param rob index for robot in Environment.
		*@param obst index for obstacle in Environment.
		*@param inside The gernerated Cfg inside C-Obstacle.
		*@return Always return true
		*@see GenerateOutsideCfg
		*/
	  static bool
	  GenerateInsideCfg(Environment*, CollisionDetection* cd, 
						int rob, int obst, Cfg* inside, GNInfo &_info);

	  /**Get Cfg that is in CFree accroding to given
		*Cfg in C-Obs and incremental value.
		*This is done by increasing Cfg in C-Obs
		*in given ray.
		*
		*@param rob index for robot in Environment.
		*@param obst index for obstacle in Environment.
		*@param InsideNode The Cfg which is inside C-Obstacle.
		*@param incrCfg This a ray in CSpace. This method will
		*       inf Cfg in Cfree in this given direction.
		*
		*@return Cfg in free space if this method found one. Otherwise
		*InsideNode will be returned.
		*@note this method will try 500 times to find free Cfg in
		*given direction.
		*@see GenerateInsideCfg
		*/
	  static Cfg
	  GenerateOutsideCfg(Environment*, CollisionDetection*,
						int rob, int obst, Cfg InsideNode, Cfg incrCfg, GNInfo&);

	  
	  /**Check if all Cfgs in cfgs are in bouding box of Environment.
		*@param cfgs a list of parameters.
		*@param env Provides bouding box information.
		*@return True if all Cfgs in cfgs are in side bouding box.
		*@see Cfg::InBoundingBox
		*/
	  static vector<Cfg>
	  InsideBoundingBox(Environment *env, vector<Cfg> cfgs);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Staic Methods : Helper Methods for Advanced OBPRM
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods for Advanced OBPRM */
  //@{

	  /// generates random cfgs within a specified area around a given cfg
	  static void
	  Spread(Cfg, double, double, int, vector<Cfg>* );

	  /**Generate seeds in specified (user or default) manner.
		*Another way to generate Cfgs in C-Obstacle.
		*Follow algorithm is applied to get one such Cfg:
		*	-# Generate a point, r, on robot.
		*	-# Generate a point, o, on obstacle.
		*	-# Move robot such that r and o are overlap.
		*	-# Rotate robot randomly.
		*	-# if this Cfg causes collision, then insert
		*      this Cfg to seeds
		*nseeds Cfg(s) will be generated
		*
		*@param obst index for obstacle MultiBody in Envoriment.
		*@param nseeds number of seed that will be generated.
		*@param selectRobot PairOptions
		*@param selectObstacle PairOptions
		*@param seeds generated Cfgs
		*
		*@see PointsOnMultiBody to get point on the MultiBody.
		*Cfg::GenerateOverlapCfg to get overlap Cfg.
		*/
	  static void
	  GenerateSeeds(Environment * env,CollisionDetection *cd, GNInfo &_gnInfo,
				  int obst, int nseeds,
				  int selectRobot, int selectObstacle,
				  vector<Cfg>* seeds);

	  /**Generate several points on the given MultiBody.
		*@param mbody a pointer to MultiBody from where 
		*       points will be generated.
		*@param npts How many points are going to be generated.
		*@param select which method are going to be uesd.
		*
		*@note for robot(which have freebodys),
		*sample points on last link only. for obstacle, 
		*sample first link.
		*
		*@return a list of generated points on the mbody.
		*@see PointOnBody
		*/
	  static vector<Vector3D>
	  PointsOnMultiBody(MultiBody * mbody, int npts, int select);

	  /**Generate Free Cfgs which are near C-Obstacles with 
		*clearanceFactor=1.
		*This method calls 
		*GenCfgsFromCObst(Environment *,CollisionDetection *,
		*DistanceMetric *, int, int,GNInfo&, double)
		*with clearanceFactor=1.
		*/
	  static vector<Cfg>
	  GenCfgsFromCObst(Environment * env,CollisionDetection* cd,
						DistanceMetric * dm, int obstacle, int nCfgs, 
						GNInfo &_info);
  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Staic Methods : OBPRM Special Case -> For Signle Object Planning
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name For Signle Object Environment.
    *These methods are the case that only object
	*in environemnt is robot.
    */
  //@{
	  /**Generate Free Cfgs which are near C-Obstacles.
		*This method is used to generate free Cfgs for
		*the Environment with only one MultiBody, which
		*is robot. Therefore, there is no obstacle in
		*the world. Due to self-collision, there are 
		*C-Obstacles in C-Space.
		*
		*Following Algorithm is used:
		*	-# randomly generate the orienatation of Robot.
		*	-# if this Cfg causes self-collision.
		*		-# save this Cfg to obs_seed list
		*	-# else save to (Free Cfg) return list.
		*	-# Repeat above nCfgs times.
		*	-# for each Cfg, seedCfg, in obs_seed
		*		-# Generate a ray emanating out from seedCfg
		*		-# Find a Cfg free in direction of this ray
		*		-# GenerateSurfaceCfg, binary search for free Cfgs
		*		-# Get info.numShells Cfgs as Shell
		*		-# insert sell to return list.
		*	-# end for
		*	-# return "return list"
		*
		*@param obstacle index for obstacle MultiBody in Envoriment.
		*@param nCfgs number of free Cfgs that will be generated.
		*@param clearanceFactor used to calculate cleanance between 
		*Cfg and CObstacle.
		*@return a list of Free Cfgs which are near C-Obstacles.
		*/
	  static vector<Cfg>
	  GenCfgsFromCObst(Environment * env,CollisionDetection* cd,
						DistanceMetric * dm, int obstacle, int nCfgs, 
						GNInfo &_info, double clearanceFactor);

	  /**Generate Free Cfgs in specified (user or default) manner.
		*Another way to generate Cfgs around C-Obstacle.
		*Follow algorithm is applied to get one such Cfg:
		*	-# Generate a point, r, on robot.
		*	-# Generate a point, o, on obstacle.
		*	-# Move robot such that r and o are overlap.
		*	-# Rotate robot randomly.
		*	-# if this Cfg is collision free, then insert
		*      this Cfg to return list.
		* "nCfgs" Cfg(s) will be generated
		*
		*@param obstacle index for obstacle MultiBody in Envoriment.
		*@param nCfgs number of free Cfgs that will be generated.
		*@param _info this method uses _info.freePair information
		*to generate points in robot and obstacle. 
		*_info.freePair.first is for robot and _info.freePair.second
		*is for obstacle.
		*
		*
		*@see PointsOnMultiBody to get point on the MultiBody.
		*Cfg::GenerateOverlapCfg to get overlap Cfg.
		*@see GenerateSeeds for generating Cfg in C-Obs in
		*similar mannar.
		*/
	  static vector<Cfg>
	  GenFreeCfgs4Obst(Environment * env,CollisionDetection *,
						int obstacle, int nCfgs, GNInfo &_info);
  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Staic Methods : Helper Methods for Collision and Free Pair (Adv OBPRM)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods for Collision and Free Pair */
  //@{

	  /**Any two of these are valid values for collision and free pairs
		*@see PointsOnMultiBody
		*/
	  enum PairOptions
	  {
		  cM,	///< Center of Mass
		  rV,	///< Random Vertex
		  rT,	///< point in Random Triangle
		  rE,	///< Random Extreme vertex
		  rW,	///< point in Random Weighted triangle 
		  cM_rV,///< Center of Mass or Random Vertex
		  rV_rT,///< random Vertex or point in random Triangle
		  rV_rW,///< random Vertex or point in random Weighted triangle
		  N_rT, ///< Normal of a random Triangle
		  all,  ///< all of the basics
		  INVALID_OPTION	///< This means somthing "Bad"...
	  };

	  /**Character specification of PairOptions must be converted to enum.
		*For example string "cM" convert to cM. string "all" convert to all.
		*
		*@return INVALID_OPTION in no coorsponding PairOptions value in found.
		*Otherwise one of PairOptions values will be returned.
		*
		*@note Error message will be post if this method does not know how
		*to convert.
		*
		*@param mnemonic The string which is going to be convert to PairOptions value.
		*@param PairOptions used for print out error message.
		*
		*@warning Return value of this method actually is a interger number, which
		*coorsponds to the order in PairOptions. If the order of PairOptions changed
		*The returned value of this method should be changed, too.
		*/
	  static
	  int TranslateOptionCode(char* mnemonic, n_str_param param);

	  /**validate values specified for collision *or* free pairs
		*Call TranslateOptionCode to translate user input from string
		*to PairOptions value. 
		*@param params Contains two string values which will be converted
		*to PairOptions values.
		*@param results A pair of PairOptions values converted from 
		* two string values in params.
		*@param msg used as error message.
		*@return false if one of two strings are converted to INVALID_OPTION.
		*Otherwise true will be returned.
		*
		*@see TranslateOptionCode
		*/
	  static
	  bool ValidatePairs(char *msg, n_str_param params, pair<int,int> * results);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Staic Methods: Random Sampling in Body
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Random Sampling in Body
    *Radomly choose a point inside or
	*on the surface of given Body,
    */
  
	  /**Get a point on the Body accroding specified method.
		*@param body a pointer to Body where a point will be generated.
		*@param isFreeBody 1 is this body is free body.
		*@param select one of PairOptions values. Defines which way this
		*new point should be generated.
		*@note Accroding to select value following methods are called.
		*	-# cM Body::GetCenterOfMass()
		*	-# rV ChooseRandomVertexOnBody
		*	-# rT ChooseRandomTriangleOnBody
		*	-# rE ExtremeVertex
		*	-# rW ChooseRandomWeightedTriangleOnBody
		*	-# cM_rV GetCenterOfMass or ChooseRandomVertexOnBody
		*      (either on, randomly choosed)
		*	-# rV_rT ChooseRandomVertexOnBody or ChooseRandomTriangleOnBody
		*      (either on, randomly choosed)
		*	-# rV_rW ChooseRandomVertexOnBody or 
		*      ChooseRandomWeightedTriangleOnBody 
		*      (either on, randomly choosed)
		*	-# all One of above. Randomly choosed.
		*	-# none of above exit will be called.
		*@return a point which is generated by one of above method.
		*/
	  static Vector3D
	  PointOnBody(Body * body, int select, bool isFreeBody);

	  //@{
		/**Choose a random vertex on given Body.
		*
		*@param isFreeBody Check if this is a Body of robot.
		*
		*@note if this is a Body of robot, GetPolyhedron,
		*which returns vertex in local coordinate system, will
		*be called to get robot's geometry. Otherwise,
		*GetWorldPolyhedron will be called to get obstacle's
		*geometry in world coordinate system.
		*
		*@see Body::GetPolyhedron, Body::GetWorldPolyhedron,
		*and GMSPolyhedron::vertexList.
		*/
	  static Vector3D
	  ChooseRandomVertexOnBody(Body *body, bool isFreeBody);


	  /**Choose a randome point inside a given triangle.
		*This triangle is made by 3 vertices, p, q, and r.
		*This mehotd randomly generates Barycentric coordinate
		*for this return point.
		*@see This funtion is called by ChooseRandomTriangleOnBody.
		*/
	  static Vector3D
	  ChoosePointOnTriangle(Vector3D p, Vector3D q, Vector3D r);

	  /**Choose a random point inside a random trianagle on the
		*given Body.
		*
		*@param isFreeBody Check if this is a Body of robot.
		*
		*@note if this is a Body of robot, GetPolyhedron,
		*which returns vertex in local coordinate system, will
		*be called to get robot's geometry. Otherwise,
		*GetWorldPolyhedron will be called to get obstacle's
		*geometry in world coordinate system.
		*
		*@see Body::GetPolyhedron, Body::GetWorldPolyhedron,
		*GMSPolyhedron::vertexList, and GMSPolyhedron::polygonList.
		*/
	  static Vector3D
	  ChooseRandomTriangleOnBody(Body*, bool);

	  /**Choose a random extreme vertex on given Body.
		*A extreme vertex is a vertex has max/min coordinate
		*in X, Y, or Z direction.
		*One of extreme vertices on this body will be returned.
		*
		*@param isFreeBody Check if this is a Body of robot.
		*
		*@note if this is a Body of robot, GetPolyhedron,
		*which returns vertex in local coordinate system, will
		*be called to get robot's geometry. Otherwise,
		*GetWorldPolyhedron will be called to get obstacle's
		*geometry in world coordinate system.
		*
		*@see Body::GetPolyhedron, Body::GetWorldPolyhedron,
		*and GMSPolyhedron::vertexList.
		*/
	  static Vector3D
	  ExtremeVertex(Body*, bool isFreeBody);

	  /**Choose a random point inside a random weighted trianagle 
		*on the given Body.
		*Weighted trianagle is a trianagle weighted by its area. 
		*If the size of a trianagle triangle is big, this 
		*trianagle has big chance to be chose.
		*
		*@param isFreeBody Check if this is a Body of robot.
		*
		*@note if this is a Body of robot, GetPolyhedron,
		*which returns vertex in local coordinate system, will
		*be called to get robot's geometry. Otherwise,
		*GetWorldPolyhedron will be called to get obstacle's
		*geometry in world coordinate system.
		*
		*@see Body::GetPolyhedron, Body::GetWorldPolyhedron,
		*GMSPolyhedron::vertexList, and GMSPolyhedron::polygonList.
		*/
	  static Vector3D
	  ChooseRandomWeightedTriangleOnBody(Body *body, bool isFreeBody);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
public:
  GNSets generators;  ///< actual generators
  GNInfo gnInfo;      ///< information passes to and from generators

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  /// max. # of attempts at surface convergence
  static const int MAX_CONVERGE;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Private Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};

//---------------------------------------------------------------
// Ran3 -- "Global" UNIFORM Random number function
// This ought to be placed elsewhere, but at the moment
// it - along with Medial Axis is "in development"
// so it is stuck here - do NOT count on it remaining here !!!
// The code comes from Numerical Recipes in C, pages 274 - 285
//---------------------------------------------------------------
#define MBIG    1000000000
#define MSEED    161803398
#define MZ      0
#define FAC ((float)1.0 / MBIG)

float Ran3(long *idum);

#endif
