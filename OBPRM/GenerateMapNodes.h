// $Id$

/////////////////////////////////////////////////////////////////////
/**@file GenerateMapNodes.h
        This set of classes supports a "RoadMap Node Generation Algobase".
        Generate roadmap nodes and enter each as a graph node.
  
        The classes in the set are:
          o GN            -- info related to an individual method of node
                                generation
          o GNSets        -- contains 'master' list of all generation methods
                             and maintains sets of generation methods
                             (derived from BasicSets<GN>)
          o GenerateMapNodes
                          -- has GNSets as data element, and its methods
                             include the actual generation algorithms.
                             A 'wrapper' method cycles thru sets of gn's.
  
         Each GN element is given a unique id, which is used to compose
                hmmm, I don't know what...
  
    @author Lucia K. Dale
    @date   8/27/98
*/
/////////////////////////////////////////////////////////////////////

#ifndef GenerateMapNodes_h
#define GenerateMapNodes_h

#include "OBPRM.h"
#include "Sets.h"

#include "util.h"
#include "Input.h"
#include "CollisionDetection.h"

#include <stdio.h>
#include <stdlib.h>

#include <iomanip.h>
#include <fstream.h>
#include <iostream.h>
#include <strstream.h>

#include <vector.h>

#define MAX_NODES           5000000
#define MAX_NODES_PER_OBST 5000

//---------------------------------------------------------------
/** Pre-defined Algobase Sets
    @warning DO NOT CHANGE THE ENUMERATION ORDERS w/o CHANGING
                SET DEFN's in "GenerateMapNodes.c"
*/
//---------------------------------------------------------------
enum gn_predefined {    //----------------
        BASICPRM,       // Probabalistic roadmap
        BASICOBPRM,     // Simple Binary Search to surface
        GN_USER1};      // first user defined gn set, if any

//---------------------------------------------------------------
/// Algo base information data structure
//---------------------------------------------------------------
struct GNInfo {
    SID gnsetid;        // generator set id
    SID cdsetid;        // collision detection set id
    SID dmsetid;        // distance metric set id
    int numShells;
    int calcClearance;
    bool addNodes2Map;
    n_str_param collPair;
    n_str_param freePair;
    double proportionSurface;
    vector<Cfg> nodes;
    int tag;
    CDInfo cdInfo;
};

class GN;
class GNSets;
class GenerateMapNodes;

class Input;
class Roadmap;
class Environment;
class DistanceMetric;

/// pointer to gn function
typedef void (*GNF) (Environment*, CollisionDetection*,DistanceMetric*,GN&, GNInfo&);


/////////////////////////////////////////////////////////////////////
/** class GN
    This class contains information relevant to a particular
    map node generator (e.g., name, ptr to function, etc).
*/
/////////////////////////////////////////////////////////////////////
class GN {
  friend class GNSets;
public:

  //===================================================================
  /**@name Constructors and Destructor */
  //===================================================================
  //@{
  ///Default constructor. Initialize its data member to invalid values.
  GN();
  ///Destructor. Currently do nothing.
  ~GN();
  //@}

  //===================================================================
  /**@name Operator overloading*/
  //===================================================================
  //@{
  /**Compare name of given GN with name of this instance.
    @param _gn the name of this GN is compared with name of this instance.
    @return True, if names of these two instance are the same. Otherwise False.
    */
  bool operator==(const GN&) const;
  //@}

  //===================================================================
  /**@name Access Methods
    All of these functions used for getting values of datamembers.
    */
  //===================================================================
  //@{
  char*  GetName() const;
  GNF    GetGenerator();
  EID    GetID() const;
  //@}

  //===================================================================
  /**@name Usage Documentation Methods
    Methods are used to print out incrementally useful documentation
    messages about proper usage based on the type of "bad" usage observed.
    */
  //===================================================================
  //@{
  void  PrintUsage_All(ostream& _os);
  void  PrintUsage_BasicPRM(ostream& _os);
  void  PrintUsage_BasicOBPRM(ostream& _os);
  void  PrintUsage_OBPRM(ostream& _os);
  void  PrintUsage_GaussPRM(ostream& _os);
  void  PrintUsage_BasicMAPRM(ostream& _os);
  //@}

public:
  //===================================================================
  /**@name Data
    All of these functions used for getting values of datamembers.
  //===================================================================
    */
  //@{
    num_param<int>    numNodes;    /// number of nodes to generate
    num_param<int>    numShells;   /// number of shells to retain
    num_param<double> proportionSurface; /// proportion of free nodes to surface
    n_str_param       collPair;    /// (surface nodes) collision pair
    n_str_param       freePair;    /// (free nodes)     free pair
    num_param<double> clearanceFactor; 
    num_param<double> gauss_d;     /// distance from surface to retain Gausian

protected:
  char   name[80];           // method name in character format
  GNF    generator;          // ptr to generator code
  EID    gnid;
  //@}

private:

};

ostream& operator<< (ostream&, const GN& );



/////////////////////////////////////////////////////////////////////
/**This class is derived from BasicSets<GN>.
   This class manages/contains:
          o a 'master' list of all generators, and
          o sets of generators
  
       Each generator and set is given a unique id. The gn ids (EIDs)
       are used to compose labels encoding generator sets (used, e.g.,
       to record
  
  */
/////////////////////////////////////////////////////////////////////
class GNSets : public BasicSets<GN> {
public:

  //===================================================================
  /**@name Constructors and Destructor */
  //===================================================================
  //@{
  ///Default constructor. Initialize its data member to invalid values.
  GNSets();
  ///Destructor. Currently do nothing.
  ~GNSets();
  //@}

  //===================================================================
  //  Other Methods
  //===================================================================

   /**@name Adding GNs, Making & Modifying GN sets */
   //@{
   int AddGN(const char*);     // add gn(s) to universe
   int AddGNToSet(const SID, const EID);
   int DeleteGNFromSet(const SID, const EID);

   SID MakeGNSet(const char*);  // make an ordered set of gns,
   SID MakeGNSet(istream&);     //  - add gn to universe if not there
   SID MakeGNSet(const EID);
   SID MakeGNSet(const vector<EID> );

   int DeleteGNSet(const SID _sid);
   //@}

   /// Setting up any default values for sets
  void   PutDefaults(Environment *_env);

   /**@name Getting Data & Statistics */
   //@{ 
   GN GetGN(const EID ) const;
   vector<GN> GetGNs() const;
   vector<GN> GetGNSet(const SID ) const;
   vector<pair<SID,vector<GN> > > GetGNSets() const;
   //@}

   /**@name Display, Input, Output */
   //@{ 
   void DisplayGNs() const;
   void DisplayGN(const EID ) const;
   void DisplayGNSets() const;
   void DisplayGNSet(const SID ) const;

   void WriteGNs(const char* _fname) const;
   void WriteGNs(ostream& ) const;
   void ReadGNs(const char* _fname);
   void ReadGNs(istream& );
   //@}

  //===================================================================
  // Data
  //===================================================================
public:
protected:
private:
};

/////////////////////////////////////////////////////////////////////
/** class GenerateMapNodes
    General Description
       This is the main generator class. It has an GNSets object
       as a data element (list of gns and gn sets), and its methods
       include the actual map node generator algorithms:
          o probabalistic roadmap (PRM)
          o simple binary search
  
       'Wrapper' methods cycle thru sets of gns, stopping upon 1st
       success or trying all.
*/  
/////////////////////////////////////////////////////////////////////
class GenerateMapNodes {
   friend SID MakeGNSet(istream&);
public:


  //===================================================================
  /**@name Constructors and Destructor */
  //===================================================================
  //@{
  ///Default constructor. Initialize its data member to invalid values.
  GenerateMapNodes();
  ///Destructor. Currently do nothing.
  ~GenerateMapNodes();
  //@}

  //===================================================================
  /**@name Initialization Methods */
  //===================================================================
  //@{
  void DefaultInit();
  void UserInit(Input * input, Environment *_env);
  //@}

  //===================================================================
  /**@name Drivers for Node Generation (may call more than one`
     "Actual Node Generation Heuristic" in sets */
  //===================================================================
  //@{
  void GenerateNodes(Roadmap*, CollisionDetection*,DistanceMetric*,
                     SID, GNInfo&);
  //@}

  //===================================================================
  /**@name Actual Node Generation Heuristics */
  //===================================================================
  //@{
  /// Basic Randomized (probabilistic) Node Generation
  static
  void BasicPRM
	(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );
  /// Basic, no frills,  Obstacle Based Node Generation 
  static
  void BasicOBPRM
	(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );
  static
  /** Obstacle Based Node Generation  with some nifty enhancements such
  shells, alternative seed picking heuristics, and some free nodes
  */
  void OBPRM
       	(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );
  /** Filters randomly generated nodes in such a way that a "Gaussian" 
   distribution on obstacle surfaces is retained.
  */
  static
  void GaussPRM
	(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );
  /// Moves randomly generated nodes to some point on the medial axis
  static
  void BasicMAPRM
	(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );
  //@}



  //===================================================================
  /**@name Helper Methods for Heuristics (public) */
  //===================================================================
  //@{

  /// validate values specified for collision and free pairs
  static
  bool ValidateParameters(Input*);

  static
  void MoveToMedialAxis
  (Cfg &cfg, vector<Cfg> *path, Environment *_env, 
   CollisionDetection* cd, DistanceMetric *dm, GN& _gn, GNInfo &_info);

  static vector<Cfg>
  GenerateSurfaceCfg(Environment*, CollisionDetection *,
			    DistanceMetric*, GNInfo&, int, int, Cfg, Cfg);

  static vector<Cfg>
  GenerateSurfaceCfg(Environment*, CollisionDetection*,
                            DistanceMetric*,GNInfo&, int, int, Cfg, Cfg,double);

  static vector<Cfg>
  GenSurfaceCfgs4Obst(Environment * env,CollisionDetection *,
                            DistanceMetric* dm, int obstacle, int nCfgs, 
                            GNInfo &_info);

  static vector<Cfg>
  GenSurfaceCfgs4Obst(Environment * env,CollisionDetection *,
                            DistanceMetric* dm, int obstacle, int nCfgs, 
                            GNInfo &_info, double clearanceFactor);

  static vector<Cfg>
  GenSurfaceCfgs4ObstVERTEX(Environment * env,CollisionDetection *,
                            DistanceMetric* dm, int obstacle, int nCfgs, 
                            GNInfo &_info);

  static vector<Cfg>
  GenSurfaceCfgs4ObstVERTEX(Environment * env,CollisionDetection *,
                            DistanceMetric* dm, int obstacle, int nCfgs, 
                            GNInfo &_info, double clearanceFactor);
  //@}

  //===================================================================
  /**@name Helper Methods for Heuristics (protected) */
  //===================================================================
  //@{
protected:

  /// any two of these are valid values for collision and free pairs
  enum PairOptions {cM,rV,rT,rE,rW,cM_rV,rV_rT,rV_rW,N_rT,all,INVALID_OPTION};

  /// character specification of PairOptions must be converted to enum
  static
  int TranslateOptionCode(char*, n_str_param);

  /// validate values specified for collision *or* free pairs
  static
  bool ValidatePairs(char*, n_str_param, pair<int,int>*);

  /// generate seeds in specified (user or default) manner
  static void
  GenerateSeeds(Environment*, CollisionDetection *,GNInfo&,
		int, int, int, int, vector<Cfg>*);

  /// generates random cfgs within a specified area around a given cfg
  static void
  Spread(Cfg, double, double, int, vector<Cfg>* );

  /** returns the first n free cfgs from a given vector of cfgs.  Returns all
  free cfgs if there are less than n. */
  static vector<Cfg>
  FirstFreeCfgs(Environment*, CollisionDetection*, vector<Cfg>, GNInfo&, int);

  /// return all free cfgs in the given vector of cfgs.
  static vector<Cfg>
  FirstFreeCfgs(Environment*, CollisionDetection*,vector<Cfg>, GNInfo&);

  /// given a vector of cfg's return only those which are in the bounding box
  static vector<Cfg>
  InsideBoundingBox(Environment*, vector<Cfg>);

  static Vector3D
  ChooseRandomVertexOnBody(Body*, bool);

  static Vector3D
  ChoosePointOnTriangle(Vector3D p, Vector3D q, Vector3D r);

  static Vector3D
  ChooseRandomTriangleOnBody(Body*, bool);

  static Vector3D
  ExtremeVertex(Body*, bool isFreeBody);

  static Vector3D
  ChooseRandomWeightedTriangleOnBody(Body *body, bool isFreeBody);

  static Vector3D
  PointOnBody(Body*, int, bool);

  static vector<Vector3D>
  PointsOnMultiBody(MultiBody*, int, int);

  static vector<Cfg>
  Shells(vector<Cfg>, int);

  static bool
  GenerateInsideCfg(Environment*, CollisionDetection* cd, 
                    int, int, Cfg*, GNInfo &_info);

  static Cfg
  GenerateOutsideCfg(Environment*, CollisionDetection*,
                    int, int, Cfg, Cfg, GNInfo&);

  static vector<Cfg>
  GenCfgsFromCObst(Environment * env,CollisionDetection* cd,
                    DistanceMetric * dm, int obstacle, int nCfgs, 
                    GNInfo &_info);

  static vector<Cfg>
  GenCfgsFromCObst(Environment * env,CollisionDetection* cd,
                    DistanceMetric * dm, int obstacle, int nCfgs, 
                    GNInfo &_info, double clearanceFactor);

  static vector<Cfg>
  GenFreeCfgs4Obst(Environment * env,CollisionDetection *,
                    int obstacle, int nCfgs, GNInfo &_info);
  //@}

  //===================================================================
  /**@name Data */
  //===================================================================
public:
  GNSets generators;  /// actual generators
  GNInfo gnInfo;      /// information passes to and from generators

protected:
  /// max. # of attempts at surface convergence
  static const int MAX_CONVERGE = 20;  

private:
  //@}
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
