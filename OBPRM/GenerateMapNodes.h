// $Id$
/////////////////////////////////////////////////////////////////////
//
//  GenerateMapNodes.h
//
//  General Description
//      This set of classes supports a "RoadMap Node Generation Algobase".
//      Generate roadmap nodes and enter each as a graph node.
//
//      The classes in the set are:
//        o GN            -- info related to an individual method of node
//                              generation
//        o GNSets        -- contains 'master' list of all generation methods
//                           and maintains sets of generation methods
//                           (derived from BasicSets<GN>)
//        o GenerateMapNodes
//                        -- has GNSets as data element, and its methods
//                           include the actual generation algorithms.
//                           A 'wrapper' method cycles thru sets of gn's.
//
//       Each GN element is given a unique id, which is used to compose
//              hmmm, I don't know what...
//
//  Created
//      8/27/98  Lucia K. Dale
//
//  Last Modified
//      8/21/99  Lucia K. Dale  added GaussPRM
//
/////////////////////////////////////////////////////////////////////

#ifndef GenerateMapNodes_h
#define GenerateMapNodes_h

#include "OBPRM.h"
#include "Sets.h"

#include "util.h"
#include "Input.h"
#include "CollisionDetection.h"

#include "Debug.h"

#include <stdio.h>
#include <stdlib.h>

#include <iomanip.h>
#include <fstream.h>
#include <iostream.h>
#include <strstream.h>

#include <vector.h>


//---------------------------------------------------------------
//  Pre-defined Algobase Sets
//  CAUTION:  DO NOT CHANGE THE ENUMERATION ORDERS w/o CHANGING
//              SET DEFN's in "GenerateMapNodes.c"
//---------------------------------------------------------------
enum gn_predefined {    //----------------
        BASICPRM,       // Probabalistic roadmap
        BASICOBPRM,     // Simple Binary Search to surface
        GN_USER1};      // first user defined gn set, if any

//---------------------------------------------------------------
// Algo base information data structures
//---------------------------------------------------------------

struct GNInfo {
    SID gnsetid;        // generator set id
    SID cdsetid;        // collision detection set id
    SID dmsetid;        // distance metric set id
    int numNodes;
    int numNodesPerObst;
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

typedef void (*GNF) (Environment*, CollisionDetection*,DistanceMetric*,GN&, GNInfo&);  // pointer to gn function


/////////////////////////////////////////////////////////////////////
//  class GN
//
//  General Description
//     This class contains information relevant to a particular
//     map node generator (e.g., name, ptr to function, etc).
//
//
/////////////////////////////////////////////////////////////////////
class GN {
  friend class GNSets;
public:

  //===================================================================
  // Constructors and Destructor
  //===================================================================
  GN();
  ~GN();

  //===================================================================
  // Operators
  //===================================================================
  bool operator==(const GN&) const;

  //===================================================================
  // Other Methods
  //===================================================================
  char*  GetName() const;
  GNF    GetGenerator();
  EID    GetID() const;
  double Get_Gauss_d() const;
  double Get_clearanceFactor() const;

protected:
  //===================================================================
  // Data
  //===================================================================
  char   name[80];
  GNF    generator;          // ptr to generator code
  EID    gnid;
  double Gauss_d;
  double clearanceFactor;

private:

};

ostream& operator<< (ostream&, const GN& );



/////////////////////////////////////////////////////////////////////
//  class GNSets
//
//  General Description
//     This class is derived from BasicSets<GN>.
//
//     This class manages/contains:
//        o a 'master' list of all generators, and
//        o sets of generators
//
//     Each generator and set is given a unique id. The gn ids (EIDs)
//     are used to compose labels encoding generator sets (used, e.g.,
//     to record
//
/////////////////////////////////////////////////////////////////////
class GNSets : public BasicSets<GN> {
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
    GNSets();
    ~GNSets();

  //===================================================================
  //  Other Methods
  //===================================================================

        // Setting up any default values for sets
  void   PutDefaults(Environment *_env);

        // Adding GNs, Making & Modifying GN sets
   int AddGN(const char*);     // add gn(s) to universe
   int AddGNToSet(const SID, const EID);
   int DeleteGNFromSet(const SID, const EID);

   SID MakeGNSet(const char*);  // make an ordered set of gns,
   SID MakeGNSet(istream&);     //  - add gn to universe if not there
   SID MakeGNSet(const EID);
   SID MakeGNSet(const vector<EID> );

   int DeleteGNSet(const SID _sid);

        // Getting Data & Statistics
   GN GetGN(const EID ) const;
   vector<GN> GetGNs() const;
   vector<GN> GetGNSet(const SID ) const;
   vector<pair<SID,vector<GN> > > GetGNSets() const;

        // Display, Input, Output
   void DisplayGNs() const;
   void DisplayGN(const EID ) const;
   void DisplayGNSets() const;
   void DisplayGNSet(const SID ) const;

   void WriteGNs(const char* _fname) const;
   void WriteGNs(ostream& ) const;
   void ReadGNs(const char* _fname);
   void ReadGNs(istream& );

  //===================================================================
  //  Data
  //===================================================================
protected:
private:
  double DEFAULT_Gauss_d;
};

/////////////////////////////////////////////////////////////////////
//  class GenerateMapNodes
//
//  General Description
//     This is the main generator class. It has an GNSets object
//     as a data element (list of gns and gn sets), and its methods
//     include the actual map node generator algorithms:
//        o probabalistic roadmap (PRM)
//        o simple binary search
//
//     'Wrapper' methods cycle thru sets of gns, stopping upon 1st
//     success or trying all.
//
/////////////////////////////////////////////////////////////////////

class GenerateMapNodes {
   friend SID MakeGNSet(istream&);
public:
  //===================================================================
  // Constructors and Destructor
  //===================================================================
  GenerateMapNodes();
  ~GenerateMapNodes();

  //===================================================================
  // Other Methods
  //===================================================================

  void DefaultInit();
  void UserInit(Input * input, Environment *_env);

  void GenerateNodes(Roadmap*, CollisionDetection*,DistanceMetric*,SID, GNInfo&);

  static
  void BasicPRM
	(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );

  static
  void BasicOBPRM
	(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );

  static
  void OBPRM
       	(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );

  static
  void GaussPRM
	(Environment*,CollisionDetection*,DistanceMetric* dm, GN&, GNInfo& );

  static
  bool ValidateParameters(Input*);

  static
  vector<Cfg>
  GenerateSurfaceCfg(Environment*, CollisionDetection *,
			DistanceMetric*,GNInfo& ,
                   int, int, Cfg, Cfg);

  static vector<Cfg>
  GenerateSurfaceCfg(Environment*, CollisionDetection*,DistanceMetric*,GNInfo&,
                   int, int, Cfg, Cfg,double);

  static vector<Cfg>
  GenSurfaceCfgs4Obst(Environment * env,CollisionDetection *,DistanceMetric* dm, int obstacle, int nCfgs, GNInfo &_info);

  static vector<Cfg>
  GenSurfaceCfgs4Obst(Environment * env,CollisionDetection *,DistanceMetric* dm, 
    int obstacle, int nCfgs, GNInfo &_info, double clearanceFactor);

  static vector<Cfg>
  GenSurfaceCfgs4ObstVERTEX(Environment * env,CollisionDetection *,DistanceMetric* dm, int obstacle, int nCfgs, GNInfo &_info);

  static vector<Cfg>
  GenSurfaceCfgs4ObstVERTEX(Environment * env,CollisionDetection *,DistanceMetric* dm, 
    int obstacle, int nCfgs, GNInfo &_info, double clearanceFactor);

//===================================================================
// protected method implementations
//===================================================================
protected:

  enum PairOptions {cM,rV,rT,rE,rW,cM_rV,rV_rT,rV_rW,N_rT,all,INVALID_OPTION};

  static
  int TranslateOptionCode(char*, n_str_param);

  static
  bool ValidatePairs(char*, n_str_param, pair<int,int>*);

  static void
  GenerateSeeds(Environment*, CollisionDetection *,GNInfo&,
		int, int, int, int, vector<Cfg>*);

  static void
  Spread(Cfg, double, double, int, vector<Cfg>* );

  //-----------------------------------------
  // used by "FarthestFromStart" for sort
  //-----------------------------------------
  typedef pair<int,double> VID_DISTANCE_TYPE;
  static bool DIST_Compare (const VID_DISTANCE_TYPE&, const VID_DISTANCE_TYPE&);

  static void
  FarthestFromStart(Environment*, DistanceMetric*,
		GNInfo&, Cfg, vector<Cfg>, vector<Cfg>*);

  static void
  FirstNFreeCfgs(Environment*, CollisionDetection *,GNInfo&,
		 int, vector<Cfg>, vector<Cfg>*);

  static void
  GenNewPivots(Environment*, CollisionDetection *cd,DistanceMetric*,GNInfo&,
        Cfg, vector<Cfg>, double, double, int, int, vector<Cfg>*);

  static void
  SpreadCfg(Environment*, CollisionDetection *cd,DistanceMetric*,GNInfo&,
            Cfg, double, double, int, int, int);

  static vector<Cfg>
  FreeCfgs(Environment*, CollisionDetection *cd,vector<Cfg>, GNInfo&);

  static vector<Cfg>
  InsideBB(Environment*, vector<Cfg>);

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
  GenerateInsideCfg(Environment*, CollisionDetection* cd, int, int, Cfg*, GNInfo &_info);

  static Cfg
  GenerateOutsideCfg(Environment*, CollisionDetection*,int, int, Cfg, Cfg, GNInfo&);

  static vector<Cfg>
  GenCfgsFromCObst(Environment * env,CollisionDetection* cd,DistanceMetric * dm, int obstacle, int nCfgs, GNInfo &_info);

  static vector<Cfg>
  GenCfgsFromCObst(Environment * env,CollisionDetection* cd,DistanceMetric * dm, 
     int obstacle, int nCfgs, GNInfo &_info, double clearanceFactor);

  static vector<Cfg>
  GenFreeCfgs4Obst(Environment * env,CollisionDetection *,
			 int obstacle, int nCfgs, GNInfo &_info);


  //===================================================================
  // Data
  //===================================================================

public:
  GNSets generators;
  GNInfo gnInfo;

protected:
//brc added
  static const int MAX_CONVERGE = 20;

private:
};

#endif
