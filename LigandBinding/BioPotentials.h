// $Id$
/////////////////////////////////////////////////////////////////////
//
//  BioPotentials.h
//
//  General Description
//      Potential functions & parameters to be used to computational
//	biology simulation: e.g. protein folding, molecular docking.
//      
//
//
//
//  Created
//      06/13/2000      Guang Song
//
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////

#ifndef BioPotentials_h
#define BioPotentials_h

#include"Cfg.h"
#include"PotentialForceGrids.h"
class Environment;

enum PotentialType {FullMainChain, SideChainSameSphereHbonds, LigandBinding,
		    BindingExactPotential};

class BioPotentials {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================
  BioPotentials();
  ~BioPotentials();

  static bool isInPotentialRangeOfNodeGeneration(const Cfg &c, Environment * env);
  static double ConnectionThreshold();

  // driver function
  static double GetPotential(const Cfg &c, Environment * env);

  static double PotentialFullMainChain(const Cfg &c, Environment * env);
  static double PotentialSideChainSameSphereHbonds(const Cfg &c, Environment * env);
  static double PotentialLigandBinding(const Cfg &c, Environment * env);
  static double PotentialBindingExact(const Cfg &c, Environment * env);

  //static double PotentialFromHydrogenBondsAndssBonds(const Cfg &c, Environment * env);
  static double PotentialFromHydrogenBondsAndssBonds(vector<Vector3D> coordinates);
  //double PotentialVirtualBonds(...);
  //double PotentialFromTinker();

  // driver funtion
  static vector<Vector3D> GetCoordinatesPotential(const Cfg &c, Environment * env);
  static vector<Vector3D> GetCoordinatesForRMSD(const Cfg &c, Environment * env);
  static vector<Vector3D> GetCoordinatesLigandBinding(const Cfg &c, Environment * env);

protected:

  static vector<Vector3D> GetCoordinatesFullMainChain(Environment * env);
  static vector<Vector3D> GetCoordinatesSideChainSameSphereHbonds(Environment * env); // not defined yet.

  //===================================================================
  //  data (potential parameters)
  //===================================================================
public:
  static double gnEmin;  // Emin at node generation
  static double gnEmax;  // Emax at node generation
  static double cnEmax;  // the maximum potential allowed during connection.


  static int HbondsNum;
  static int SSbondsNum;
  static vector< pair<int, int> > HbondPair;
  static vector< pair<int, int> > SSbondPair;

  static double vdwSphereRadiusMax;
  static double vdwSphereRadiusMin;

  static PotentialType potentialType;

  static vector<int> torsionAngle; // units: degree. Using 'int' ok.

  // these two are for ligand binding
  static vector<vector<Vector3D> > atomCoord;
  static vector<char> atomType;
  static PotentialForceGrids * potentialGrid;
};

#endif
