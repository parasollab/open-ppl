///////////////////////////////////////////////////////////////
//
//	PotentialForceGrids.h
//
//	General Description:
//	   This class builds or reads in a Potential/Force grids 
//	   around a protein. It returns force/potential for a 
//	   given configuration of a specified ligand molecule.
//
//	References on PDB format:
//		http://www.umass.edu/microbio/rasmol/pdb.htm
//              http://www.rcsb.org/pdb/info.html#File_Formats_and_Dictionaries
//
//	Author:
//		Guang Song	06/19/2000
//
/////////////////////////////////////////////////////////////////
#ifndef PotentialForceGrids_h
#define PotentialForceGrids_h


#include<stdlib.h>
#include<stdio.h>
#include<iostream.h>
#include<fstream.h>
#include<string.h>
#include<vector.h>
#include<math.h>

#include<Vectors.h>


#define PI 3.1415926
#define radDeg (180/3.1415926)
enum GridType {FORCE, POTENTIAL, FORCEANDPOTENTIAL};
enum AtomType {Nitrogen, Carbon, Oxygen, Sulfar};

class PotentialForceGrids {

public:
    PotentialForceGrids(char * tableName);
    PotentialForceGrids(char * proteinFilename, GridType _gt, int _dim, double _gridSize);
    ~PotentialForceGrids();

    Vector3D GetForce(vector<Vector3D> ligandAtoms) const;
    double GetPotential(vector<Vector3D> ligandAtoms) const;
    double GetPotential(const vector<Vector3D> &ligandAtoms, 
		        const vector<char> &atomtype) const;
    double GetRealPotential(const vector<Vector3D> &ligandAtoms,
                        const vector<char> &atomtype) const;

    void ReadPDB(char * pdbName, vector<Vector3D> &coordinates, vector<AtomType> &atomtype) ;
    void SetLigandFileName(char *ligandName);   
    void ReadTable(char * tableName);
    void WriteTable();
    void CalForceTable();
    void CalPotentialTable();



    //////////////////////////////////////////
    //   Data 
    //////////////////////////////////////////
    typedef double AtomCharge;

    vector<Vector3D> proteinCoordinates, ligandCoordinates;
    vector<AtomType> proteinAtomType,    ligandAtomType;
    vector<AtomCharge>  proteinAtomCharge, ligandAtomCharge;

    char proteinPDBname[80];
    char ligandPDBname[80];

    GridType gridtype;
    double gridSize;
    int dim;
    int siz;

    double A[4][4], B[4][4];


    //------- table/grid stuff ---------------------
    //int tableAvailable;

    double *Npotential, *Opotential,
	   *Cpotential, *Spotential;

    Vector3D *Nforce, *Oforce, *Cforce, *Sforce;


};

#endif
