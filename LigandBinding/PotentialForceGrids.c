///////////////////////////////////////////////////////////////
//
//      PotentialForceGrids.c
//
//      General Description:
//         This class builds or reads in a Potential/Force grids
//         around a protein. It returns force/potential for a
//         given configuration of a specified ligand molecule.
//
//      References on PDB format:
//              http://www.umass.edu/microbio/rasmol/pdb.htm
//              http://www.rcsb.org/pdb/info.html#File_Formats_and_Dictionaries
//
//      Author:
//              Guang Song      06/19/2000
//
/////////////////////////////////////////////////////////////////


#include"PotentialForceGrids.h"


PotentialForceGrids::PotentialForceGrids(char * tableName) {
  strcpy(ligandPDBname, "");
  strcpy(proteinPDBname, "");
  double AA[4] = {3952850, 1200965, 145834, 3075695};
  double BB[4] = {2556, 425, 328, 953};

  for(int i=0; i<4; ++i) {
     for(int j=0; j<4; ++j) {
        A[i][j] = sqrt(AA[i]*AA[j]);
        B[i][j] = sqrt(BB[i]*BB[j]);
     }
  }


  ReadTable(tableName);
}

PotentialForceGrids::PotentialForceGrids(char * proteinFilename, GridType _gt,
int _dim, double _gridSize) {

  strcpy(proteinPDBname, proteinFilename);
  strcpy(ligandPDBname, "");
  gridtype = _gt;
  gridSize = _gridSize;
  dim = _dim;

  double AA[4] = {3952850, 1200965, 145834, 3075695};
  double BB[4] = {2556, 425, 328, 953};

  for(int i=0; i<4; ++i) {
     for(int j=0; j<4; ++j) {
        A[i][j] = sqrt(AA[i]*AA[j]);
        B[i][j] = sqrt(BB[i]*BB[j]);
     }
  }

#if 0
  siz = 2*dim + 1;
  if(gridtype == POTENTIAL || gridtype == FORCEANDPOTENTIAL) { 
     Npotential = new double[siz*siz*siz];
     Opotential = new double[siz*siz*siz];
     Cpotential = new double[siz*siz*siz];
     Spotential = new double[siz*siz*siz];
  } else {
     Npotential = Cpotential = Opotential = Spotential = NULL;
  }
  if(gridtype == FORCE || gridtype == FORCEANDPOTENTIAL) { 
     Nforce = new Vector3D[siz*siz*siz];
     Oforce = new Vector3D[siz*siz*siz];
     Cforce = new Vector3D[siz*siz*siz];
     Sforce = new Vector3D[siz*siz*siz];
  } else {
     Nforce = Oforce = Cforce = Sforce = NULL;
  }
#endif
  
  ReadPDB(proteinFilename, proteinCoordinates, proteinAtomType);
  //CalPotentialTable();
  //CalForceTable();

}

PotentialForceGrids::~PotentialForceGrids() {}

void PotentialForceGrids::ReadPDB(char * pdbName, vector<Vector3D> &coordinates, 
vector<AtomType> &atomtype) {
  ifstream is(pdbName);
  if(!is) {
        cout << "file " << pdbName << "  does not exists! " << endl;
        exit(4);
  }

  char line[90];
  char recordName[6];
  vector<Vector3D> coord;
  while(is.getline(line, 90, '\n')) {
     strncpy(recordName, line, 6);
     if(!strncmp(recordName, "ATOM  ", 6)) {
         char atomName[4], resName[3], x[8], y[8], z[8];
         double point[3];
	 int i;
         for(i=0; i<4; ++i)
            atomName[i] = line[12+i];
         for(i=0; i<3; ++i)
            resName[i] = line[17+i];
         for(i=0; i<8; ++i) {
            x[i] = line[30+i];
            y[i] = line[38+i];
            z[i] = line[46+i];
         }
         sscanf(x, "%lf", &point[0]);
         sscanf(y, "%lf", &point[1]);
         sscanf(z, "%lf", &point[2]);

         //cout << atomName << "111" << endl;
         if(!strncmp(atomName, " N", 2))
            atomtype.push_back(Nitrogen);
         else if(!strncmp(atomName, " C", 2))
            atomtype.push_back(Carbon);
         else if(!strncmp(atomName, " O", 2))
            atomtype.push_back(Oxygen);
         else if(!strncmp(atomName, " S", 2))
            atomtype.push_back(Sulfar);
         else {
           cout << "error: unknown atom type:" << atomName << endl;
           exit(1);
         }
         coord.push_back(Vector3D(point[0], point[1], point[2]));
     }

  }
  Vector3D sum(0,0,0);
  int i;
  for(i=0; i<coord.size(); ++i) {
        sum = sum + coord[i];
  }
  Vector3D com = sum/coord.size();
  for(i=0; i<coord.size(); ++i) {
       coord[i] = coord[i] - com;
  }

  coordinates = coord;
   
}

void PotentialForceGrids::CalPotentialTable() {

  if(gridtype == FORCE) return;
  for(int i=-dim; i<=dim; ++i) {
     for(int j=-dim; j<=dim; ++j) {
        for(int k=-dim; k<=dim; ++k) {
	   int index = (i+dim)*siz*siz + (j+dim)*siz + (k+dim);
           Npotential[index] = 0;
           Opotential[index] = 0;
           Cpotential[index] = 0;
	   Spotential[index] = 0;

           Vector3D tmpR(i*gridSize, j*gridSize, k*gridSize);

           for(int m=0; m<proteinCoordinates.size(); ++m) {

                Vector3D R = tmpR - proteinCoordinates[m];
                double r = R.magnitude();
		if(r < 0.001)
		   continue;
                double r6 = pow(r,6);
                double r12 = r6*r6;

                AtomType at = proteinAtomType[m];

                Npotential[index] += A[at][Nitrogen]/r12 - B[at][Nitrogen]/r6;
                Opotential[index] += A[at][Oxygen]/r12 - B[at][Oxygen]/r6;
                Cpotential[index] += A[at][Carbon]/r12 - B[at][Carbon]/r6;
		Spotential[index] += A[at][Sulfar]/r12 - B[at][Sulfar]/r6;


           }
        }
     }
  }

}



void PotentialForceGrids::CalForceTable() {
  if(gridtype == POTENTIAL ) return;

  for(int i=-dim; i<=dim; ++i) {
     for(int j=-dim; j<=dim; ++j) {
        for(int k=-dim; k<=dim; ++k) {
	   int index = (i+dim)*siz*siz + (j+dim)*siz + (k+dim);
           Nforce[index] = Vector3D(0,0,0);
           Oforce[index] = Vector3D(0,0,0);
           Cforce[index] = Vector3D(0,0,0);
	   Sforce[index] = Vector3D(0,0,0);

           Vector3D tmpR(i*gridSize, j*gridSize, k*gridSize);

           for(int m=0; m<proteinCoordinates.size(); ++m) {

                Vector3D R = tmpR - proteinCoordinates[m];
                double r = R.magnitude();
		if(r < 0.001)
		   continue;
                double r6 = pow(r,6);
                double r12 = r6*r6;

                AtomType at = proteinAtomType[m];

		Vector3D direct = R*(1.0/r/r);
		Nforce[index] = Nforce[index] +
			direct*(12*A[at][Nitrogen]/r12 - 6*B[at][Nitrogen]/r6);
		Oforce[index] = Oforce[index] +
                        direct*(12*A[at][Oxygen]/r12 - 6*B[at][Oxygen]/r6);
		Cforce[index] = Cforce[index] +
                        direct*(12*A[at][Carbon]/r12 - 6*B[at][Carbon]/r6);
		Sforce[index] = Sforce[index] +
                        direct*(12*A[at][Sulfar]/r12 - 6*B[at][Sulfar]/r6);


           }
        }
     }
  }

}

void PotentialForceGrids::ReadTable(char *tableName) {

   ifstream is(tableName); 

   int gt;
   is >> gt;
   switch(gt) {
      case 0:
	gridtype = FORCE;
	break;
      case 1:
	gridtype = POTENTIAL;
	break;
      case 2:
	gridtype = FORCEANDPOTENTIAL;
	break;
   }
	  
   is >> dim;
   is >> gridSize;

   siz = 2*dim + 1;
   if(gridtype == POTENTIAL || gridtype == FORCEANDPOTENTIAL) {
     Npotential = new double[siz*siz*siz];
     Opotential = new double[siz*siz*siz];
     Cpotential = new double[siz*siz*siz];
     Spotential = new double[siz*siz*siz];
   } else {
     Npotential = Cpotential = Opotential = Spotential = NULL;
   }
   if(gridtype == FORCE || gridtype == FORCEANDPOTENTIAL) {
     Nforce = new Vector3D[siz*siz*siz];
     Oforce = new Vector3D[siz*siz*siz];
     Cforce = new Vector3D[siz*siz*siz];
     Sforce = new Vector3D[siz*siz*siz];
   } else {
     Nforce = Oforce = Cforce = Sforce = NULL;
   }


   if(gridtype == POTENTIAL || gridtype == FORCEANDPOTENTIAL) {
      for(int i=-dim; i<=dim; ++i) {
         for(int j=-dim; j<=dim; ++j) {
            for(int k=-dim; k<=dim; ++k) {
		int index = (i+dim)*siz*siz + (j+dim)*siz + (k+dim);
		is >> Npotential[index];
		is >> Opotential[index];
		is >> Cpotential[index];
		is >> Spotential[index];
	    }
	 }
       }
    }

    if(gridtype == FORCE || gridtype == FORCEANDPOTENTIAL) {
      for(int i=-dim; i<=dim; ++i) {
         for(int j=-dim; j<=dim; ++j) {
            for(int k=-dim; k<=dim; ++k) {
		int index = (i+dim)*siz*siz + (j+dim)*siz + (k+dim);
                is >> Nforce[index][0] >> Nforce[index][1] >> Nforce[index][2];
                is >> Oforce[index][0] >> Oforce[index][1] >> Oforce[index][2];
                is >> Cforce[index][0] >> Cforce[index][1] >> Cforce[index][2];
                is >> Sforce[index][0] >> Sforce[index][1] >> Sforce[index][2];
            }   
         }
       }
    }

}

void PotentialForceGrids::WriteTable() {
   char tmp[80];
   sprintf(tmp, "%s%s", proteinPDBname, ".gridTable");
   ofstream os(tmp);

   os << gridtype << "\n";
   os << dim << "\n";
   os << gridSize << "\n";

   if(gridtype == POTENTIAL || gridtype == FORCEANDPOTENTIAL) {
      for(int i=-dim; i<=dim; ++i) {
         for(int j=-dim; j<=dim; ++j) {
            for(int k=-dim; k<=dim; ++k) {
		int index = (i+dim)*siz*siz + (j+dim)*siz + (k+dim);
		os << Npotential[index] << "\n";
		os << Opotential[index] << "\n";
		os << Cpotential[index] << "\n";
		os << Spotential[index] << "\n";
	    }
	 }
       }
    }

    os << "\n\n";

    if(gridtype == FORCE || gridtype == FORCEANDPOTENTIAL) {
      for(int i=-dim; i<=dim; ++i) {
         for(int j=-dim; j<=dim; ++j) {
            for(int k=-dim; k<=dim; ++k) {
		int index = (i+dim)*siz*siz + (j+dim)*siz + (k+dim);
                os << Nforce[index][0] << "  " << Nforce[index][1]
		   << "  " << Nforce[index][2] << "\n";
                os << Oforce[index][0] << "  " << Oforce[index][1]
		   << "  " << Oforce[index][2] << "\n";
                os << Cforce[index][0] << "  " << Cforce[index][1]
		   << "  " << Cforce[index][2] << "\n";
                os << Sforce[index][0] << "  " << Sforce[index][1]
		   << "  " << Sforce[index][2] << "\n";
            }   
         }
       }
    }

}

void PotentialForceGrids::SetLigandFileName(char *ligandName) {
   strcpy(ligandPDBname, ligandName);
   ReadPDB(ligandName, ligandCoordinates, ligandAtomType);
} 

double PotentialForceGrids::GetPotential(vector<Vector3D> ligandAtoms) const {

   if(gridtype == FORCE) {
	cout << "no potential grids available, build that first!" << endl;
	exit(5);
   }

   if(! ligandPDBname ) {
      cout << "\nLigand is not defined! Use SetLigandFileName(...) to set Ligand first!" << endl;
      exit(1);
   } else if ( ligandAtoms.size() != ligandCoordinates.size() ) {
      cout << "\nInput ligand number does NOT match that in ligand PDB file" << endl;
      exit(2);
   }
  
   double potential = 0;
   for(int i=0; i<ligandAtoms.size(); ++i) {
     int p[3];
     for(int j=0; j<3; ++j) {
	p[j] = rint(ligandAtoms[i][j]/gridSize);
	if(p[j] > dim ) 
	   p[j] = dim;
	else if(p[j] < -dim) 
	   p[j] = -dim;	
     }
     int index = (p[0]+dim)*siz*siz + (p[1]+dim)*siz + (p[2]+dim);
     switch(ligandAtomType[i]) {
	case Nitrogen:
	   potential += Npotential[index];
	   break;
	case Carbon:
	   potential += Cpotential[index];
	   break;
	case Oxygen:
	   potential += Opotential[index];
	   break;
	case Sulfar:
	   potential += Spotential[index];
	   break;
	default:
	   break;
    }
  }
  return potential;
}


double PotentialForceGrids::GetPotential(const vector<Vector3D> &ligandAtoms,
const vector<char> &atomtype) const {

   if(gridtype == FORCE) {
	cout << "no potential grids available, build that first!" << endl;
	exit(5);
   }
  
   double potential = 0;
   for(int i=0; i<ligandAtoms.size(); ++i) {
     int p[3];
     for(int j=0; j<3; ++j) {
	p[j] = rint(ligandAtoms[i][j]/gridSize);
	if(p[j] > dim ) 
	   p[j] = dim;
	else if(p[j] < -dim) 
	   p[j] = -dim;	
     }
     int index = (p[0]+dim)*siz*siz + (p[1]+dim)*siz + (p[2]+dim);
     switch(atomtype[i]) {
	case 'N':
	   potential += Npotential[index];
	   break;
	case 'C':
	   potential += Cpotential[index];
	   break;
	case 'O':
	   potential += Opotential[index];
	   break;
	case 'S':
	   potential += Spotential[index];
	   break;
	default:
	   break;
    }
  }
  return potential;
}

double PotentialForceGrids::GetRealPotential(const vector<Vector3D> &ligandAtoms,
const vector<char> &atomtype) const {

   vector<AtomType> vat;
   int i;
   for(i=0; i<ligandAtoms.size(); ++i) {
      switch(atomtype[i]) {
        case 'N':
           vat.push_back(Nitrogen);
           break;
        case 'C':
           vat.push_back(Carbon);
           break;
        case 'O':
           vat.push_back(Oxygen);
           break;
        case 'S':
           vat.push_back(Sulfar);
           break;
	//case 'H':
	//   vat.push_back(Hydrogen);
	//   break;
        default:
	   cout << "unknown atom type: " << atomtype[i] << endl;
           break;
      }
   }


   double potential = 0;

   for(i=0; i<ligandAtoms.size(); ++i) {
      for(int m=0; m<proteinCoordinates.size(); ++m) {
                Vector3D R = ligandAtoms[i] - proteinCoordinates[m];
                double r = R.magnitude();
                if(r < 0.001)
                   return 1000000;
                double r6 = pow(r,6);
                double r12 = r6*r6;

                AtomType at1 = proteinAtomType[m];
		AtomType at2 = vat[i];

                potential += A[at1][at2]/r12 - B[at1][at2]/r6;
      }
   }
   return potential;
}



Vector3D PotentialForceGrids::GetForce(vector<Vector3D> ligandAtoms) const {

   if(gridtype == POTENTIAL) {
        cout << "no force grids available, build that first!" << endl;
        exit(5);
   }

   if(! ligandPDBname ) {
      cout << "\nLigand is not defined! Use SetLigandFileName(...) to set Ligand first!" << endl;
      exit(1);
   } else if ( ligandAtoms.size() != ligandCoordinates.size() ) {
      cout << "\nInput ligand number does NOT match that in ligand PDB file" << endl;
      exit(2);
   }

   Vector3D force(0,0,0);
   for(int i=0; i<ligandAtoms.size(); ++i) {
     int p[3];
     for(int j=0; j<3; ++j) {
        p[j] = rint(ligandAtoms[i][j]/gridSize);
        if(p[j] > dim ) 
           p[j] = dim;
        else if(p[j] < -dim)    
           p[j] = -dim; 
     }
     int index = (p[0]+dim)*siz*siz + (p[1]+dim)*siz + (p[2]+dim);
     switch(ligandAtomType[i]) {
        case Nitrogen:
           force = force + Nforce[index];
           break;
        case Carbon:
	   force = force + Cforce[index];
           break;
        case Oxygen:
           force = force + Oforce[index];
           break;
        case Sulfar:
           force = force + Sforce[index];
           break;
        default:
	   break;
    }   
  }
  return force;
}

