// $Id$
/////////////////////////////////////////////////////////////////////
//
//  BioPotentials.c
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


#include"BioPotentials.h"
#include"Environment.h"
#include"Cfg.h"
#include"CfgManager.h"

#define MAXPOTENTIAL 9999999.9

// default values for potential parameters, etc. 
PotentialType BioPotentials::potentialType = FullMainChain;

double BioPotentials::gnEmin = 200.0; // kJ/mols
double BioPotentials::gnEmax = 500.0;
double BioPotentials::cnEmax = 3000.0;

double BioPotentials::vdwSphereRadiusMax = 2.4; // Angstrom
double BioPotentials::vdwSphereRadiusMin = 1.0; // Angstrom

int BioPotentials::HbondsNum = 0;
int BioPotentials::SSbondsNum = 0;
vector< pair<int, int> > BioPotentials::HbondPair; // = NULL;
vector< pair<int, int> > BioPotentials::SSbondPair; // = NULL;
vector<int> BioPotentials::torsionAngle; 

// for ligand binding
vector<vector<Vector3D> > BioPotentials::atomCoord;
vector<char> BioPotentials::atomType;
PotentialForceGrids * BioPotentials::potentialGrid = NULL;
///////////////////////////////////////////////////////


BioPotentials::BioPotentials() {};

BioPotentials::~BioPotentials() {};


bool BioPotentials::isInPotentialRangeOfNodeGeneration(const Cfg &c, Environment * env) {

   double potential = GetPotential(c, env);
cout << "potential is " << potential << endl;
   if(potential < gnEmin )
       return false;
   else if(potential < gnEmax && (potential-gnEmin)/(gnEmax-gnEmin) < drand48() )
       return false;

   return true;
}

double BioPotentials::ConnectionThreshold() {
   return cnEmax;
}

double BioPotentials::GetPotential(const Cfg &c, Environment * env) {
  if(potentialType == FullMainChain) 
     return PotentialFullMainChain(c,env);
  else if(potentialType == SideChainSameSphereHbonds) 
     return PotentialSideChainSameSphereHbonds(c,env);
  else if(potentialType == LigandBinding) 
     return PotentialLigandBinding(c,env);
  else if(potentialType == BindingExactPotential) {
     return PotentialBindingExact(c,env);
  } else {
     cout << "Error: in BioPotentials::GetPotential. Unknown Potential type:" 
	  << potentialType << ". " << endl;
     cout << "Possible Potential Type: FullMainChain -- " << FullMainChain << "\n" 
	  << "                         SideChainSameSphereHbonds -- " << 
	    SideChainSameSphereHbonds << endl;
     exit(10);
  }

}

vector<Vector3D> BioPotentials::GetCoordinatesPotential(const Cfg &c, Environment * env) {
   Cfg::CfgHelper->ConfigEnvironment(c, env);
   return GetCoordinatesFullMainChain(env);
}

vector<Vector3D> BioPotentials::GetCoordinatesForRMSD(const Cfg &c, Environment * env) {
   Cfg::CfgHelper->ConfigEnvironment(c, env);
   return GetCoordinatesFullMainChain(env);
}

vector<Vector3D> BioPotentials::GetCoordinatesLigandBinding(const Cfg &c, Environment * env) {
    Cfg::CfgHelper->ConfigEnvironment(c, env);
    vector<Vector3D> coordinates;

    MultiBody *robot = env->GetMultiBody(env->GetRobotIndex());

    for(int i=0 ; i<robot->GetFreeBodyCount(); i++){
        Transformation tmp = robot->GetFreeBody(i)->WorldTransformation();
        for(int j=0; j<atomCoord[i].size(); ++j) {
	   coordinates.push_back(tmp*atomCoord[i][j]);
	}
    }
    return coordinates;
}

double BioPotentials::PotentialLigandBinding(const Cfg &c, Environment * env) {
    vector<Vector3D> coordinates = GetCoordinatesLigandBinding(c,env);
    return potentialGrid->GetPotential(coordinates, atomType);
}

double BioPotentials::PotentialBindingExact(const Cfg &c, Environment * env) {
    vector<Vector3D> coordinates = GetCoordinatesLigandBinding(c,env);
    return potentialGrid->GetRealPotential(coordinates, atomType);
}



double BioPotentials::PotentialFullMainChain(const Cfg &c, Environment * env) {
    enum AtomType {Nitrogen, Hydrogen, Carbon, Oxygen, ExtendedC};
    static AtomType atoms[6] = {Nitrogen, Hydrogen, ExtendedC, ExtendedC, Carbon, Oxygen};
    static double A[5] = {3952850, 290, 1200965, 145834, 3075695};
    static double B[5] = {2556, 1.08, 425, 328, 953};

    vector<Vector3D> coordinates = GetCoordinatesPotential(c,env);
    double numofAtoms = coordinates.size();
    double valueA, valueB;
    double potential = 0.0;
    int i;
    for(i=0; i<numofAtoms; ++i) {
        int shift = i%2 ? 4 : 7;
        for(int j=i+shift; j<numofAtoms; ++j) {
           AtomType atomA = atoms[i%6];
           AtomType atomB = atoms[j%6];
           if((atomA == Hydrogen && atomB == Oxygen) ||
              (atomB == Hydrogen && atomA == Oxygen)) {
              valueA = 2913;
              valueB = 241;
           } else if(atomA == atomB) {
              valueA = A[atomA];
              valueB = B[atomA];
           } else {
              valueA = sqrt(A[atomA]*A[atomB]);
              valueB = sqrt(B[atomA]*B[atomB]);
           }
           double r = (coordinates[i]-coordinates[j]).magnitude();
           if(r < 0.5)
                return MAXPOTENTIAL;
           potential += valueA/pow(r,12)-valueB/pow(r,6);
        }
    }
    int numofJoints = Cfg::CfgHelper->GetDOF();
    for(i=0; i<numofJoints; ++i) {
        double equalibriumAngle = i%2 ? -40 : -65;
        double deltaPhi = c.GetData()[i]*TWOPI - equalibriumAngle*TWOPI/360;
        potential += 2.0-2.0*cos(6.0*deltaPhi);
    }

    return potential;

}



double BioPotentials::PotentialSideChainSameSphereHbonds(const Cfg &c, Environment * env) {
    vector<Vector3D> coordinates = GetCoordinatesPotential(c, env);
    //int numofAtoms = coordinates.size();
    int numofResidue = coordinates.size()/6;
    bool penetrate = false;
    for(int i=0; i<numofResidue; ++i) {
        for(int j=i+1; j<numofResidue; ++j) {
            double r = (coordinates[i*6+3]-coordinates[j*6+3]).magnitude();
            if(r<vdwSphereRadiusMin) // old value: 1.2 1.4
                return cnEmax*2; // not allowed for connection: in collision
            else if(r<vdwSphereRadiusMax)
                penetrate = true;
        }
    }
    if(penetrate)
	return (cnEmax + gnEmax)/2; // this allows such nodes to be ok in connection
			            // phase but not in node generation.

    else
        return PotentialFromHydrogenBondsAndssBonds(coordinates);
}

double BioPotentials::PotentialFromHydrogenBondsAndssBonds(vector<Vector3D> coordinates) {
    int i; 
    double potential = 0.0;
 
    for(i=0; i<HbondsNum; ++i) {
        int donor = HbondPair[i].first-1;
        int acceptor = HbondPair[i].second - 1;
        // cal distance between donor's Hydrogen atom and acceptor's Oxygen.
        double r = (coordinates[donor*6+1] - coordinates[acceptor*6+5]).magnitude();
        potential += 100.0*(sqrt((r-2)*(r-2) + 4) - 2);
    }

    for(i=0; i<SSbondsNum; ++i) {
        int donor = SSbondPair[i].first-1;
        int acceptor = SSbondPair[i].second - 1;
        // cal distance between donor's Hydrogen atom and acceptor's Oxygen.
        double r = (coordinates[donor*6+3] - coordinates[acceptor*6+3]).magnitude();
        potential += 100.0*(sqrt((r-3)*(r-3) + 4) - 2);
    }
    return potential;
}



vector<Vector3D> BioPotentials::GetCoordinatesFullMainChain(Environment * env) {
    static Vector3D N(0.0, 0.0, 0), Hn(0.0, 0.971, 0.47308),
                    CA(0.0, 0.0, 0.0), Rca(-1.217, 0.818, 0.4654),
                    C(0.0, 0.0, 0.0), Oc(-0.011, 1.049, 0.65485);
    vector<Vector3D> coordinates;

    MultiBody *robot = env->GetMultiBody(env->GetRobotIndex());

    Transformation &t = robot->GetFixedBody(0)->WorldTransformation();
    coordinates.push_back(t*N); // nitrogen atom
    coordinates.push_back(t*Hn);


    for(int i=0 ; i<robot->GetFreeBodyCount(); i++){
        Transformation tmp = robot->GetFreeBody(i)->WorldTransformation();
        switch (i%3) {
           case 0: // CA: carbon alpha
                coordinates.push_back(tmp*CA); // CA: Carbon alpha atom.
                //coordinates.push_back(tmp*HAca);     // HA atom in 'ca' frame.
                coordinates.push_back(tmp*Rca);      // R atom group in 'ca' frame.
                break;
           case 1: // C:
                coordinates.push_back(tmp*C); // C atom.
                coordinates.push_back(tmp*Oc);       // Oxygen atom at 'C' frame.
                break;
           case 2: // Nitrogen
                coordinates.push_back(tmp*N); // nitrogen atom
                coordinates.push_back(tmp*Hn);       // H atom at N frame: Hn.
                break;
	   default:
		break;
        }
    }
    return coordinates;

}


//double PotentialVirtualBonds(...);
//double PotentialFromTinker();

