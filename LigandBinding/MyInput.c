// $Id$
////////////////////////////////////////////////////
//
//  MyInput.c
//
//  derived class of Input
//
/////////////////////////////////////////////////////

#include"MyInput.h"
#include"Cfg.h"
//#include"Cfg_tree.h"
//#include"CfgProtein.h"
#include"CfgLigand.h"
#include"BioPotentials.h"
#include"util.h"

void MyInput::ReadCommandLine(int argc, char** argv) {
  Input::ReadCommandLine(argc, argv);
  //if (!(strncmp(cfgName,"Cfg_fixed_tree",14))) {
  //     Cfg::CfgHelper = new Cfg_tree(numofJoints);
  //} else if (!(strncmp(cfgName,"CfgProtein",10))) {
  //     Cfg::CfgHelper = new CfgProtein(numofJoints);
  //} else 
  if (!(strncmp(cfgName,"CfgLigand",9))) {
       Cfg::CfgHelper = new CfgLigand(numofJoints);
  }
  ReadPotentialParameters();
}


void MyInput::ReadPotentialParameters() {
  char tmp[80];
  strcpy(tmp, defaultFile.GetValue() );
  strcat(tmp, ".parameters");
  VerifyFileExists(tmp, EXIT);

  ifstream is(tmp);
  ReadPotentialParameters(is);
}

void MyInput::ReadPotentialParameters(istream &is) {
  char ptype[80];
  readfield(is, &ptype);
  //if(ptype == ...) then ReadPotentialParametersMainChain...();
  //else if(ptype == ) ...
  // Since different type of potentials probably have different input 
  // file input format. So the above 'if .. else ..' may happen later.
  if(!strncmp(ptype, "FullMainChain", 13)) {
     BioPotentials::potentialType = FullMainChain;
  } else if(!strncmp(ptype, "SideChainSameSphereHbonds", 13)) {
     BioPotentials::potentialType = SideChainSameSphereHbonds;
  } else if(!strncmp(ptype, "LigandBinding", 13)) {
     BioPotentials::potentialType = LigandBinding;
  } else if(!strncmp(ptype, "BindingExactPotential", 13)) {
     BioPotentials::potentialType = BindingExactPotential;
  } else {
     cout << "\nError: in BioPotentials::GetPotential. Unknown Potential type:"
          << ptype << ". " << endl;
     cout << "Possible Potential Type: FullMainChain -- " << FullMainChain << "\n"
          << "                         SideChainSameSphereHbonds -- " <<
            SideChainSameSphereHbonds << endl;
     exit(10);
  }

  readfield(is, &BioPotentials::gnEmin);
  readfield(is, &BioPotentials::gnEmax);
  readfield(is, &BioPotentials::cnEmax);

  // this is something extra for ligand binding.
  if(BioPotentials::potentialType == LigandBinding || 
     BioPotentials::potentialType == BindingExactPotential ) {
     char tableName[80];
     strcpy(tableName, defaultFile.GetValue() );
     strcat(tableName, ".gridTable");
     VerifyFileExists(tableName, EXIT);

     if(BioPotentials::potentialType == LigandBinding) {
         BioPotentials::potentialGrid = new PotentialForceGrids(tableName);
     } else {
         BioPotentials::potentialGrid = new PotentialForceGrids(defaultFile.GetValue(),
     					POTENTIAL, 2, 2);
     }
     char ligandFile[80];
     strcpy(ligandFile, defaultFile.GetValue() );
     strcat(ligandFile, ".ligand");
     VerifyFileExists(ligandFile, EXIT);

     ifstream fs(ligandFile);
     int frames, atoms;
     vector<Vector3D> v;
     char ch;
     Vector3D tmp;
     readfield(fs, &frames);
     for(int n=0; n<frames; ++n) {
	readfield(fs, &atoms);
	for(int k=0; k<atoms; ++k) {
	   readfield(fs,&ch);
	   readfield(fs, &tmp);
           BioPotentials::atomType.push_back(ch);
	   v.push_back(tmp);
	}
	BioPotentials::atomCoord.push_back(v);
	v.erase(v.begin(), v.end());
     }
     cout << "finish setting up stuff .... " << endl << endl;
     return;
  }
	
	 
     


  readfield(is, &BioPotentials::vdwSphereRadiusMax);
  readfield(is, &BioPotentials::vdwSphereRadiusMin);

  readfield(is, &BioPotentials::HbondsNum);
  readfield(is, &BioPotentials::SSbondsNum);

  int i;
  for(i=0; i<BioPotentials::HbondsNum; ++i) {
     int donor, acceptor;
     readfield(is, &donor);
     readfield(is, &acceptor);
     BioPotentials::HbondPair.push_back(pair<int,int>(donor, acceptor));
  }

  for(i=0; i<BioPotentials::SSbondsNum; ++i) {
     int sA, sB;
     readfield(is, &sA);
     readfield(is, &sB);
     BioPotentials::SSbondPair.push_back(pair<int,int>(sA, sB));
  }

  int numofResidue = numofJoints/2;
  for(i=0; i< numofResidue; ++i) {
    char structureType[2];
    readfield(is, &structureType);
    switch(structureType[0]) {
      case 'H':  // alpha Helix
	 BioPotentials::torsionAngle.push_back(-60); // phi
	 BioPotentials::torsionAngle.push_back(-40); // psi
 	 break;
      case 'E': // Extended beta
	 BioPotentials::torsionAngle.push_back(-120); // phi
	 BioPotentials::torsionAngle.push_back(150); // psi
         break;
      default: // T, for turns, and others
	 BioPotentials::torsionAngle.push_back(-90);
	 BioPotentials::torsionAngle.push_back(90);
	 break;
    }
  }

  // now, read in the value of 'std' & 'range' for CfgProtein class.
  //readfield(is, &CfgProtein::std);
  //readfield(is, &CfgProtein::range);
}	 

     
  
