// $Id$
/////////////////////////////////////////////////////
//
//	util.c
//
//	Created	7/17/98 	Daniel Vallejo
//
////////////////////////////////////////////////////

#include <math.h>

#include "util.h"
#include "Roadmap.h"


// Calculate the minimum DIRECTED angular distance between two angles
// normalized to 1.0
double DirectedAngularDistance(double a,double b)
{

    // normalize both a and b to [0, 1)
    a = a - floor(a);
    b = b - floor(b);

    // shorten the distance between a and b by shifting a or b by 1.
    // have to do ROUND-UP INTEGER comparision to avoid subtle numerical errors.
    int intA = rint(a*1000000);
    int intB = rint(b*1000000);

    if( intB - intA  > 500000 ) 
	++a;
    else if ( intA - intB > 500000 )
	++b;

    // this would cause error. assume a = 0, b = 0.50000001 at mkmp stage
    // and b = 0.5 (rounded-up after printed to the map) at query stage.
    //if( b-a > 0.500000 )
    //    ++a;
    //else if( a-b > 0.500000 )
    //    ++b;

    return b-a;

}


// read cfgs from a file into a vector.
void ReadCfgs(char * filename,  vector<Cfg> &cfgs) {
  ifstream  is(filename);
  if (!is) {
         cout << endl << "In util::ReadCfgs: can't open infile: " << filename << endl;
         return;
  }

  Cfg tempCfg;
  while (1) {
      tempCfg.Read(is);
      if(!is) break;
      cfgs.push_back(tempCfg);
  }
}



/////////////////////////////////////////////////////////////////////
// Output to a file the sequence of cfgs.
/////////////////////////////////////////////////////////////////////
void
WritePathTranformationMatrices( char output_file[80],
                                const vector<Cfg>& path,
                                Environment *env ) { 
    FILE *fp;

    if((fp = fopen(output_file,"w")) == NULL){
        printf("\n\t Can't open file %s \n",output_file);
        exit(1);
    }

    Cfg::print_preamble_to_file(env, fp, path.size());

    for(int i = 0 ; i < path.size() ; i++){
        // Translate all path configurations such that their resulting
        // center of gravity is what was saved (ie, the rover original)
	//path[i].print_vizmo_format_to_file(env,fp);
	vector<Vector6D> tmp;
	path[i].printLinkConfigurations(env, tmp);
	for(int j=0; j<tmp.size(); ++j) {
	    fprintf(fp,"\n%f %f %f %f %f %f", tmp[j][0], tmp[j][1], tmp[j][2],
	            tmp[j][3], tmp[j][4], tmp[j][5]);
	}	    
	// Cfg class need Environment Info to interprete 'abstract' Cfg.

    }
    fprintf(fp,"\n");
    fclose(fp);
}

ostream& operator<< (ostream& _os, const IntWeight& w) {
  _os<< w.lp << " " << w.nticks ;
};
istream& operator>> (istream& _is, IntWeight& w) {
  _is>> w.lp >> w.nticks ;
};
istream& operator>> (istream& _is, DblWeight& w) {
  _is>> w.lp >> w.weight ;
};
ostream& operator<< (ostream& _os, const DblWeight& w) {
  _os<< w.lp << " " << w.weight ;
}; 

