// $Id$
/////////////////////////////////////////////////////
//
//	util.c
//
//	Created	7/17/98 	Daniel Vallejo
//
//  Last Modified By:
//      08/24/98  Daniel Vallejo
//
////////////////////////////////////////////////////

#include <math.h>

#include "util.h"
#include "Roadmap.h"

/////////////////////////////////////////////////////////////////////
//
// Output to a file the sequence of cfgs in matrix form
// after to be transformed using the original center of gravity
// of the robot.
//
//////////////////////////////////////////////////////////////////////
void
WritePathTranformationMatrices( char output_file[80],
                                vector<Cfg> path,
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
	path[i].print_vizmo_format_to_file(env,fp);
	// Cfg class need Environment Info to interprete 'abstract' Cfg.

    }
    fprintf(fp,"\n");
    fclose(fp);
}

// brc moved the followings from  OBPRM.h
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

// brc following is moved from Graph.h
ostream& operator<< (ostream& _os, const dkinfo& dk) {
  _os<< "dkinfo = [" << dk.predvid << "," << dk.vid << "," << dk.dist << "]";
};

bool dkinfo_Compare ( const dkinfo& _d1, const dkinfo& _d2) {
  return ( _d1.dist > _d2.dist );
};
