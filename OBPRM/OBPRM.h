// $Id$
/////////////////////////////////////////////////////////////////////
//
//  OBPRM.h
//
//  General Description
//      This is a set of definitions useful to the OBPRM application
//	specifically.  Almost all the OBPRM specific files will need 
//	to include them.
//
//  Created
//      8/25/98  Lucia K. Dale
//  History
//      8/27/98  Lucia K. Dale
//      8/31/98  O.B. Bayazit     added the Cfg class
//      03/03/99 Guang Song       add Cfg_fixed_PRR
//	                          add Cfg_free_serial & its iostream.
//
/////////////////////////////////////////////////////////////////////

#ifndef OBPRM_h
#define OBPRM_h

#ifdef HPUX
#include <sys/io.h>
#endif
#include "BasicDefns.h"

//-----------------------------------
// Constants
//---------------------------------
#define ORIENTATION_RES             0.05
#define POSITION_RES_FACTOR         0.05

#define INVALID_SID -999
#define INVALID_EID -999

#define INVALID_DM -999                 // invalid dm id
#define INVALID_CD -999                 // invalid cd id
#define INVALID_LP -999                 // invalid lp id
#define INVALID_GN -999                 // invalid gn id
#define INVALID_CN -999                 // invalid cn id

// NMA: for edge weights... not good values or placement (in BasicDefns?)
#define MAX_INT  999999999
#define INVALID_INT -999
#define MAX_DBL  999999999
#define INVALID_DBL -999

//-----------------------------------
// general data structures
//-----------------------------------
typedef short EID;  // element id type
typedef short SID;  // set id type

//-----------------------------------
// c-space representations
//-----------------------------------
#include "Cfg.h"

//-----------------------------------
// choose represention for edge weights in roadmap graph, some samples
//-----------------------------------

// "normal" edges 
class IntWeight { 
public:
  IntWeight(){ lp = INVALID_LP; nticks=1; };
  IntWeight(int i){ lp=i; nticks=1; };
  IntWeight(int i, int j){ lp=i; nticks=j; };
  ~IntWeight(){};

  bool operator== (const IntWeight &tmp) const{return ((lp==tmp.lp)&&(nticks==tmp.nticks)) ;};
  friend ostream& operator<< (ostream& _os, const IntWeight& w);  // in util.c
  friend istream& operator>> (istream& _is, IntWeight& w);        // in util.c

  static IntWeight InvalidWeight() { return IntWeight(INVALID_LP); };
  static int MaxWeight() { return MAX_INT; }; // for Dijkstra's Alg
  int& Weight() { return nticks; };

  int& LP() { return lp; };
  int& NTicks() { return nticks; };

private:
  int lp;
  int nticks;
};

// "special" edges, e.g., for protein folding
class DblWeight {
public:
  DblWeight(){ lp=INVALID_LP; weight = INVALID_DBL; };
  DblWeight(int i){ lp=i; weight=INVALID_DBL; };
  DblWeight(int i, double j){ lp=i; weight=j; };
  ~DblWeight(){};

  bool operator== (const DblWeight &tmp) const{return ((lp==tmp.lp)&&(weight==tmp.weight)); };
  friend ostream& operator<< (ostream& _os, const DblWeight& w);  // in util.c
  friend istream& operator>> (istream& _is, DblWeight& w);        // in util.c  

  static DblWeight InvalidWeight() { return DblWeight(INVALID_LP); };
  static double MaxWeight() { return MAX_DBL; }; // for Dijkstra's Alg
  double& Weight() { return weight; };
  int& LP() { return lp; };

private:
  int    lp;
  double weight;
};


//-----------------------------------
// now, actually choose weight type 
//-----------------------------------
#ifndef WEIGHT
typedef IntWeight WEIGHT;
//typedef DblWeight WEIGHT;
#endif


#endif
