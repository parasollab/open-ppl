/////////////////////////////////////////////////////
//
//	util.c
//
//	Created	7/17/98 	Daniel Vallejo
//
////////////////////////////////////////////////////

#include "util.h"
#include "Roadmap.h"
#include "MultiBody.h"
#include "Environment.h"
#include "Cfg.h"

// Calculate the minimum DIRECTED angular distance between two angles
// normalized to 1.0
double DirectedAngularDistance(double a,double b) {
	
  // normalize both a and b to [0, 1)
  a = a - floor(a);
  b = b - floor(b);
  
  // shorten the distance between a and b by shifting a or b by 1.
  // have to do ROUND-UP INTEGER comparision to avoid subtle numerical errors.
  int intA = (int)rint(a*1000000);
  int intB = (int)rint(b*1000000);
  
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


double
GaussianDistribution(double m, double s) 
{
  double x1, x2, w, r;
  do {
    r = OBPRM_drand();
    x1 = 2. * r - 1.;
    r = OBPRM_drand();
    x2 = 2. * r - 1.;
    w = x1 * x1 + x2 * x2;
  } while(w >= 1. || w < 1E-30);
  w = sqrt((-2. * log(w)) / w);
  x1 *= w;
  return x1 * s + m;
}


/////////////////////////////////////////////////////////////
//	RandomNumGenerator
/////////////////////////////////////////////////////////////


double
OBPRM_drand() {
  return drand48();
  
}

long
OBPRM_lrand() {
  return lrand48();
}

long
OBPRM_mrand() {
  return mrand48();
}

// normally(gaussian) distributed random number generator.
double OBPRM_grand(bool reset) {
  double v1, v2, rsq;
  static int iset = 0;
  if( reset )  { //reset iset to 0 and return
    iset = 0;
    //cout<<"OBPRM_grand() is called. reset internal static variables"<<endl;
    return 0.0;
  }
  static double gset;
  if(iset == 0) {
     do {
        v1 = 2*drand48() - 1.0;
	v2 = 2*drand48() - 1.0;
	rsq = v1*v1 + v2*v2;
     } while (rsq >= 1.0 || rsq == 0.0);
     double fac = sqrt(-2.0*log(rsq)/rsq);
     gset = v1*fac;
     iset = 1;
     return v2*fac;
  } else {
     iset = 0;
     return gset;
  }
}

  
long
OBPRM_srand(long seedval){
  static long oldSeed = seedval;
  if(oldSeed != seedval) {
    oldSeed = seedval;
    return OBPRM_srand("NONE", 0, seedval, true);
  } else
  return OBPRM_srand("NONE", 0, seedval);
}

//the real seed is decided by: baseSeed, methodName, nextNodeIndex
long
OBPRM_srand(std::string methodName, int nextNodeIndex, long base, bool reset) {
  static long baseSeed = base;
  if (reset)
    baseSeed = base;
  //cout<<"baseSeed is: "<<baseSeed<<endl;
  
  if (methodName != "NONE") {
    long methodID = 0;
    for(size_t i=0; i<methodName.length(); i++){
      int tmp = methodName[i];
      methodID += tmp*(i+1)*(i+2);
    }
    cout<<"the seed is:"<<long (baseSeed * (nextNodeIndex+1) + methodID)<<endl;
    srand48(long (baseSeed * (nextNodeIndex+1) + methodID));
  } 
  else {
    srand48(baseSeed);
  }
  
  return baseSeed;
}


/////////////////////////////////////////////////////////////////////
// Output to a file the sequence of cfgs.
/////////////////////////////////////////////////////////////////////
void
WritePathLinkConfigurations( const char output_file[80],
			     vector<Cfg*>& path,
			     Environment *env ) { 
  FILE *fp;
  
  if((fp = fopen(output_file,"w")) == NULL){
    printf("\n\t Can't open file %s \n",output_file);
    exit(1);
  }
  
  //Cfg::print_preamble_to_file(env, fp, path.size());
  fprintf(fp,"VIZMO_PATH_FILE   Path Version %d\n", PATHVER_20001022);
  int numofLink = env->GetMultiBody(env->GetRobotIndex())->GetFreeBodyCount();
  fprintf(fp,"%d\n", numofLink);
  fprintf(fp,"%zu",path.size());
  
  char cfgFile[100];
  sprintf(cfgFile, "%s%s", output_file, ".cfg");
  ofstream oc(cfgFile);
  for(size_t i = 0 ; i < path.size() ; i++){
    // Translate all path configurations such that their resulting
    // center of gravity is what was saved (ie, the rover original)
    //path[i].print_vizmo_format_to_file(env,fp);
    vector<Vector6D> tmp;
    path[i]->printLinkConfigurations(env, tmp);
    for(size_t j=0; j<tmp.size(); ++j) {
      fprintf(fp,"\n%f %f %f %f %f %f", tmp[j][0], tmp[j][1], tmp[j][2],
	      tmp[j][3], tmp[j][4], tmp[j][5]);
    }	    
    // Cfg class need Environment Info to interprete 'abstract' Cfg.
    path[i]->Write(oc);
    oc << "\n";
    
    
  }
  fprintf(fp,"\n");
  fclose(fp);
}



void
WritePathConfigurations( const char output_file[80],
			 vector<Cfg*>& path,
			 Environment *env ) {
  FILE *fp;
  
  if((fp = fopen(output_file,"w")) == NULL){
    printf("\n\t Can't open file %s \n",output_file);
    exit(1);
  }
  
  //Cfg::print_preamble_to_file(env, fp, path.size());        
  fprintf(fp,"VIZMO_PATH_FILE   Path Version %d\n", PATHVER_20001125);
  fprintf(fp,"%d\n", 1);
  fprintf(fp,"%zu \n", path.size());
  
  for(size_t i = 0 ; i < path.size() ; i++){
    vector<double> tmp = path[i]->GetData();
    // Translate all path configurations such that their resulting
    // center of gravity is what was saved (ie, the rover original)
    for(size_t j=0; j<tmp.size(); ++j) {
      fprintf(fp,"%f ",tmp[j]);
    }
    if((int)i!=((int)path.size()-1))fprintf(fp,"\n");
    
  }
  fprintf(fp,"\n");
  fclose(fp);
}



bool
VerifyFileExists(const char *_fname,int action) {
  ifstream is(_fname);
  char ch;
  if (!is.get(ch)) {
    cout << "\nERROR: Can't open \"" << _fname << "\"" << endl;
    if(action==PMPL_EXIT)
      exit(1);
    else return false;
  }                                                                             
  is.close();
  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//
//Modified for VC
//
//
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Implementation of drand48
// These are not standard c functions. therefore, they are not in vc.....

#ifdef _WIN32

/*	@(#)drand48.c	2.2	*/
/*LINTLIBRARY*/
/*
*	drand48, etc. pseudo-random number generator
*	This implementation assumes unsigned short integers of at least
*	16 bits, long integers of at least 32 bits, and ignores
*	overflows on adding or multiplying two unsigned integers.
*	Two's-complement representation is assumed in a few places.
*	Some extra masking is done if unsigneds are exactly 16 bits
*	or longs are exactly 32 bits, but so what?
*	An assembly-language implementation would run significantly faster.
*/
extern "C" {
	
#ifndef HAVEFP
#define HAVEFP 1
#endif
#define N	16
#define MASK	((unsigned)(1 << (N - 1)) + (1 << (N - 1)) - 1)
#define LOW(x)	((unsigned)(x) & MASK)
#define HIGH(x)	LOW((x) >> N)
#define MUL(x, y, z)	{ long l = (long)(x) * (long)(y); \
	(z)[0] = LOW(l); (z)[1] = HIGH(l); }
#define CARRY(x, y)	((long)(x) + (long)(y) > MASK)
#define ADDEQU(x, y, z)	(z = CARRY(x, (y)), x = LOW(x + (y)))
#define X0	0x330E
#define X1	0xABCD
#define X2	0x1234
#define A0	0xE66D
#define A1	0xDEEC
#define A2	0x5
#define C	0xB
#define SET3(x, x0, x1, x2)	((x)[0] = (x0), (x)[1] = (x1), (x)[2] = (x2))
#define SETLOW(x, y, n) SET3(x, LOW((y)[n]), LOW((y)[(n)+1]), LOW((y)[(n)+2]))
#define SEED(x0, x1, x2) (SET3(x, x0, x1, x2), SET3(a, A0, A1, A2), c = C)
#define REST(v)	for (i = 0; i < 3; i++) { xsubi[i] = x[i]; x[i] = temp[i]; } \
	return (v);
#define NEST(TYPE, f, F)	TYPE f(register unsigned short *xsubi) { \
	register int i; register TYPE v; unsigned temp[3]; \
	for (i = 0; i < 3; i++) { temp[i] = x[i]; x[i] = LOW(xsubi[i]); }  \
	v = F(); REST(v); }
#define HI_BIT	(1L << (2 * N - 1))
	
	static unsigned x[3] = { X0, X1, X2 }, a[3] = { A0, A1, A2 }, c = C;
	static unsigned short lastx[3];
	//static void next();
	
#if HAVEFP
  double drand48() {
#if pdp11
    static double two16m; /* old pdp11 cc can't compile an expression */
    two16m = 1.0 / (1L << N); /* in "double" initializer! */
#else
    static double two16m = 1.0 / (1L << N);
#endif
    
    next();
    return (two16m * (two16m * (two16m * x[0] + x[1]) + x[2]));
  }
  
  NEST(double, erand48, drand48);
  
#else
  long irand48(register unsigned short m) {
    /* Treat x[i] as a 48-bit fraction, and multiply it by the 16-bit
     * multiplier m.  Return integer part as result.
     */
    unsigned r[4], p[2], carry0 = 0;
    
    next();
    MUL(m, x[0], &r[0]);
    MUL(m, x[2], &r[2]);
    MUL(m, x[1], p);
    if (CARRY(r[1], p[0]))
      ADDEQU(r[2], 1, carry0);
    return (r[3] + carry0 + CARRY(r[2], p[1]));
  }
  
  long krand48(register unsigned short *xsubi, unsigned short m) {
    /* same as irand48, except user provides storage in xsubi[] */
    register int i;
    register long iv;
    unsigned temp[3];
    
    for (i = 0; i < 3; i++) {
      temp[i] = x[i];
      x[i] = xsubi[i];
    }
    iv = irand48(m);
    REST(iv);
  }
#endif
  
  long lrand48() {
    next();
    return (((long)x[2] << (N - 1)) + (x[1] >> 1));
  }
  
  long mrand48() {
    register long l;
    
    next();
    /* sign-extend in case length of a long > 32 bits
       (as on Honeywell) */
    return ((l = ((long)x[2] << N) + x[1]) & HI_BIT ? l | -HI_BIT : l);
  }
  
  static void next() {
    unsigned p[2], q[2], r[2], carry0, carry1;
    
    MUL(a[0], x[0], p);
    ADDEQU(p[0], c, carry0);
    ADDEQU(p[1], carry0, carry1);
    MUL(a[0], x[1], q);
    ADDEQU(p[1], q[0], carry0);
    MUL(a[1], x[0], r);
    x[2] = LOW(carry0 + carry1 + CARRY(p[1], r[0]) + q[1] + r[1] +
	       a[0] * x[2] + a[1] * x[1] + a[2] * x[0]);
    x[1] = LOW(p[1] + r[0]);
    x[0] = LOW(p[0]);
  }
  
  void srand48(long seedval) {
    SEED(X0, LOW(seedval), HIGH(seedval));
  }
	
  unsigned short* seed48(unsigned short seed16v[3]) {
    SETLOW(lastx, x, 0);
    SEED(LOW(seed16v[0]), LOW(seed16v[1]), LOW(seed16v[2]));
    return (lastx);
  }
  
  void lcong48(unsigned short param[7]) {
    SETLOW(x, param, 0);
    SETLOW(a, param, 3);
    c = LOW(param[6]);
  }
  
  double rint(double x) {
    double d_FractionOfX=x-floor(x);	//fraction part of x
    if( d_FractionOfX < 0.5 ) return floor(x);
    return ceil(x);
  }
  
}//End of extern "C"
#endif //endif _WIn32
