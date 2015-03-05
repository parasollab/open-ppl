#include "MPUtils.h"
#include "MetricUtils.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Random Number Generation
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

double DRand() {return drand48();}
long LRand() {return lrand48();}
long MRand() {return mrand48();}

// normally(gaussian) distributed random number generator implementation of Polar Box-Muller method.
double GRand(bool _reset) {
  double v1, v2, rsq;// Two independent random variables and r^2
  static bool hasNext = false;//flag that tells whether the next gaussian has been computed
  if(_reset)  { //reset hasNext to return the mean
    hasNext=false;
    return 0.0;
  }
  static double gset;
  //If the next value to return is not already computed, compute it and the one after that
  if(!hasNext) {
    do {
      v1 = 2*DRand() - 1.0;
      v2 = 2*DRand() - 1.0;
      rsq = v1*v1 + v2*v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    double fac = sqrt(-2.0*log(rsq)/rsq);
    //Generates two gaussians and returns one and stores the other for another call
    gset = v1*fac;
    hasNext = true;
    return v2*fac;
  } else {
    // Else return gset that was computed in the previous call
    hasNext = false;
    return gset;
  }
}

//Same as GRand but with a scales the output by _stdev and centers it around _mean
double GaussianDistribution(double _mean, double _stdev) {
  return GRand(false) * _stdev + _mean;
}

long SRand(long _seedVal){
  static long oldSeed = _seedVal;
  if(oldSeed != _seedVal) {
    oldSeed = _seedVal;
    return SRand("NONE", 0, _seedVal, true);
  }
  else
    return SRand("NONE", 0, _seedVal);
}

//the real seed is decided by: baseSeed, methodName, nextNodeIndex
long SRand(string _methodName, int _nextNodeIndex, long _base, bool _reset) {
  static long baseSeed = _base;
  if(_reset)
    baseSeed = _base;
  if(_methodName != "NONE") {
    long methodID = 0;
    for(size_t i=0; i<_methodName.length(); i++){
      int tmp = _methodName[i];
      methodID += tmp*(i+1)*(i+2);
    }
    srand48(long (baseSeed * (_nextNodeIndex+1) + methodID));
  }
  else {
    srand48(baseSeed);
  }
  return baseSeed;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Simple Distance Utilities
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//normalize a number to [-1,1)
double Normalize(double _a){
  _a = fmod(_a+1.0, 2.0);
  //if(_a < -1E-6)
  if(_a < 0.0)
    _a+=2.0;
  _a--;
  return _a;
}

// Calculate the minimum DIRECTED angular distance between two angles
double DirectedAngularDistance(double _a, double _b) {
  // normalize both a and b to [-1, 1)
  _a = Normalize(_a);
  _b = Normalize(_b);

  if( _b - _a  > 1.0 )
    _a+=2.0;
  else if ( _a - _b > 1.0 )
    _b+=2.0;

  return _b-_a;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
bool
PtInTriangle(const Point2d& _A, const Point2d& _B, const Point2d& _C,const Point2d & _P) {
  // FIRST CHECK THE SIGN OF THE Z-COMPONENT OF THE NORMAL BY CALCULATING
  // THE CROSS-PRODUCT (ABxBC). THIS WILL DETERMINE THE ORDERING OF THE
  // VERTICES. IF NEGATIVE, VERTICES ARE CLOCKWISE ORDER; OTHERWISE CCW.
  // THEN EVALUATE SIGN OF Z-COMPONENTS FOR ABxAP, BCxBP, and CAxCP TO
  // DETERMINE IF P IS IN "INSIDE" HALF-SPACE FOR EACH EDGE IN TURN ("INSIDE"
  // IS DETERMINED BY SIGN OF Z OF NORMAL (VERTEX ORDERING).
  // NOTE: FULL CROSS-PRODS ARE NOT REQUIRED; ONLY THE Z-COMPONENTS
  Vector2d dAB=_B-_A, dBC=_C-_B;  // "REPEATS"
  if ((dAB[0]*dBC[1]-dAB[1]*dBC[0]) < 0) // CW
  {
    if (dAB[0]*(_P[1]-_A[1]) >= dAB[1]*(_P[0]-_A[0])) return false;           // ABxAP
    if (dBC[0]*(_P[1]-_B[1]) >= dBC[1]*(_P[0]-_B[0])) return false;           // BCxBP
    if ((_A[0]-_C[0])*(_P[1]-_C[1]) >= (_A[1]-_C[1])*(_P[0]-_C[0])) return false; // CAxCP
  }
  else // CCW
  {
    if (dAB[0]*(_P[1]-_A[1]) < dAB[1]*(_P[0]-_A[0])) return false;           // ABxAP
    if (dBC[0]*(_P[1]-_B[1]) < dBC[1]*(_P[0]-_B[0])) return false;           // BCxBP
    if ((_A[0]-_C[0])*(_P[1]-_C[1]) < (_A[1]-_C[1])*(_P[0]-_C[0])) return false; // CAxCP
  }
  return true; // "INSIDE" EACH EDGE'S IN-HALF-SPACE (PT P IS INSIDE TRIANGLE)
}

//----------------------------------------------------------------------------
// CHECKS IF 2D POINT P IS IN TRIANGLE ABC. RETURNS 1 IF IN, 0 IF OUT
//   uses barycentric coordinates to compute this and return the uv-coords
//   for potential usage later
//----------------------------------------------------------------------------
bool
PtInTriangle(const Point2d& _A, const Point2d& _B, const Point2d& _C,const Point2d& _P,
 double& _u, double& _v) {

  double epsilon = 0.0000001;

  // Compute vectors
  Vector2d v0 = _C - _A;
  Vector2d v1 = _B - _A;
  Vector2d v2 = _P - _A;

  // Compute dot products
  double dot00 = v0*v0;
  double dot01 = v0*v1;
  double dot02 = v0*v2;
  double dot11 = v1*v1;
  double dot12 = v1*v2;

  // Compute barycentric coordinates
  double invDenom = 1. / (dot00 * dot11 - dot01 * dot01);
  _u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  _v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (_u >= -epsilon) && (_v >= -epsilon) && (_u + _v < 1. + epsilon);
}

Point3d
GetPtFromBarycentricCoords(const Point3d& _A, const Point3d& _B, const Point3d& _C, double _u, double _v) {
  // P = A + u * (C - A) + v * (B - A)
  Point3d p = _A + (_u*(_C - _A)) + (_v*(_B - _A));
  return p;
}

double NormalizeTheta(double _theta){
  double val = _theta + PI;
  if (val == 0)
    return _theta;
  else
    return (val-TWOPI * floor(val/TWOPI) - PI);
}

