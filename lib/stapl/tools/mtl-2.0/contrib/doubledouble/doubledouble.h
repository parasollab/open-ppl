// doubledouble.h
// Keith Briggs.   Last revised 98 Feb 09

/*
Copyright (C) 1997 Keith Martin Briggs

Except where otherwise indicated,
this program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

// Change log:  (See doubledouble.cc also)
//  97 Dec 22 KMB added x86_FIX
//            Eliminates -ffloat-store requirement
//  97 Aug 04 KMB added ldexp
//  97Jul25 (WH) - fixed Qrand48 to return drand48 + (2^-47)*drand48
//  97Jul11 (WH) added qtoa declaration.
//	- made all constants (Pi etc) member constants, moved to quad.cc
//	- added doubledoubleRand48().
//	- cleaned up constructors (fewer, with default arguments).
//	- added some code for integer exponent, but commented it out.  It
//	  looks hard.  eg, adding numbers with different exponents.  Almost
//	  always the smaller would be essentially zero, but near boundaries...
//  96Nov20 added normalizing constructor doubledouble(double,double) (needed by floor)

#ifndef __QUAD_H__
#define __QUAD_H__

#define DEBUG_QUAD 0

#ifdef SGI_CC
#define bool int
#undefine _G_HAVE_BOOL
#endif

#ifdef __GNUC__
//  #if (__GNUC_MAJOR__<=2 && __GNUC_MINOR__<=6)
//    #define bool int
    #define explicit
    #ifndef __isinf
      #define __isinf(x) ((x)!=(x)) // gcc 2.7.2 defines __isinf as a function
    #endif
//  #endif
#endif

#ifdef __KCC
  #ifndef __isinf
    #define __isinf(x) ((x)!=(x)) // gcc 2.7.2 defines __isinf as a function
  #endif
#endif

//JGS AIX doesn't have this in /usr/include
//  #include <nan.h>  // defines NAN, at least in gcc
#include <assert.h>

// DOMAIN_ERROR=action to take on function domain errors
#ifdef NAN
#define DOMAIN_ERROR return(doubledouble(NAN,NAN))
#else
#define DOMAIN_ERROR assert(0)
#endif

#ifdef x86
#define x86_FIX \
  unsigned short __old_cw, __new_cw; \
  asm volatile ("fnstcw %0":"=m" (__old_cw)); \
  __new_cw = (__old_cw & ~0x300) | 0x200; \
  asm volatile ("fldcw %0": :"m" (__new_cw));
#define END_x86_FIX  asm volatile ("fldcw %0": :"m" (__old_cw));
#else
#define x86_FIX
#define END_x86_FIX
#endif


#include <iostream.h>
#include <iomanip.h>
#include <math.h>
#include <stdlib.h>
//#include <float.h>
#include <sys/types.h>
#include <string.h>
#include <stdarg.h>

namespace std {

/**
  @name Double-Double Numeric Class

  This is the {\tt doubledouble} class created by Keith Martin Briggs.
  The code has been modifed to compile with KAI C++ in addition to
  egcs. Also all of the functionality has been moved to header files
  so there is no need to link in a library.  Furthermore, the
  doubledouble class has been encorporated into the MTL test suite to
  ensure its interoperability with the MTL containers and algorithms.

  There is more documentation for the doubledouble class in the
  /contrib/doubledouble directory.
  
  The web site for the {\tt doubledouble} class is
  http://www-epidem.plantsci.cam.ac.uk/~kbriggs/doubledouble.html

  @memo Extended Precision

*/
//@{
///
class doubledouble {
protected:
  double hi, lo;
public:
  // Public data members, initialized in doubledouble.cc
  // moved these into member functions which create the
  // constants on the fly and return them to the user.
#if 0
  static const double Split; // cannot be initialized here; see ARM 9.4
  static const doubledouble Log2, Log10, Pi, TwoPi, Pion2, Pion4, _Pi;
#endif

  /// Public access to hi and lo
  inline double h() const { return hi; }
  ///
  inline double l() const { return lo; }

  /**@name Constructors */
  //@{
  ///
  inline doubledouble():hi(0.0),lo(0.0) {}
  ///
  inline doubledouble(const int n) { hi=double(n); lo=0.0; }
  ///
  inline doubledouble(const double x, const double y);
  ///
  inline doubledouble(const doubledouble&);
  ///
  doubledouble(const char*);
  //@}
  /**@name Operators */
  ///
  doubledouble& operator +=(const doubledouble&);
  ///
  doubledouble& operator +=(const double&);
  ///
  doubledouble& operator +=(const int);
  ///
  doubledouble& operator -=(const doubledouble&);
  ///
  doubledouble& operator -=(const double&);
  ///
  doubledouble& operator -=(const int);
  ///
  doubledouble& operator *=(const doubledouble&);
  ///
  doubledouble& operator *=(const double&);
  ///
  doubledouble& operator *=(const int);
  ///
  doubledouble& operator /=(const doubledouble&);
  ///
  doubledouble& operator /=(const double&);
  ///
  doubledouble& operator /=(const int);
  ///
  doubledouble& operator=(const doubledouble&);
  ///
  doubledouble& operator=(const double&);
  ///
  doubledouble& operator=(const int);  // Get funny errors without this
  ///
  doubledouble& operator=(const char*);

  // Type conversion operator.  Not really necessary...
  operator double() const { return hi+lo; }
#if _KCC
  operator int() const { return (int)(hi+lo); }
#endif
  ///
  inline doubledouble normalize(void) { 
    double h=hi+lo; lo=lo+(hi-h); hi=h;
    return *this;
  }
  void dump(char*) const;  // debugging use only

  /**@name Friends */
  //@{
  ///
  inline friend doubledouble operator -(const doubledouble& x);  // Unary -
  ///
  friend doubledouble operator +(const doubledouble&, const doubledouble& );
  ///
  friend doubledouble operator +(const double&, const doubledouble& );
  ///
  friend doubledouble operator +(const int, const doubledouble& );
  ///
  friend doubledouble operator +(const doubledouble&, const double& );
  ///
  friend doubledouble operator +(const doubledouble&, const int );
  ///
  friend doubledouble operator -(const doubledouble&, const doubledouble& );
  ///
  friend doubledouble operator -(const double&, const doubledouble& );
  ///
  friend doubledouble operator -(const int, const doubledouble& );
  ///
  friend doubledouble operator -(const doubledouble&, const double& );
  ///
  friend doubledouble operator -(const doubledouble&, const int );
  ///
  friend doubledouble operator *(const doubledouble&, const doubledouble& );
  ///
  friend doubledouble operator *(const double&, const doubledouble& );
  ///
  friend doubledouble operator *(const int, const doubledouble& );
  ///
  friend doubledouble operator *(const doubledouble&, const double& );
  ///
  friend doubledouble operator *(const doubledouble&, const int );
  ///
  friend doubledouble operator /(const doubledouble&, const doubledouble& );
  ///
  friend doubledouble operator /(const doubledouble&, const double& );
  ///
  friend doubledouble operator /(const doubledouble&, const int );
  ///
  friend doubledouble operator /(const double&, const doubledouble& );
  ///
  friend doubledouble operator /(const int, const doubledouble& );
  ///
  friend doubledouble recip(const doubledouble &);
  ///
  friend doubledouble operator |(const doubledouble&, const doubledouble& );
  ///
  friend double dnorm(const doubledouble&);
  ///
  friend int intq(const doubledouble&);
  ///
  friend doubledouble ldexp(const doubledouble x, const int exp);
  //@}

  ///
  bool operator!=(const doubledouble& y) { return hi!=y.h() || lo!=y.l(); };
  ///
  static double Split() { return 134217729.0L; } // 2^27+1, for IEEE double
  ///
  static doubledouble Log2() { return doubledouble("0.6931471805599453094172321214581765680755"); }
  ///
  static doubledouble Log10() { return doubledouble("2.302585092994045684017991454684364207601"); }
  ///
  static doubledouble Pi() { return doubledouble("3.1415926535897932384626433832795028841972"); }
  ///
  static doubledouble TwoPi() { return doubledouble("6.2831853071795864769252867665590057683943"); }
  ///
  static doubledouble Pion2() { return doubledouble("1.5707963267948966192313216916397514420985"); }
  ///
  static doubledouble Pion4() { return  doubledouble("0.7853981633974483096156608458198757210493"); }
  ///
  static doubledouble _Pi() { return  doubledouble("0.3183098861837906715377675267450287240689"); }

};  // end class doubledouble

void base_and_prec(void);
doubledouble atodd(const char *);  // string to doubledouble conversion
// doubledouble to string conversion.  W must be long enough.  Returns W.
char *qtoa(char *Word, int prec, int fmtch, doubledouble q);
bool operator> (const doubledouble&, const doubledouble&);
bool operator>=(const doubledouble&, const doubledouble&);
bool operator< (const doubledouble&, const doubledouble&);
bool operator<=(const doubledouble&, const doubledouble&);
bool operator==(const doubledouble&, const doubledouble&);
//bool operator!=(const doubledouble&, const doubledouble&);

// inline members

inline doubledouble::doubledouble(const double x, const double y = 0.0) {
  x86_FIX
  hi=x+y; lo=y+(x-hi); // normalize
  END_x86_FIX
}
inline doubledouble::doubledouble(const doubledouble& x):hi(x.hi),lo(x.lo) {}
inline doubledouble& doubledouble::operator=(const doubledouble& x){ hi=x.hi; lo=x.lo; return *this;}
inline doubledouble& doubledouble::operator=(const double& x){ hi=x; lo=0.0; return *this;}
inline doubledouble& doubledouble::operator=(const int x){ hi=x; lo=0.0; return *this;}

// inline functions
inline doubledouble operator-(const doubledouble& x) { return doubledouble(-x.hi, -x.lo); }
inline doubledouble normalize(const doubledouble& x) { return doubledouble(x.h(), x.l()); }
inline double dnorm(const doubledouble& x) { return fabs(x.h());}
inline int intq(const doubledouble& x) { // explicit type conversion doubledouble -> int
  return int(x.h()); 
} 
inline doubledouble doubledoubleRand48(void) {
  return doubledouble(drand48(), ldexp(drand48(), -47)); 
}

// inline functions (defined in doubledouble.cc)
doubledouble Qcopysign(const doubledouble&, const double);

// non inline functions (defined in doubledouble.cc and math.cc)

///
istream& operator >> (istream&, doubledouble&);
///
ostream& operator << (ostream&, const doubledouble&);
///
int sign(const doubledouble&);
///
doubledouble hypot(const doubledouble, const doubledouble);
///
doubledouble recip(const doubledouble&);
///
doubledouble sqrt(const doubledouble&);
///
doubledouble sqr(const doubledouble&);
///
doubledouble cub(const doubledouble&);
///
doubledouble sqr_double(const double&);
///
doubledouble rint(const doubledouble&);
///
doubledouble floor(const doubledouble&);
///
doubledouble trunc(const doubledouble&);
///
doubledouble fmod(const doubledouble&, const int);
///
doubledouble modf(const doubledouble&, doubledouble *ip);
///
doubledouble fabs(const doubledouble&);
///
doubledouble abs(const doubledouble&);
///
doubledouble exp(const doubledouble&); 
///
doubledouble log(const doubledouble&);  
///
doubledouble log10(const doubledouble&);  
///
doubledouble powint(const doubledouble&, const int);
///
doubledouble pow(const doubledouble&, const doubledouble&);
///
doubledouble sin(const doubledouble&);
///
void sincos(const doubledouble x, doubledouble& sinx, doubledouble& cosx);
///
doubledouble cos(const doubledouble&);
///
doubledouble atan(const doubledouble&);
///
doubledouble atan2(const doubledouble&, const doubledouble&);
///
doubledouble asin(const doubledouble&);
///
doubledouble sinh(const doubledouble&);
///
doubledouble cosh(const doubledouble&);
///
doubledouble tanh(const doubledouble&);
///
doubledouble erf(const doubledouble);
///
doubledouble erfc(const doubledouble);
///
doubledouble gamma(const doubledouble);
///
int  digits(const doubledouble&,const doubledouble&);
///
doubledouble modr(const doubledouble a, const doubledouble b, int& n, doubledouble& rem);

//@}

//JGS always include this 
//#ifdef DD_INLINE
#include "inline.h"
//#endif	// DD_INLINE

}

#include "math.cc"

#endif	// __QUAD_H__
