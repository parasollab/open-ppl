// $Id$
/**@file VectorConstantSize.h
   This set of template classes provides an implementation for
   the a vector data type.

   The user must provide the parameter type ELEMENT, which is
   the data that will be stored in each coordinate of the vector.
   In addition, for the base class a SIZE must be specified.
   This is the dimension of the vector and is always an integer.

   The classes in this hierarchy (so far) include:
   "VectorConstantSize" a vector class template
   "Vector3"  a derived 3-dimensional vector class template
   "Vector6"  a derived 6-dimensional vector class template

   @date 7/26/98
   @author Lucia K. Dale
*/

#ifndef VectorConstantSize_h
#define VectorConstantSize_h

#include <vector.h>
#include <math.h>
#include "BasicDefns.h"


#define Xx 	0
#define Yy 	1
#define Zz	2
#define ROLLroll 	3
#define PITCHpitch	4
#define YAWyaw  	5

/**
  class VectorConstantSize<SIZE,ELEMENT>

  General Description
      Base class for all vectors of constant size.
*/

template <int SIZE,class ELEMENT>
class VectorConstantSize{
public:

  //===================================================================
  //  Constructors
  //===================================================================
    VectorConstantSize();
    VectorConstantSize(const VectorConstantSize&);
    VectorConstantSize(const ELEMENT&);
    VectorConstantSize(const vector<ELEMENT>&);
    VectorConstantSize(istream &);

  //===================================================================
  // Operators
  //===================================================================

	// Assignment
    VectorConstantSize& operator=  (const VectorConstantSize &); 

	// Addition, Subtration, Negation, Multiply & Divide by a Constant
    VectorConstantSize  operator+  (const VectorConstantSize &)const; 
    VectorConstantSize  operator-  (const VectorConstantSize &)const;
    VectorConstantSize  operator-  ()const;
    VectorConstantSize  operator*  (double)const;
    VectorConstantSize  operator/  (double)const;
    ELEMENT & operator[] (int s) { return v[s]; }
    const ELEMENT & operator[] (int s) const { return v[s]; }

	// Tests for Equality and Inequality
    bool            operator== (const VectorConstantSize &)const; 
    bool            operator!= (const VectorConstantSize &)const; 

  //===================================================================
  // Other Methods
  //===================================================================

	// Query for dimension(ie, size) of vector
    inline  int             size       ()const;

	// Magnitude of a vector
    double          magnitude  ()const;

	// Change values of vector so as to Normalize it
    inline  void            normalize  ();

	// Dot Product Operation
    inline  double          dotProduct (VectorConstantSize&) const;

	// I/O for vector
    void            Read       (istream&) ;
    inline  void    Write      (ostream&) const;

protected:
  //===================================================================
  // Data
  //===================================================================
    vector<ELEMENT> v;
};


//=====================================================================
// Guts of VectorConstantSize constructors & methods
//=====================================================================
template<int SIZE,class ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::VectorConstantSize(){
        v.reserve(SIZE);
        for(int i=0;i<SIZE;++i)
	   v.push_back(0.0);
};
template<int SIZE,class ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::VectorConstantSize(const VectorConstantSize &c){
        v.reserve(SIZE);
	v = c.v;
};
template<int SIZE,class ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::VectorConstantSize(const ELEMENT &assignVal){
        v.reserve(SIZE);
        for (int i=0;i<SIZE;++i){
		v.push_back(assignVal);
        };
};
template<int SIZE,class ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::VectorConstantSize(const vector<ELEMENT> &assignVal){
        if (SIZE == assignVal.size()){
                v.reserve(SIZE);
                v = assignVal;
        } else {
                cout << "\nERROR: you gave me " << assignVal.size()
                        << " assignment values for a vector "
                        << "you wanted to be size = "
                        << SIZE << endl << "       Bye-bye!!!" << endl;
                exit(0);
        }
};
template<int SIZE,class ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::VectorConstantSize(istream &_is){
	v.reserve(SIZE);
        ELEMENT tmp;
        for (int i=0;i<SIZE;++i){
                _is>>tmp;
                v.push_back(tmp);
        }
};

template<int SIZE,class ELEMENT>
int VectorConstantSize<SIZE,ELEMENT>::size()const{
	return SIZE;
};

template<int SIZE,class ELEMENT>
double VectorConstantSize<SIZE,ELEMENT>::magnitude()const{
        double result=0.0;
	for (int i=0;i<SIZE;++i)
                result += (v[i])*(v[i]);
        return (sqrt(result));
};
template<int SIZE,class ELEMENT>
void VectorConstantSize<SIZE,ELEMENT>::normalize(){
	double m = magnitude();
	for (int i=0;i<SIZE;++i)
        	v[i] = v[i]/m;
};

template<int SIZE,class ELEMENT>
double VectorConstantSize<SIZE,ELEMENT>::dotProduct(
		VectorConstantSize &v2) const{
        double result=0.0; 
        for (int i=0; i<SIZE;++i){
                result += (v[i]*v2.v[i]);
        }
        return (result);
};
template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT> &
VectorConstantSize<SIZE,ELEMENT>::operator=(const VectorConstantSize & aa){
        v = aa.v;
        return *this;
};
template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator+(const VectorConstantSize & aa)const{
        vector <ELEMENT> result;
        for (int i=0; i< SIZE; ++i){
        	result.push_back(v[i] + aa.v[i]);
        }
        return VectorConstantSize(result);
};
template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator-(const VectorConstantSize & aa)const{
        vector <ELEMENT> result;
        for (int i=0; i< SIZE; ++i){
        	result.push_back(v[i]-aa.v[i]);
        }
        return VectorConstantSize(result);
};
template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator-()const{
        vector <ELEMENT> result;
        for (int i=0; i< SIZE; ++i){
                result.push_back(-v[i]);
        }
        return VectorConstantSize(result);
};
template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator*(double s)const{
        vector <ELEMENT> result;
        for (int i=0; i< SIZE; ++i){
                result.push_back(v[i]*s);
        }
        return VectorConstantSize(result);
};
template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator/(double s)const{
        vector <ELEMENT> result;
	if (s == 0.0){
                cout << "\nERROR: divide by zero detected"
                        << endl << "       Bye-bye!!!" << endl;
                exit(0);
	}
        for (int i=0; i< SIZE; ++i){
                result.push_back(v[i]/s);
        }
        return VectorConstantSize(result);
};

template<int SIZE,class ELEMENT> bool
VectorConstantSize<SIZE,ELEMENT>::operator==(const VectorConstantSize & aa)const{
        return (v == aa.v);
};
template<int SIZE,class ELEMENT> bool
VectorConstantSize<SIZE,ELEMENT>::operator!=(const VectorConstantSize & aa)const{
        return (v != aa.v);
};

template<int SIZE,class ELEMENT>
void VectorConstantSize<SIZE,ELEMENT>::Read(istream & _is){
        ELEMENT tmp;
        if (v.size() == SIZE) {
                for (int i=0;i<SIZE;++i){
                        _is>>tmp;
                        v[i]=tmp;
                }
        } else {
                for (int i=0;i<SIZE;++i){
                        _is>>tmp;
                        v.push_back(tmp);
                }
        }
};
template<int SIZE,class ELEMENT>
void VectorConstantSize<SIZE,ELEMENT>::Write(ostream & _os)const{
	for (int i=0;i<SIZE;++i)
		_os << v[i] << ' ';
};
/**
  class Vector3<ELEMENT>

  General Description
      Template class derived from base class VectorConstantSize
	but with dimension (ie,size) always equal to 3.
*/

template <class ELEMENT>
class Vector3 : public VectorConstantSize<3,ELEMENT>{
public:

  //===================================================================
  //  Constructors 
  //===================================================================
    Vector3();
    Vector3(const ELEMENT &);
    Vector3(const VectorConstantSize<3,ELEMENT> &);
    Vector3(const ELEMENT &_x,const ELEMENT &_y,const ELEMENT &_z);
    Vector3(istream &);


  //===================================================================
  // Operators
  //===================================================================

	// Addition, Subtration, Negation, Multiply & Divide by a Constant
    Vector3 operator+ (const Vector3 &) const;
    Vector3 operator- (const Vector3 &) const;
    Vector3 operator- () const;
    Vector3 operator* (double) const;
    Vector3 operator/ (double) const;

  //===================================================================
  // Other Methods
  //===================================================================

	// Magnitude of a vector
    double magnitude ();
    const double normsqr() const { return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]; }

	// Cross Product Operation
    inline Vector3 crossProduct (Vector3&) const;

	// Get various data element values
    inline ELEMENT getX()const;
    inline ELEMENT getY()const;
    inline ELEMENT getZ()const;

};

//=====================================================================
// Guts of Vector3 constructors & methods
//=====================================================================
template<class ELEMENT>
Vector3<ELEMENT>::Vector3()
		:VectorConstantSize<3,ELEMENT>(){
};
template<class ELEMENT>
Vector3<ELEMENT>::Vector3(const VectorConstantSize<3,ELEMENT> &c)
		:VectorConstantSize<3,ELEMENT>(c){
};
template<class ELEMENT>
Vector3<ELEMENT>::Vector3(const ELEMENT &assignVal)
		:VectorConstantSize<3,ELEMENT>(assignVal){
};
template<class ELEMENT>
Vector3<ELEMENT>::Vector3
	(const ELEMENT &_x,const ELEMENT &_y,const ELEMENT &_z)
		:VectorConstantSize<3,ELEMENT>(){
     if(v.size() == 0) {
	v.push_back(_x); v.push_back(_y); v.push_back(_z);
     } else if(v.size() == 3) {
	v[0] = _x; v[1] = _y; v[2] = _z;
     } else {
	cout << " Error in Vector3D(x,y,z) constructor " << endl;
	exit(-1);
     }

};
template<class ELEMENT>
Vector3<ELEMENT>::Vector3(istream & _is)
		:VectorConstantSize<3,ELEMENT>(_is){
};
template<class ELEMENT>
Vector3<ELEMENT> Vector3<ELEMENT>::operator+(
		const Vector3<ELEMENT> &tmp) const{
	return (Vector3(VectorConstantSize<3,ELEMENT>::operator+(tmp) ));
};
template<class ELEMENT>
Vector3<ELEMENT> Vector3<ELEMENT>::operator-(
		const Vector3<ELEMENT> &tmp) const{
	return (Vector3(VectorConstantSize<3,ELEMENT>::operator-(tmp) ));
};

template<class ELEMENT>
Vector3<ELEMENT> Vector3<ELEMENT>::operator-() const{
         return Vector3( VectorConstantSize<3,ELEMENT>::operator-() );
};
template<class ELEMENT>
Vector3<ELEMENT> Vector3<ELEMENT>::operator*(double s) const{
         return Vector3( VectorConstantSize<3,ELEMENT>::operator*(s) );
};
template<class ELEMENT>
Vector3<ELEMENT> Vector3<ELEMENT>::operator/(double s) const{
         return Vector3( VectorConstantSize<3,ELEMENT>::operator/(s) );
};
template<class ELEMENT>
double Vector3<ELEMENT>::magnitude() {
         return ( VectorConstantSize<3,ELEMENT>::magnitude() );
};
template<class ELEMENT>
Vector3<ELEMENT> Vector3<ELEMENT>::crossProduct(
		Vector3 &w) const{
	return (Vector3(
		  v[Yy]*w.v[Zz] - w.v[Yy]*  v[Zz],
		w.v[Xx]*  v[Zz] -   v[Xx]*w.v[Zz],
		  v[Xx]*w.v[Yy] - w.v[Xx]*  v[Yy]
		));
};

template<class ELEMENT>
ELEMENT Vector3<ELEMENT>::getX()const{
	return v[Xx];
};
template<class ELEMENT>
ELEMENT Vector3<ELEMENT>::getY()const{
	return v[Yy];
};
template<class ELEMENT>
ELEMENT Vector3<ELEMENT>::getZ()const{
	return v[Zz];
};

/**
  class Vector6<ELEMENT>

  General Description
      Template class derived from base class VectorConstantSize
      but with dimension (ie,size) always equal to 6.
*/

template <class ELEMENT>
class Vector6 : public VectorConstantSize<6,ELEMENT>{
public:

  //===================================================================
  //  Constructors 
  //===================================================================
    Vector6();
    Vector6(const ELEMENT &);
    Vector6(const VectorConstantSize<6,ELEMENT> &);
    Vector6(const ELEMENT &_x,const ELEMENT &_y,const ELEMENT &_z,
             const ELEMENT &_r,const ELEMENT &_p,const ELEMENT &_yaw);
    Vector6(istream &);

  //===================================================================
  // Operators
  //===================================================================
        // Addition, Subtration, Negation, Multiply & Divide by a Constant
    Vector6 operator+ (const Vector6&) const;
    Vector6 operator- (const Vector6&) const;
    Vector6 operator- () const;
    Vector6 operator* (double) const;
    Vector6 operator/ (double) const;

  //===================================================================
  // Other Methods
  //===================================================================

	// Magnitude of a vector
    double magnitude ();

        // Get various data element values
    inline ELEMENT getX()const;
    inline ELEMENT getY()const;
    inline ELEMENT getZ()const;
    inline ELEMENT getRoll()const;
    inline ELEMENT getPitch()const;
    inline ELEMENT getYaw()const;

};

//=====================================================================
// Guts of Vector6 constructors & methods
//=====================================================================
template<class ELEMENT>
Vector6<ELEMENT>::Vector6()
		:VectorConstantSize<6,ELEMENT>(){
};
template<class ELEMENT>
Vector6<ELEMENT>::Vector6(const VectorConstantSize<6,ELEMENT> &c)
		:VectorConstantSize<6,ELEMENT>(c){
};
template<class ELEMENT>
Vector6<ELEMENT>::Vector6(const ELEMENT &assignVal)
		:VectorConstantSize<6,ELEMENT>(assignVal){
};
template<class ELEMENT>
Vector6<ELEMENT>::Vector6
	(const ELEMENT &_x,const ELEMENT &_y,const ELEMENT &_z,
	 const ELEMENT &_r,const ELEMENT &_p,const ELEMENT &_yaw)
		:VectorConstantSize<6,ELEMENT>(){
     if(v.size() == 0) {
	v.push_back(_x); v.push_back(_y); v.push_back(_z);
	v.push_back(_r); v.push_back(_p); v.push_back(_yaw);
     } else if(v.size() == 6) {
	v[0] = _x; v[1] = _y; v[2] = _z;
	v[3] = _r; v[4] = _p; v[5] = _yaw;
     } else {
	cout << " Error in Vector6<ELEMENT>(x,y,z,r,p,w) constructor " << endl;
	exit(-1);
     }
};
template<class ELEMENT>
Vector6<ELEMENT>::Vector6(istream & _is)
                :VectorConstantSize<6,ELEMENT>(_is){
};

template<class ELEMENT>
Vector6<ELEMENT> Vector6<ELEMENT>::operator+(
		const Vector6<ELEMENT> &tmp) const{
	return (Vector6(VectorConstantSize<6,ELEMENT>::operator+(tmp) ));
};
template<class ELEMENT>
Vector6<ELEMENT> Vector6<ELEMENT>::operator-(
		const Vector6<ELEMENT> &tmp) const{
	return (Vector6(VectorConstantSize<6,ELEMENT>::operator-(tmp) ));
};

template<class ELEMENT>
Vector6<ELEMENT> Vector6<ELEMENT>::operator-() const {
         return Vector6( VectorConstantSize<6,ELEMENT>::operator-() );
};
template<class ELEMENT>
Vector6<ELEMENT> Vector6<ELEMENT>::operator*(double s) const{
         return Vector6( VectorConstantSize<6,ELEMENT>::operator*(s) );
};
template<class ELEMENT>
Vector6<ELEMENT> Vector6<ELEMENT>::operator/(double s) const{
         return Vector6( VectorConstantSize<6,ELEMENT>::operator/(s) );
};
template<class ELEMENT>
double Vector6<ELEMENT>::magnitude() {
         return ( VectorConstantSize<6,ELEMENT>::magnitude() );
};

template<class ELEMENT>
ELEMENT Vector6<ELEMENT>::getX()const{
	return v[Xx];
};
template<class ELEMENT>
ELEMENT Vector6<ELEMENT>::getY()const{
	return v[Yy];
};
template<class ELEMENT>
ELEMENT Vector6<ELEMENT>::getZ()const{
	return v[Zz];
};
template<class ELEMENT>
ELEMENT Vector6<ELEMENT>::getRoll()const{
	return v[ROLLroll];
};
template<class ELEMENT>
ELEMENT Vector6<ELEMENT>::getPitch()const{
	return v[PITCHpitch];
};
template<class ELEMENT>
ELEMENT Vector6<ELEMENT>::getYaw()const{
	return v[YAWyaw];
};
#endif
