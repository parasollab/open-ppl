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

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef VectorConstantSize_h
#define VectorConstantSize_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include general headers
#include <vector.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "BasicDefns.h"

////////////////////////////////////////////////////////////////////////////////////////////
/**@name 3D rigit body Cfg index*/
//@{
#define Xx 	0
#define Yy 	1
#define Zz	2
#define ROLLroll 	3
#define PITCHpitch	4
#define YAWyaw  	5
//@}

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *class VectorConstantSize<SIZE,ELEMENT>.
  *
  *General Description
  *    Base class for all vectors of constant size.
  */
////////////////////////////////////////////////////////////////////////////////////////////

template <int SIZE,class ELEMENT>
class VectorConstantSize{
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  //===================================================================
  /**@name  Constructors and Destructor*/
  //===================================================================
  //@{
	/**Reserve SIZE memory blocks and fill all of them as 0.0
	  *@note what happens when ELEMENT type is not float point or not numbers??
	  */
    VectorConstantSize();
	///Copy constructor
    VectorConstantSize(const VectorConstantSize&);
	///Reserve SIZE memory blocks and fill all of them as given ELEMENT
    VectorConstantSize(const ELEMENT&);
	/**Copy given std::vector and copy its values to this instance.
	  *@note this given vector should have the same size as this VectorConstantSize instance.
	  */
    VectorConstantSize(const vector<ELEMENT>&);
	/**Reserve SIZE memory blocks and read data from given inputstream
	  *@warning Should this throw exception?
	  */
    VectorConstantSize(istream &);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  //===================================================================
  /** Operators*/
  //===================================================================
  //@{
	/// Assignment
    VectorConstantSize& operator=  (const VectorConstantSize &); 

	/// Addition, Subtration, Negation, Multiply & Divide by a Constant
    VectorConstantSize  operator+  (const VectorConstantSize &)const; 
    VectorConstantSize  operator-  (const VectorConstantSize &)const;
    VectorConstantSize  operator-  () const;
    VectorConstantSize  operator*  (double)const;
    VectorConstantSize  operator/  (double)const;
    ELEMENT & operator[] (int s) { return v[s]; }
    const ELEMENT & operator[] (int s) const { return v[s]; }

	/// Tests for Equality and Inequality
    bool operator== (const VectorConstantSize &)const; 
    bool operator!= (const VectorConstantSize &)const; 
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Helper methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  //===================================================================
  /** Helper Methods*/
  //===================================================================
  //@{
	
	/**
	 *set vector value by arrary, this array should have size "SIZE".
     *This method will NOT check the length of given array, if
	 *the actural size of given array is different from Size of this instance
	 *, memory access violation might occur.
	 *@param pElem A array of length "SIZE"
	 *@postcondition #v will contain values in pElem
	 */
	virtual void setValue( const ELEMENT * pElem );

	/// Query for dimension(ie, size) of vector
    inline int size() const;

	/// Magnitude of a vector. return = sqrt( (V1*V1)+(V2*V2)... )
    double magnitude() const;

	/// Change values of vector so as to Normalize it
    inline void normalize();

	/// Dot Product Operation. Vnew = (V11*V21)+(V12*V22)+(V13*V23)+....
    inline double dotProduct(VectorConstantSize&) const;

	/// I/O for vector
    void Read(istream&) ;
    inline void Write(ostream&) const;
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected data members and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

protected:
  //===================================================================
  // Data
  //===================================================================
    vector<ELEMENT> v; ///< The storage of VectorConstantSize. A std::vector
};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//	Implementation of VectorConstantSize
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//=====================================================================
//
// Guts of VectorConstantSize constructors
//
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

//=====================================================================
//
// Operators
//
//=====================================================================
template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT> &
VectorConstantSize<SIZE,ELEMENT>::operator=(const VectorConstantSize & aa){
        v = aa.v;
        return *this;
};

template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator+(const VectorConstantSize & aa)const{
        vector <ELEMENT> result;
        for (int i=0; i< SIZE; ++i)
		{
        	result.push_back(v[i] + aa.v[i]);
        }
        return VectorConstantSize(result);
};

template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator-(const VectorConstantSize & aa)const{
        vector <ELEMENT> result;
        for (int i=0; i< SIZE; ++i)
		{
        	result.push_back(v[i]-aa.v[i]);
        }
        return VectorConstantSize(result);
};

template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator-()const{
        vector <ELEMENT> result;
        for (int i=0; i< SIZE; ++i)
		{
                result.push_back(-v[i]);
        }
        return VectorConstantSize(result);
};

template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator*(double s)const{
        vector <ELEMENT> result;
        for (int i=0; i< SIZE; ++i)
		{
                result.push_back(v[i]*s);
        }
        return VectorConstantSize(result);
};

template<int SIZE,class ELEMENT> VectorConstantSize<SIZE,ELEMENT>
VectorConstantSize<SIZE,ELEMENT>::operator/(double s)const{
        vector <ELEMENT> result;
		if (s == 0.0)
		{
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

//=====================================================================
//
// Helper functions
//
//=====================================================================

template<int SIZE,class ELEMENT>
void VectorConstantSize<SIZE,ELEMENT>::setValue( const ELEMENT * pElem )
{
	if( pElem==NULL ) return; //huh.. given value is null, give up

	//clean elements in v
	v.clear();

	for( int i=0;i<SIZE;i++ )
	{
		v.push_back(pElem[i]);
	}
}

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

template<int SIZE,class ELEMENT> double 
VectorConstantSize<SIZE,ELEMENT>::dotProduct(VectorConstantSize &v2) const{
    double result=0.0; 
    for (int i=0; i<SIZE;++i)
	{
      result += (v[i]*v2.v[i]);
    }
    return (result);
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

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//	Vector3
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

/**
  * class Vector3<ELEMENT>
  *
  * General Description
  *    Template class derived from base class VectorConstantSize
  *	but with dimension (ie,size) always equal to 3.
  *
  */

template <class ELEMENT>
class Vector3 : public VectorConstantSize<3,ELEMENT>{
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  //===================================================================
  /**@name  Constructors and Destructor*/
  //===================================================================
  //@{
	/**create a (0.0,0.0,0.0) vector.
	  *@note what happens when ELEMENT type is not float point or not numbers??
	  */
    Vector3();
	///Reserve SIZE memory blocks and fill all of them as given ELEMENT
    Vector3(const ELEMENT &);
	///Copy constructor
    Vector3(const VectorConstantSize<3,ELEMENT> &);
	///Create this instance by given 3 elemets.
    Vector3(const ELEMENT &_x,const ELEMENT &_y,const ELEMENT &_z);
	/**Reserve SIZE memory blocks and read data from given inputstream
	  *@warning Should this throw exception?
	  */
    Vector3(istream &);
  //@}

  //===================================================================
  /**Operators.*/
  //===================================================================
  //@{
	// Addition, Subtration, Negation, Multiply & Divide by a Constant
    /*Do nothing special
	Vector3 operator+ (const Vector3 &) const;
    Vector3 operator- (const Vector3 &) const;
    Vector3 operator- () const;
    Vector3 operator* (double) const;
    Vector3 operator/ (double) const;*/
  //@}

  //===================================================================
  /** Other Methods*/
  //===================================================================
  //@{
	/// Magnitude of a vector
    //double magnitude ();

	///Return (V1*V1)+(V2*V2)+(V3*V3)
    const double normsqr() const { return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]; }

	/// Cross Product Operation.
    inline Vector3 crossProduct (Vector3&) const;

    // Get various data element values
    inline ELEMENT getX()const;	///<Get the first element in vector
    inline ELEMENT getY()const; ///<Get the second element in vector
    inline ELEMENT getZ()const; ///<Get the third element in vector
  //@}
};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//	Implementation of Vector3
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//=====================================================================
//
// Guts of Vector3 constructors
//
//=====================================================================
template<class ELEMENT>
Vector3<ELEMENT>::Vector3():VectorConstantSize<3,ELEMENT>(){
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
Vector3<ELEMENT>::Vector3(const ELEMENT &_x,const ELEMENT &_y,const ELEMENT &_z)
:VectorConstantSize<3,ELEMENT>(){

    if(v.size() == 0) 
	{
		v.push_back(_x); v.push_back(_y); v.push_back(_z);
    } 
	else if(v.size() == 3) 
	{
		v[0] = _x; v[1] = _y; v[2] = _z;
    }
	else 
	{
		cout << " Error in Vector3D(x,y,z) constructor " << endl;
		exit(-1);
    }
};

template<class ELEMENT>
Vector3<ELEMENT>::Vector3(istream & _is)
		:VectorConstantSize<3,ELEMENT>(_is){
};

//=====================================================================
//
// Guts of Vector3 constructors (Why these all call its parent's method
// and without doing anything else? These methods are inherent from base
// class, you could o the same thing without writing ant code.)
//
//=====================================================================
/*
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
*/
//=====================================================================
//
// Helper functions
//
//=====================================================================
/*
template<class ELEMENT>
double Vector3<ELEMENT>::magnitude() {
         return ( VectorConstantSize<3,ELEMENT>::magnitude() );
};*/

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

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//	Vector6
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

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
  /**@name  Constructors */
  //===================================================================
  //@{
	/**create a (0.0,0.0,0.0,0.0,0.0,0.0) vector.
	  *@note what happens when ELEMENT type is not float point or not numbers??
	  */
    Vector6();
	///Reserve SIZE memory blocks and fill all of them as given ELEMENT
    Vector6(const ELEMENT &);
	///Copy constructor
    Vector6(const VectorConstantSize<6,ELEMENT> &);

	///Create this instance by given 6 elemets (in order) .
    Vector6(const ELEMENT &_x,const ELEMENT &_y,const ELEMENT &_z,
             const ELEMENT &_r,const ELEMENT &_p,const ELEMENT &_yaw);

	/**Reserve SIZE memory blocks and read data from given inputstream
	  *@warning Should this throw exception?
	  */
    Vector6(istream &);
  //@}

  //===================================================================
  // Operators
  //===================================================================
    // Addition, Subtration, Negation, Multiply & Divide by a Constant
   /*comment out do nothing special than its parent!!
    Vector6 operator+ (const Vector6&) const;
    Vector6 operator- (const Vector6&) const;
    Vector6 operator- () const;
    Vector6 operator* (double) const;
    Vector6 operator/ (double) const;*/

  //===================================================================
  /**@name Helper Methods*/
  //===================================================================
  //@{
	// Magnitude of a vector
    //double magnitude ();

    // Get various data element values
    inline ELEMENT getX()const;	///<Get the first element in vector
    inline ELEMENT getY()const;	///<Get the second element in vector
    inline ELEMENT getZ()const;	///<Get the third element in vector
    inline ELEMENT getRoll()const;	///<Get the fourth element in vector
    inline ELEMENT getPitch()const;	///<Get the fifth element in vector;
    inline ELEMENT getYaw()const;	///<Get the sixth element in vector
  //@}
};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//	Implementation of Vector6
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

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

//=====================================================================
//
// Operator
//
//=====================================================================

/*
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
*/

//=====================================================================
//
// Helper functions
//
//=====================================================================

/*template<class ELEMENT>
double Vector6<ELEMENT>::magnitude() {
         return ( VectorConstantSize<6,ELEMENT>::magnitude() );
};*/

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
