// $Id$

/**@file Transformation.h
  *@author Aaron Michalk
  *@date 2/25/98
  */

///////////////////////////////////////////////////////////////////////////////////////////

#ifndef Transformation_h
#define Transformation_h

///////////////////////////////////////////////////////////////////////////////////////////
//Include general header
#include <fstream.h>

///////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM header
#include "Orientation.h"

///////////////////////////////////////////////////////////////////////////////////////////
// Forward declaration
class DHparameters;

///////////////////////////////////////////////////////////////////////////////////////////
/** This class contains transformational information and operations.
  * Orientation and position of object are stored in the instance of this class.
  * Position of object are stored as instance of Vector3D.
  */
class Transformation {
public:
    //-----------------------------------------------------------
    //  Static Data
    //-----------------------------------------------------------
    static const Transformation Identity; ///< Indentity transform

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name  Constructors and Destructor*/
    //-----------------------------------------------------------
    //@{
    ///Create a Transformation with position (0,0,0) and a matrix represented orientation.
    Transformation();
    
    ///Create a Transformation with given position and orientation.
    Transformation(const Orientation & _orientation, const Vector3D & _position);

    ///@todo what is DHparameters??
    Transformation(const DHparameters & _dhparameters);

    ///Copy constructor
    Transformation(const Transformation & _t);
    ///Destructor. Do nothing.
    ~Transformation();
    //@}
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Operator Overloadings
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name Operator Overload with other Transformation or Vector*/
    //-----------------------------------------------------------
    //@{
    ///return a vector which is (_vector*Orientation+position)
    Vector3D operator*(const Vector3D & _vector);
    /*This acts like +=.
     *The orientation of this instance is changed to (orientation+_t.orientation)
     *The position of this instance is changed to (position+_t.position)
     *this instance is returned.
     */
    Transformation & operator+(const Transformation & _t);
    /*This acts like -=.
     *The orientation of this instance is changed to (orientation-_t.orientation)
     *The position of this instance is changed to (position-_t.position)
     *this instance is returned.
     */
    Transformation operator-(const Transformation & _t);
    /**Return a new Transformation which is 
      *The new orientation will be (orientation * _t.orientation)
      *The new position will be (orientation * _t.position + position)
      */
    Transformation operator*(const Transformation & _t);
    ///Copy position and orientation from _t
    Transformation & operator=(const Transformation & _t);
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper methods : I/O and Invert
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name Helper  Methods*/
    //-----------------------------------------------------------
    //@{
    /**Invert this instance of Transformation.
      *This is calculated by Orientation is Orientation::Invert
      *position is -(Orientation::Invert * position)
      *@see Orientation::Invert
      */
    void Invert();

    ///Create a new Transformation which is inverse of this Transformation.
    Transformation Inverse();

    void Read(ifstream & _is);  ///<Read position and orientation from _is
    void Write(ostream & _os);  ///<Write position and orientation to _os
    //@}

    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------
    Vector3D position;          ///<Translation
    Orientation orientation;    ///<Rotation

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};

#endif