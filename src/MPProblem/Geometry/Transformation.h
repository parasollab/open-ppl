/** This class contains transformational information and operations
* Orientation and position of object are stored in the instance of this class.
* Position of object are stored as instance of Vector3D.
*/
               
#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_

#include "Orientation.h"

class DHparameters;

class Transformation {
  public:
    //-----------------------------------------------------------
    //  Static Data
    //-----------------------------------------------------------
    //in this case, we are creating inside each instance of Transformation,
    //another instance of Transformation called Identity.  This Identity
    //Transformation object will be the same within each unique instance of
    //Transformation. We can use this object to zero-out everything on our
    //transformation using the assignment operator.
    static const Transformation Identity; ///< Indentity transform

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Create a Transformation with position (0,0,0) and a matrix represented orientation.
    Transformation();

    ///Create a Transformation with given position and orientation.
    Transformation(const Orientation& _orientation, const Vector3D& _position);

    //Create a Transformation out of a vector3D and interpret the orientation as _type
    Transformation(const vector<double>& _configuration, Orientation::OrientationType _type = Orientation::FixedXYZ);

    //This constructor will take the four values from DHparameters and insert
    //them into the constructed Transformation object.  Essentially this sets
    //the reference frame for this particular Transformation.
    Transformation(const DHparameters& _dhparameters);

    Transformation(const Transformation& _t);

    ~Transformation();

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Operator Overloadings
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///return a vector which is (_vector*Orientation+position)
    Vector3D operator*(const Vector3D& _vector);

    /*The orientation of this instance is changed to (orientation+_t.orientation)
     *The position of this instance is changed to (position+_t.position)
     *this instance is returned.
     */
    Transformation& operator+(const Transformation& _t);

    /*The orientation of this instance is changed to (orientation-_t.orientation)
     *The position of this instance is changed to (position-_t.position)
     *this instance is returned.
     */
    Transformation& operator-(const Transformation& _t);

    /**Return a new Transformation which is 
     *The new orientation will be (orientation * _t.orientation)
     *The new position will be (orientation * _t.position + position)
     */
    Transformation operator*(const Transformation& _t);

    Transformation& operator=(const Transformation& _t);

    bool operator==(const Transformation& t) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Helper methods : I/O and Invert
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    void Invert();

    ///Create a new Transformation which is inverse of this Transformation.
    Transformation Inverse();

    friend istream& operator>>(istream& _is, Transformation& _t);
    friend ostream& operator<<(ostream& _os, const Transformation& _t);

    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------
    Vector3D m_position;          ///<Translation
    Orientation m_orientation;    ///<Rotation

};

#endif
