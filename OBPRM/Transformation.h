// $Id$
/////////////////////////////////////////////////////////////////////
//  Transformation.h
//
//  Created   2/25/98 Aaron Michalk
//  Modified  4/13/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#ifndef Transformation_h
#define Transformation_h

#include <fstream.h>
#include "Vectors.h"
#include "Orientation.h"
#include "DHparameters.h"

class DHparameters;


class Transformation {
public:
    //-----------------------------------------------------------
    //  Static Data
    //-----------------------------------------------------------
    static const Transformation Identity;
    //-----------------------------------------------------------
    //  Constructors and Destructor
    //-----------------------------------------------------------
    Transformation();
    Transformation(const Orientation & _orientation, const Vector3D & _position);
    Transformation(const DHparameters & _dhparameters);
    Transformation(const Transformation & _t);
    ~Transformation();
    //-----------------------------------------------------------
    //  Operators
    //-----------------------------------------------------------
    Vector3D operator*(const Vector3D & _vector);
    Transformation & operator+(const Transformation & _t);
    Transformation operator-(const Transformation & _t);
    Transformation operator*(const Transformation & _t);
    Transformation & operator=(const Transformation & _t);
    //-----------------------------------------------------------
    //  Methods
    //-----------------------------------------------------------
    void Invert();
    Transformation Inverse();
    void glTransform();
    void Read(ifstream & _is);
    void Write(ostream & _os);
    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------
    Vector3D position;	
    Orientation orientation;
protected:
private:
};

#endif
