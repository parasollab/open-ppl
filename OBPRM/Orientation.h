// $Id$
/////////////////////////////////////////////////////////////////////
//  Orientation.h
//
//  Created   2/25/98 Aaron Michalk
//  Modified  4/13/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#ifndef Orientation_h
#define Orientation_h

#include "Vectors.h"

const double IdentityMatrix[3][3] = {{1, 0, 0},
				     {0, 1, 0}, 
				     {0, 0, 1}};

class Orientation {
public:
    //-----------------------------------------------------------
    //  Enumerations
    //-----------------------------------------------------------
    enum OrientationType {
	Matrix   = 0,
	EulerXYZ = 1,  FixedZYX = 1,
	EulerXZY = 2,  FixedYZX = 2,
	EulerYXZ = 3,  FixedZXY = 3,
	EulerYZX = 4,  FixedXZY = 4,
	EulerZXY = 5,  FixedYXZ = 5,
	EulerZYX = 6,  FixedXYZ = 6,
	EulerXYX = 7,  FixedXYX = 7,
	EulerXZX = 8,  FixedXZX = 8,
	EulerYXY = 9,  FixedYXY = 9,
	EulerYZY = 10, FixedYZY = 10,
	EulerZXZ = 11, FixedZXZ = 11,
	EulerZYZ = 12, FixedZYZ = 12,
	Quaternion = 20,
	Rodriques  = 21
    };
    //-----------------------------------------------------------
    //  Static Data
    //-----------------------------------------------------------
    static const Orientation Identity;
    //-----------------------------------------------------------
    //  Constructors and Destructor
    //-----------------------------------------------------------
    Orientation();
    Orientation(OrientationType _type);
    Orientation(const double _matrix[3][3]);
//    Orientation(double alpha, double beta, double gamma);
    Orientation(OrientationType _type, double _alpha, double _beta, double _gamma);
    Orientation(const Orientation & _o);
    Orientation(double _rotationAngle, const Vector3D &_rotationAxis); // quaternion
    ~Orientation();
    //-----------------------------------------------------------
    //  Operators
    //-----------------------------------------------------------
    Vector3D operator*(const Vector3D & _v);
    Orientation operator*(const Orientation & _orientation);
    Orientation operator+(const Orientation & _orientation);
    Orientation operator-(const Orientation & _orientation);
    Orientation & operator=(const Orientation & _o);
    double & operator()(int _row, int _col);

    //-----------------------------------------------------------
    //  Methods
    //-----------------------------------------------------------
    void Invert();
    Orientation Inverse();
//    void Normalize(Vector3D & v);  // for quaternion
    void ConvertType(OrientationType _newType);
    void Read(istream & _is);
    void Write(ostream & _os);
    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------
    OrientationType type;

    // Matrix representation
    double matrix[3][3];

    // Euler angles
    double alpha;
    double beta;
    double gamma;

    // Quaternion notation
    // We assume that the vector [rotationAngle, rotationAxis] is normalized
    double rotationAngle;
    Vector3D rotationAxis;

    double epsilon1;
    double epsilon2;
    double epsilon3;
    double epsilon4;

protected:
private:
};

//===============================================================
// Inline functions
//===============================================================
inline double & Orientation::operator()(int _row, int _col) {
    return matrix[_row-1][_col-1];
}

#endif

