// $Id$
/////////////////////////////////////////////////////////////////////
//  Orientation.c
//
//  Created   3/ 1/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#include "Orientation.h"

#define EPSILON  1.0e-6


//===================================================================
//  Static Data Initialization
//===================================================================
const Orientation Orientation::Identity = Orientation(IdentityMatrix);


//===================================================================
//  Constructors and Destructor
//===================================================================
Orientation::Orientation(OrientationType _type) {
    type = _type;

    if (type == Matrix) {
        for (int i=0; i < 3; i++)
            for (int j=0; j < 3; j++)
                if (i == j)
	            matrix[i][j] = 1.0;
                else
	            matrix[i][j] = 0.0;
    } else if (type == Quaternion) {
        //-----------------------------------------------------------
        // TODO
        //-----------------------------------------------------------
    } else {
        alpha = 0.0;
	beta  = 0.0;
	gamma = 0.0;
    }
}

Orientation::Orientation(const double _matrix[3][3]) {
    type = Matrix;

    for (int i=0; i < 3; i++)
        for (int j=0; j < 3; j++) {
	    matrix[i][j] = _matrix[i][j];
	}
}

Orientation::Orientation(OrientationType _type, double _alpha, double _beta, double _gamma) {
    type  = _type;

    alpha = _alpha;
    beta  = _beta;
    gamma = _gamma;

}

Orientation::Orientation(double _rotationAngle, const Vector3D &_rotationAxis) {
    type  = Quaternion;

    rotationAngle = _rotationAngle;
    rotationAxis = _rotationAxis;

}

//-------------------------------------------------------------------
// If no argument is assumed, it is considered as having Euler angles
//-------------------------------------------------------------------
Orientation::Orientation() {
    type  = EulerXYZ;

    alpha = 0;
    beta  = 0;
    gamma = 0;
}

Orientation::Orientation(const Orientation & _o) {
    *this = _o;
}

Orientation::~Orientation() {
}

//===================================================================
//  Operators
//===================================================================
Vector3D Orientation::operator*(const Vector3D & _v) {
    ConvertType(Matrix);

    return Vector3D(
       matrix[0][0]*_v.getX() + matrix[0][1]*_v.getY() + matrix[0][2]*_v.getZ(),
       matrix[1][0]*_v.getX() + matrix[1][1]*_v.getY() + matrix[1][2]*_v.getZ(),
       matrix[2][0]*_v.getX() + matrix[2][1]*_v.getY() + matrix[2][2]*_v.getZ());
}

Orientation Orientation::operator*(const Orientation & _changed) {
    ConvertType(Matrix);
    Orientation _o(_changed);
    _o.ConvertType(Matrix);
    
    Orientation result(Matrix);
    for (int i=0; i < 3; i++)
        for (int j=0; j < 3; j++) {
            result.matrix[i][j] = matrix[i][0]*_o.matrix[0][j];
            for (int k=1; k < 3; k++)
                result.matrix[i][j] += matrix[i][k]*_o.matrix[k][j];
        }
    return result;
}

Orientation Orientation::operator+(const Orientation & _o) {
    ConvertType(EulerXYZ);
    Orientation newO = _o;
    newO.ConvertType(EulerXYZ); // Convert to the EulerXYZ representation, if needed
      
    alpha += newO.alpha;
    beta += newO.beta;
    gamma += newO.gamma;

    ConvertType(Matrix); // Convert back to the Matrix representation

    return *this;
}

Orientation Orientation::operator-(const Orientation & _o) {
    ConvertType(EulerXYZ);
    Orientation newO = _o;
    newO.ConvertType(EulerXYZ); // Convert to the EulerXYZ representation, if needed

    alpha -= newO.alpha;
    beta -= newO.beta;
    gamma -= newO.gamma;

    ConvertType(Matrix); // Convert back to the Matrix representation

    return *this;
}

Orientation & Orientation::operator=(const Orientation & _o) {
    type = _o.type;

    if (type == Matrix) {
        for (int i=0; i < 3; i++)
	    for (int j=0; j < 3; j++)
	        matrix[i][j] = _o.matrix[i][j];
    } else if (type == Quaternion) {
        //-----------------------------------------------------------
	// TODO
        //-----------------------------------------------------------
	rotationAngle = _o.rotationAngle;
	rotationAxis = _o.rotationAxis;
    } else {
        alpha = _o.alpha;
	beta  = _o.beta;
	gamma = _o.gamma;
    }
    return *this;
}

bool Orientation::operator==(const Orientation& _o) const {
  Orientation _o_copy(_o);
  _o_copy.ConvertType(type);
  
  if(type == Matrix) {
    for(int i=0; i<3; ++i)
      for(int j=0; j<3; ++j)
        if(matrix[i][j] != _o_copy.matrix[i][j])
          return false;
    return true;
  } else if (type == Quaternion) {
    return (rotationAngle == _o_copy.rotationAngle) && (rotationAxis == _o_copy.rotationAxis);
  } else {
    return (alpha == _o_copy.alpha) && (beta == _o_copy.beta) && (gamma == _o_copy.gamma);
  }
}

//===================================================================
//  Inverse
//
//  Create new inverted orientation
//===================================================================
Orientation Orientation::Inverse() {
    ConvertType(Matrix);

    Orientation result(Matrix);
    for (int i=0; i < 3; i++)
	for (int j=0; j < 3; j++)
            result.matrix[i][j] = matrix[j][i];
    return result;
}

//===================================================================
//  Invert
//
//  Invert this orientation
//===================================================================
void Orientation::Invert() {
    ConvertType(Matrix);
    double temp;
    for (int i=0; i < 3; i++)
        for (int j=i+1; j < 3; j++) {
            temp = matrix[i][j];
            matrix[i][j] = matrix[j][i];
            matrix[j][i] = temp;
        }
}

//===================================================================
//  ConvertType
//===================================================================
void Orientation::ConvertType(OrientationType _newType) {
    // Don't do the conversion, if there is no need
    if (_newType == type)
        return;

    double sa, ca, sb, cb, sg, cg;

    switch (type) {
      case Matrix:
        switch (_newType) {
	  case Quaternion: {
            double  w, x, y, z;

            double wSquare = 0.25*(1.0 + matrix[0][0] + matrix[1][1] + matrix[2][2]);
            if (wSquare > EPSILON){
               w = sqrt(wSquare);
               x = (matrix[1][2] - matrix[2][1]) / 4.0*w;
               y = (matrix[2][0] - matrix[0][3]) / 4.0*w;
               z = (matrix[0][1] - matrix[1][0]) / 4.0*w;
            }
            else {
               w = 0.0;
               double xSquare = -0.5*(matrix[1][1] + matrix[2][2]);
               if (xSquare > EPSILON){
                  x = sqrt(xSquare);
                  y = matrix[0][1] / 2.0*x;
                  z = matrix[0][2] / 2.0*x;
               }
               else{
                  x = 0.0;
                  double ySquare = 0.5*(1.0 - matrix[2][2]);
                  if (ySquare > EPSILON){
                     y = sqrt(ySquare);
                     z = matrix[1][2] / 2.0*y;
                  }
                  else{
                     y = 0.0;
                     z = 1.0;
                  }
               }
            }

            rotationAngle = w;
            rotationAxis = Vector3D(x,y,z);
	    break;
          }
	  case EulerXYZ:
	    beta = atan2(matrix[0][2], sqrt(matrix[1][2]*matrix[1][2] + matrix[2][2]*matrix[2][2]));
	    if(cos(beta) > 0) {
	        alpha = atan2(-matrix[1][2], matrix[2][2]);
		gamma = atan2(-matrix[0][1], matrix[0][0]);
	    } else {
	        alpha = atan2(matrix[1][2], -matrix[2][2]);
		gamma = atan2(matrix[0][1], -matrix[0][0]);
	    }		    
	    break;
	  case EulerZYX: // FixedXYZ:
            beta = atan2(-matrix[2][0], sqrt(matrix[2][1]*matrix[2][1] + matrix[2][2]*matrix[2][2]));
	    if(cos(beta) > 0) {
	    	alpha = atan2(matrix[1][0], matrix[0][0]);
		gamma = atan2(matrix[2][1], matrix[2][2]);
	    } else {	
                alpha = atan2(-matrix[1][0], -matrix[0][0]);
                gamma = atan2(-matrix[2][1], -matrix[2][2]);
	    }
	  case EulerXZY:
	    // TODO
	    break;
	  // etc.
	}
	break;

      case Quaternion:
        switch (_newType) {
	  case Matrix: {
            double  w = rotationAngle;
            double  x = rotationAxis.getX();
            double  y = rotationAxis.getY();
            double  z = rotationAxis.getZ();

            matrix[0][0] = 1.0 - 2.0*y*y - 2.0*z*z;
            matrix[1][0] = 2.0*x*y - 2.0*w*z;
            matrix[2][0] = 2.0*x*z + 2.0*w*y;

            matrix[0][1] = 2.0*x*y + 2.0*w*z;
            matrix[1][1] = 1.0 - 2.0*x*x - 2.0*z*z;
            matrix[2][1] = 2.0*y*z - 2.0*w*x;

            matrix[0][2] = 2.0*x*z - 2.0*w*y;
            matrix[1][2] = 2.0*y*z + 2.0*w*x;
            matrix[2][2] = 1.0 - 2.0*x*x - 2.0*y*y;
	    break;
          }
	  case EulerXYZ:
	    ConvertType(Matrix);
	    ConvertType(EulerXYZ);
	    break;
	  case EulerZYX: // or FixedXYZ
	    ConvertType(Matrix);
            ConvertType(EulerZYX);
	    break;
	  case EulerXZY:
	    // TODO
	    break;
	  // etc.
	}
	break;

      //-------------------------------------------------------------
      //  If none of the above, then must be Euler/Fixed angle type
      //-------------------------------------------------------------
      default:
	switch (_newType) {
	  case Matrix:
	    sa = sin(alpha);
	    ca = cos(alpha);
	    sb = sin(beta);
	    cb = cos(beta);
	    sg = sin(gamma);
	    cg = cos(gamma);
	    switch (type) {
	      case EulerXYZ:
	        matrix[0][0] = cb*cg;
		matrix[0][1] = -cb*sg;
		matrix[0][2] = sb;
		matrix[1][0] = sa*sb*cg + ca*sg;
		matrix[1][1] = -sa*sb*sg + ca*cg;
		matrix[1][2] = -sa*cb;
		matrix[2][0] = -ca*sb*cg + sa*sg;
		matrix[2][1] = ca*sb*sg + sa*cg;
		matrix[2][2] = ca*cb;
		break;
	      case EulerZYX: // or FixedXYZ
		matrix[0][0] = ca*cb;
		matrix[0][1] = ca*sb*sg - sa*cg;
		matrix[0][2] = ca*sb*cg + sa*sg;
		matrix[1][0] = sa*cb;
		matrix[1][1] = sa*sb*sg + ca*cg;
		matrix[1][2] = sa*sb*cg - ca*sg;
		matrix[2][0] = -sb;
		matrix[2][1] = cb*sg;
		matrix[2][2] = cb*cg;
		break;
	      case EulerXZY:
	        // TODO
		break;
	      //
	      // etc.
	      //
	    }
	    break;
	  case EulerXYZ:
	    switch (type) {
	     case EulerZYX:
		ConvertType(Matrix);
		ConvertType(EulerXYZ);
		break;
	    }
	    break;
	  case EulerZYX:
	    switch (type) {
	     case EulerXYZ:
		ConvertType(Matrix);
		ConvertType(EulerZYX);
		break;
	    }
	    break;
	  case Quaternion:
	    // TODO
	    break;
	  default:
	    // TODO
	    break;
	}
    }

    type = _newType;
}


//===================================================================
//  Read
//===================================================================
void Orientation::Read(istream & _is) {
    type = EulerXYZ;
    _is >> alpha;
    _is >> beta;
    _is >> gamma;

    ConvertType(Matrix);
}

//===================================================================
//  Write
//===================================================================
void Orientation::Write(ostream & _os) {
    ConvertType(EulerZYX);

    _os << gamma << " ";
    _os << beta << " ";
    _os << alpha << " ";
}

