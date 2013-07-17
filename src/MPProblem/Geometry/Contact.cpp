#include "Contact.h"

#define PLANE_SIZE  5.0


//-------------------------------------------------------------------
//  Constructors and Destructor
//-------------------------------------------------------------------
Contact::Contact() {
}

Contact::Contact(Body *_body1,  Body * _body2, Vector3d & _position, Vector3d & _normal1, Vector3d & _normal2) {
  body[0] = _body1;
  body[1] = _body2;
  position = _position;
  normal[0] = _normal1;
  normal[1] = _normal2;

  // Get the tangential and orthogonal from the normals 
  // in order to prepare for the subsequent computations 
  ComputeTangential();
  ComputeOrthogonal();
}

Contact::~Contact() {
}

void Contact::ComputeTangential(){
  Vector3d direction;
  double mat[3][3];

    // Get the rotation matrix for 90 degree w.r.t y-axis
    mat[0][0] =  0.0;    mat[0][1] =  0.0;    mat[0][2] = 1.0;
    mat[1][0] =  0.0;    mat[1][1] =  1.0;    mat[1][2] = 0.0;
    mat[2][0] = -1.0;    mat[2][1] =  0.0;    mat[2][2] = 0.0;
  
    Orientation orientation(mat);
    tangential[0] = orientation * normal[0];
    tangential[1] = orientation * normal[1];
}

//===================================================================
//  Alternatively, we can take a cross-product of normal and tangential
//  to get the orthogonal
//===================================================================
void Contact::ComputeOrthogonal(){
  Vector3d direction;
  double mat[3][3];

    // Get the rotation matrix for 90 degree w.r.t x-axis
    mat[0][0] = 1.0;    mat[0][1] =  0.0;    mat[0][2] =  0.0;
    mat[1][0] = 0.0;    mat[1][1] =  0.0;    mat[1][2] = -1.0;
    mat[2][0] = 0.0;    mat[2][1] =  1.0;    mat[2][2] =  0.0;
  
    Orientation orientation(mat);
    orthogonal[0] = orientation * normal[0];
    orthogonal[1] = orientation * normal[1];
}


//===================================================================
//  Check for the correct semantics
//===================================================================
void Contact::ComputeTransform(){
  double mat[3][3];


  mat[0][0] = normal[0][0];  mat[1][0] = normal[0][1];  mat[2][0] = normal[0][2];
  mat[0][1] = tangential[0][0];  mat[1][1] = tangential[0][1];  mat[2][1] = tangential[0][2];
  mat[0][2] = orthogonal[0][0];  mat[1][2] = orthogonal[0][1];  mat[2][2] = orthogonal[0][2];

  // Get the transformation (w.r.t world) for the second body
  Orientation orientation(mat);
  UtransformToContact[0] = Transformation(position, orientation);


  mat[0][0] = normal[1][0];  mat[1][0] = normal[1][1];  mat[2][0] = normal[1][2];
  mat[0][1] = tangential[1][0];  mat[1][1] = tangential[1][1];  mat[2][1] = tangential[1][2];
  mat[0][2] = orthogonal[1][0];  mat[1][2] = orthogonal[1][1];  mat[2][2] = orthogonal[1][2];

  // Get the transformation (w.r.t world) for the first body
  orientation = Orientation(mat);
  UtransformToContact[1] = Transformation(position, orientation);
}
