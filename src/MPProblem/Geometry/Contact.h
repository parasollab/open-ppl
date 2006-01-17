// $Id$

/**@file Contact.h
   @author Aaron Michalk
   @date 2/25/98 
*/

///////////////////////////////////////////////////////////////////////////////////////////

#ifndef Contact_h
#define Contact_h

///////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "Transformation.h"

///////////////////////////////////////////////////////////////////////////////////////////
// Forward declaration
class Body;

class Contact {
public:

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
    Contact();
    Contact(Body * _body1,  Body * _body2, Vector3D & _position, Vector3D & _normal1, Vector3D & _normal2);
    ~Contact();
    //@}

    Body *GetBody(int _index);
    Transformation & GetUtransformToContact(int _index);
    Vector3D & GetPosition();
    Vector3D & GetNormal(int _index);
    Vector3D & GetTangential(int _index);
    Vector3D & GetOrthogonal(int _index);
    void ComputeTangential();
    void ComputeOrthogonal();
    void ComputeTransform();

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

private:
    //-----------------------------------------------------------
    //  Data
    //  Conventions:
    //       body1 - belonging to manipulator (free body)
    //       body2 - belonging to obstacle (fixed body)
    //-----------------------------------------------------------
    Body *body[2];  // Body identities
    Transformation UtransformToContact[2];
    Vector3D position;
    Vector3D normal[2];
    Vector3D tangential[2];
    Vector3D orthogonal[2];
};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//  Implementation of Contact (inline methods)
//
//
//////////////////////////////////////////////////////////////////////////////////////////

inline Body * Contact::GetBody(int _index){
  return body[_index];
}

inline Transformation & Contact::GetUtransformToContact(int _index){
  return UtransformToContact[_index];
}

inline Vector3D & Contact::GetPosition(){
  return position;
}

inline Vector3D & Contact::GetNormal(int _index){
  return normal[_index];
}

inline Vector3D & Contact::GetTangential(int _index){
  return tangential[_index];
}

inline Vector3D & Contact::GetOrthogonal(int _index){
  return orthogonal[_index];
}


#endif
