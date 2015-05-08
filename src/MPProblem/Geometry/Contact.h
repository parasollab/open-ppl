#ifndef CONTACT_H_
#define CONTACT_H_

#include <Transformation.h>
using namespace mathtool;

class Body;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
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
    Contact(Body* _body1, Body* _body2, Vector3d& _position, Vector3d& _normal1, Vector3d& _normal2);
    ~Contact();
    //@}

    Body* GetBody(int _index);
    Transformation& GetUtransformToContact(int _index);
    Vector3d& GetPosition();
    Vector3d& GetNormal(int _index);
    Vector3d& GetTangential(int _index);
    Vector3d& GetOrthogonal(int _index);
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
    Body* body[2];  // Body identities
    Transformation UtransformToContact[2];
    Vector3d position;
    Vector3d normal[2];
    Vector3d tangential[2];
    Vector3d orthogonal[2];
};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//  Implementation of Contact (inline methods)
//
//
//////////////////////////////////////////////////////////////////////////////////////////

inline Body* Contact::GetBody(int _index){
  return body[_index];
}

inline Transformation& Contact::GetUtransformToContact(int _index){
  return UtransformToContact[_index];
}

inline Vector3d& Contact::GetPosition(){
  return position;
}

inline Vector3d& Contact::GetNormal(int _index){
  return normal[_index];
}

inline Vector3d& Contact::GetTangential(int _index){
  return tangential[_index];
}

inline Vector3d& Contact::GetOrthogonal(int _index){
  return orthogonal[_index];
}

#endif
