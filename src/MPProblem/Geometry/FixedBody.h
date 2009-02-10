// $Id$

/**@file FixedBody.h
   @date 2/25/98
   @author Aaron Michalk
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FixedBody_h
#define FixedBody_h

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "Body.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Class FreeBody is a stationary Body in workspace.
 * This class provide more specifice methods for manipulate fixed object.
 */
class FixedBody : public Body {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{

    /**Condtructor. Construct a FixedBody and set its owner as _owner.
      *@see Body::Body(MultiBody * _owner)
      */
    FixedBody(MultiBody * _owner);
    /**Constructor. Set owner and geometric information of this instance of FixedBody.
      *@see Body::Body(MultiBody * , GMSPolyhedron & )
      */
    FixedBody(MultiBody * _owner, GMSPolyhedron & _polyhedron);

    ///Do nothing
    virtual ~FixedBody();

    //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Access Methods. Use these method to acess or change internal state*/
    //@{

    ///YES!! This is a FIXED Body.
    virtual int IsFixedBody();

    /**This function returns a Polyhedron whose vertices and normals are represented in 
      *world coordinate.
      *@see Body::GetWorldPolyhedron
      */
    virtual GMSPolyhedron & GetWorldPolyhedron();

    ///Return transformation of this body in world coordinate
    virtual Transformation & GetWorldTransformation();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods. Use these method to read in/write out internal state*/
    
    //@{
    /**Write string "FixedBody", and call Body and world transformation's output method.
      *@see Body::Write, Transformation::Write
      */
    virtual void Write(ostream & _os);
    //@}

  //bool operator==(const FixedBody& b) const;

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
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  Implementation of FixedBody
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//===============================================================
// Inline Functions
//===============================================================

//---------------------------------------------------------------
// IsFixedBody
//---------------------------------------------------------------
inline int FixedBody::IsFixedBody() {
    return 1;
}

#endif

