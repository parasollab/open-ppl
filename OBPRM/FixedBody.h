// $Id$

/**@file FixedBody.h
   @date 2/25/98
   @author Aaron Michalk
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FixedBody_h
#define FixedBody_h

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//include standard headers
#include <fstream.h>

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "Body.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
class Input;
class MultiBody;
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Class FreeBody is a stationary Body in workspace.
 * This class provide more specifice methods for manipulate fixed object.
 */
class FixedBody : public Body {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
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
    ~FixedBody();

	//@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Access Methods
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

	/**Get user input information from Input instance.
	  *This method get translation info and geometric data filename from input.
	  *Then it calls Read to read geometric data from filename.
	  *@param _multibodyIndex index for the owner of this fixed body in input
	  *@param _bodyIndex index for this fixed body in the owner in input
	  *@see Read
	  */
    void Get(Input * _input, int _multibodyIndex, int _bodyIndex);
	//@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	I/O
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

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:
    int worldPolyhedronComputed;	///<True if Polyhedron in world coordinate system is available.

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Private data member and member methods
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
//	Implementation of FixedBody
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

