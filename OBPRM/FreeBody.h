// $Id$

/**@file FreeBody.h
  *@date 2/25/98
  *@author Aaron Michalk
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FreeBody_h
#define FreeBody_h

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "Body.h"

/* Class FreeBody is a movable Body in workspace.
 * This class provide more specifice methods for manipulate movable object.
 */
class FreeBody : public Body {
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

    /**Condtructor. Construct a FreeBody and set its owner as _owner.
      *@see Body::Body(MultiBody * _owner)
      */
    FreeBody(MultiBody * _owner);

    /**Constructor. Set owner and geometric information of this instance of FreeBody.
      *@see Body::Body(MultiBody * , GMSPolyhedron & )
      */
    FreeBody(MultiBody * _owner, GMSPolyhedron & _polyhedron);

    ///Do nothing
    ~FreeBody();

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

    ///NO!! This is a FREE Body.
    virtual int IsFixedBody();

    ///Call Body::GetWorldPolyhedron 
    virtual GMSPolyhedron & GetWorldPolyhedron();

    /**Get world transforamtion of this free body.
      *Transformation "this" body w.r.t the world frame in a 
      *recursive manner; multiply the world transformation
      *of the previous body with the transformation from the
      *proximal joint to the center of gravity of "this" body
      *(Need a generalization for the connectionship, since
      *currently it handles only one backward connection).
      */
    virtual Transformation & GetWorldTransformation();

    /**Get user input information from Input instance.
      *This method get geometric data filename from input.
      *Then it calls Read to read geometric data from filename.
      *@param _multibodyIndex index for the owner of this fixed body in input
      *@param _bodyIndex index for this fixed body in the owner in input
      *@see Read
      */
    virtual void Get(Input * _input, int _multibodyIndex, int _bodyIndex);

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

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helpers
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Help Methods*/
    //@{

    ///Configure "this" body with the given transformation
    void Configure(Transformation & _transformation);

    //@}

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
    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------
    double Mass;
    double jointLimit;
};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  Implementation of FreeBody
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
inline int FreeBody::IsFixedBody() {
    return 0;
}

#endif
