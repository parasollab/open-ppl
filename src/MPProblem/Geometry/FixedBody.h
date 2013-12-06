//FixedBody.h
/* Class FreeBody is a stationary Body in workspace.
 * This class provide more specifice methods for manipulate fixed object. */

#ifndef FIXEDBODY_H_
#define FIXEDBODY_H_

#include "Body.h"

class FixedBody : public Body {
  public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    FixedBody(MultiBody* _owner, const string& _filename = "");
    FixedBody(MultiBody* _owner, GMSPolyhedron& _polyhedron);

    virtual ~FixedBody();

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Access Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    virtual int IsFixedBody(){return true;}

    /**This function returns a Polyhedron whose vertices and normals are represented in
     *world coordinate.
     */
    virtual GMSPolyhedron& GetWorldPolyhedron();

    ///Return transformation of this body in world coordinate
    virtual Transformation& GetWorldTransformation();

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    I/O
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    friend ostream& operator<<(ostream& _os, const FixedBody& _fb);
    friend istream& operator>>(istream& _is, FixedBody& _fb);
};

#endif
