/* Class FreeBody is a movable Body in workspace.
 * This class provide more specific methods for manipulate movable object.
 */

#ifndef FreeBody_h
#define FreeBody_h

#include "Body.h"
#include <set>

class FreeBody : public Body {
  public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    //Condtructor. Construct a FreeBody and set its owner as _owner.
    //a free body can be a smaller component of a group.  

    FreeBody(MultiBody* _owner);

    /**Constructor. Set owner and geometric information of this instance of FreeBody.
     *@see Body::Body(MultiBody * , GMSPolyhedron & )
     */
    FreeBody(MultiBody* _owner, GMSPolyhedron& _polyhedron);

    virtual ~FreeBody();

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Access Methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    //These following virtual functions can be overridden in inheriting classes
    //by a function with the same signature.

    //programer can use this to check if this object is free or fixed body.
    virtual int IsFixedBody(){return false;}

    ///Call Body::GetWorldPolyhedron 
    virtual GMSPolyhedron& GetWorldPolyhedron();

    /**Get world transforamtion of this free body.
     *Transformation "this" body w.r.t the world frame in a 
     *recursive manner; multiply the world transformation
     *of the previous body with the transformation from the
     *proximal joint to the center of gravity of "this" body
     *(Need a generalization for the connectionship, since
     *currently it handles only one backward connection).
     */
    virtual Transformation& GetWorldTransformation();


    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    I/O
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    friend istream& operator>>(istream& _is, FreeBody& _fb);
    friend ostream& operator<<(ostream& _os, FreeBody& _fb);

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Helpers
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Configure "this" body with the given transformation
    void Configure(Transformation& _transformation);

    Transformation& ComputeWorldTransformation(std::set<int, less<int> >& visited);

};

#endif
