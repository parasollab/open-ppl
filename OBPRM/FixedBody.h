// $Id$

/**@file FixedBody.h
   @date 2/25/98
   @author Aaron Michalk
*/

#ifndef FixedBody_h
#define FixedBody_h

#include <fstream.h>
#include "Body.h"

//class Body;
class Input;
class MultiBody;

class FixedBody : public Body {
public:
    //-----------------------------------------------------------
    //  Constructors and Destructor
    //-----------------------------------------------------------
    FixedBody(MultiBody * _owner);
    FixedBody(MultiBody * _owner, GMSPolyhedron & _polyhedron);
    ~FixedBody();
    //-----------------------------------------------------------
    //  Virtual Methods
    //-----------------------------------------------------------
    virtual int IsFixedBody();
    virtual GMSPolyhedron & GetWorldPolyhedron();
    virtual Transformation & GetWorldTransformation();
    virtual void Write(ostream & _os);


    //-----------------------------------------------------------
    //  Methods
    //-----------------------------------------------------------
    void Get(Input * _input, int _multibodyIndex, int _bodyIndex);

protected:
    int worldPolyhedronComputed;
private:
};

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

