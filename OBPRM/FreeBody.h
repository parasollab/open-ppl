// $Id$
/////////////////////////////////////////////////////////////////////
//  FreeBody.h
//
//  Created   2/25/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#ifndef FreeBody_h
#define FreeBody_h

#include "Body.h"
#include "Vectors.h"


class FreeBody : public Body {
public:
    //-----------------------------------------------------------
    //  Constructors and Destructor
    //-----------------------------------------------------------
    FreeBody(MultiBody * _owner);
    FreeBody(MultiBody * _owner, GMSPolyhedron & _polyhedron);
    ~FreeBody();
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
    void Configure(Transformation & _transformation);
protected:
private:
    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------
    double Mass;
    double jointLimit;
};

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
