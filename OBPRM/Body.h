// $Id$
/////////////////////////////////////////////////////////////////////
//  Body.h
//
//  Created   2/25/98 Aaron Michalk
//  Modified  4/15/98 Aaron Michalk
//  Added     4/16/98 Wookho Son
//  Added     5/14/98 Wookho Son
//  Added     6/4/98 Wookho Son
//  Modified  7/31/98 Wookho Son
/////////////////////////////////////////////////////////////////////

#ifndef Body_h
#define Body_h

#include <fstream.h>
#include "SwitchDefines.h"
#include "BasicDefns.h"
#include "Transformation.h"
#include "DHparameters.h"
#include "GMSPolyhedron.h"
#include "Connection.h"
#include "Vectors.h"

#include "util.h"
#ifdef USE_CSTK
#include <cstkSmallAPI.h>
#endif
#ifdef USE_VCLIP
#include <vclip.h>
#endif
#ifdef USE_RAPID
#include <RAPID.H>
#endif

class MultiBody;
class Contact;

class Body {
public:
    //===============================================================
    //  Constructors and Destructor
    //===============================================================
    Body(MultiBody * _owner);
    Body(MultiBody * _owner, GMSPolyhedron & _polyhedron);
    ~Body();
    //===============================================================
    //  Pure Virtual Methods
    //  pure virtual fuction can never be called directly
    //===============================================================
    virtual int IsFixedBody() = 0;
    virtual Transformation & GetWorldTransformation() = 0;
    //===============================================================
    //  Virtual Methods
    //===============================================================
    virtual GMSPolyhedron & GetWorldPolyhedron();
    virtual void ChangeWorldPolyhedron();
    virtual GMSPolyhedron & GetPolyhedron();
    virtual void Write(ostream & _os);


    //===============================================================
    //  Methods
    //===============================================================
    void Read(char * _fileName);
    void Read(Input*, char * _fileName);

    void ReadBYU(cd_predefined cdtype, istream & _is);
    void buildCDstructure(cd_predefined cdtype, int nprocs = 1);

    MultiBody * GetMultiBody();
    void AddContactCount();
    void InitializeContact();
    void PutWorldTransformation(Transformation & _worldTransformation);

    void FindBoundingBox();
    double * GetBoundingBox();

#ifdef USE_CSTK
    void * GetCstkBody();
    void * GetCstkBody(int);
    void * From_GMS_to_CSTK();  // from GMS format to CSTK format.
    void ** From_GMS_to_cstk();
#endif
#ifdef USE_VCLIP
    PolyTree * GetVclipBody();
#endif
#ifdef USE_RAPID
    RAPID_model * GetRapidBody();
#endif

    // if GetCenterOfMass() is called for the first time,
    // this function calculates it and set 'available' flag
    void ComputeCenterOfMass();  

    // to get center of mass, you don't need to additionally call
    // the above: ComputeCenterOfMass()
    Vector3D GetCenterOfMass();


    //facilitate robot self collision checking.
    bool isAdjacent(Body *);
    Transformation & WorldTransformation();
    //---------------------------------------------------------------
    //  Connection methods
    //---------------------------------------------------------------
    int ForwardConnectionCount();
    int BackwardConnectionCount();
    Connection * GetForwardConnection(int _index);
    Connection * GetBackwardConnection(int _index);
    void AddForwardConnection(Connection * _connection);
    void AddBackwardConnection(Connection * _connection);
    void RemoveForwardConnection(Connection * _connection, int _delete);
    void RemoveBackwardConnection(Connection * _connection, int _delete);
    void Link(Body * _otherBody, const Transformation & _transformationToBody2,
     const DHparameters & _dhparameters, const Transformation &_transformationToDHFrame);
    void Link(Connection * _c);
protected:
    //===============================================================
    //  Data
    //===============================================================
    MultiBody * multibody;
    Transformation worldTransformation;
    GMSPolyhedron polyhedron;
    GMSPolyhedron worldPolyhedron;
    char polyhedronFileName[32];
    int forwardConnectionCount;
    Connection ** forwardConnection;
    int backwardConnectionCount;
    Connection ** backwardConnection;
    int contactCount;
    //int IsConvex;
    //Material material;
    //Uncertainty uncertainty;

    bool CenterOfMassAvailable;
    Vector3D CenterOfMass;

    double boundingBox[6];

#ifdef USE_CSTK
    void * cstkBody[MAXPROCS];
#endif
#ifdef USE_VCLIP
    PolyTree *vclipBody;
#endif
#ifdef USE_RAPID
    RAPID_model *rapidBody;
#endif

private:
friend class MultiBody;
friend class Connection;
};

//===================================================================
//  Inline functions
//===================================================================

//-------------------------------------------------------------------
// isAdjacent
// to check if two Body share same joint(adjacent) for a robot.
//-------------------------------------------------------------------
inline bool Body::isAdjacent(Body * otherBody) {
    for(int i=0; i < forwardConnectionCount; i++)
        if(forwardConnection[i]->GetNextBody() == otherBody)
            return true;
    for(int j=0; j < backwardConnectionCount; j++)
        if(backwardConnection[j]->GetPreviousBody() == otherBody)
            return true;

    return (this == otherBody); // if the two are the same, return true too.
}

//-------------------------------------------------------------------
// WorldTransformation
//
// If worldTransformation has been calculated(updated), this method should be used
// to avoid redundant calculation.
//-------------------------------------------------------------------
inline Transformation & Body::WorldTransformation() {
    return worldTransformation;
}


//-------------------------------------------------------------------
//  GetBoundingBox
//-------------------------------------------------------------------
inline double * Body::GetBoundingBox(){
    return boundingBox;
}

//-------------------------------------------------------------------
//  GetCenterOfMass
//-------------------------------------------------------------------
inline Vector3D Body::GetCenterOfMass(){
    if (!CenterOfMassAvailable) {
        ComputeCenterOfMass();
    }
    return CenterOfMass;
}

//-------------------------------------------------------------------
//  GetMultiBody
//-------------------------------------------------------------------
inline MultiBody * Body::GetMultiBody() {
    return multibody;
}

//-------------------------------------------------------------------
//  ContactCount
//-------------------------------------------------------------------
//inline int Body::GetContactCount() {
//    return contactCount;
//}

//-------------------------------------------------------------------
//  ForwardConnectionCount
//-------------------------------------------------------------------
inline int Body::ForwardConnectionCount() {
    return forwardConnectionCount;
}

//-------------------------------------------------------------------
//  BackwardConnectionCount
//-------------------------------------------------------------------
inline int Body::BackwardConnectionCount() {
    return backwardConnectionCount;
}

//-------------------------------------------------------------------
//  GetContact
//-------------------------------------------------------------------
//inline Contact * Body::GetContact(int _index) {
//    if (_index < contactCount)
//        return contact[_index];
//    else
//        return 0;
//}

//-------------------------------------------------------------------
//  GetForwardConnection
//-------------------------------------------------------------------
inline Connection * Body::GetForwardConnection(int _index) {
    if (_index < forwardConnectionCount)
        return forwardConnection[_index];
    else
        return 0;
}

//-------------------------------------------------------------------
//  GetBackwardConnection
//-------------------------------------------------------------------
inline Connection * Body::GetBackwardConnection(int _index) {
    if (_index < backwardConnectionCount)
        return backwardConnection[_index];
    else
        return 0;
}

//-------------------------------------------------------------------
//  AddContactCount
//  Function: Increment the contact count
//-------------------------------------------------------------------
inline void Body::AddContactCount() {
  contactCount++;
}

//-------------------------------------------------------------------
//  PutWorldTransformation
//
//  Function: Assign the given transformation as a transformation
//            w.r.t the world for "this" body
//-------------------------------------------------------------------
inline void Body::PutWorldTransformation(Transformation & _worldTransformation){
  worldTransformation = _worldTransformation;
}

#endif
