// $Id$
/////////////////////////////////////////////////////////////////////
//  MultiBody.h
//
//  Created   2/25/98 Aaron Michalk
//  Modified  4/13/98 Aaron Michalk
//  Added     4/16/98 Wookho
//  Modified  7/14/98 Wookho
//  Modified  7/31/98 Wookho Son
/////////////////////////////////////////////////////////////////////

#ifndef MultiBody_h
#define MultiBody_h

#include <iomanip.h>
#include <fstream.h>
#include <list.h>
#include "Input.h"
#include "Body.h"
#include "Transformation.h"
#include "Matrix.h"
#include "FixedBody.h"
#include "FreeBody.h"
#include "Debug.h"

#ifndef VID
	#include "Graph.h"
#endif


typedef pair<VID,int> RANGE_TYPE;

class Environment;
class Contact;

class MultiBody {
public:
    //-----------------------------------------------------------
    //  Static Methods
    //-----------------------------------------------------------
    static void ComputePUMAInverseKinematics(Transformation & _t, double _a2, double _d3, double _a3, double _d4, double theta[8][6]);
    //-----------------------------------------------------------
    //  Constructors and Destructor
    //-----------------------------------------------------------
    MultiBody(Environment * _owner);
    ~MultiBody();
    //-----------------------------------------------------------
    //  Methods
    //-----------------------------------------------------------
    FreeBody * GetFreeBody(int _index);
    FixedBody * GetFixedBody(int _index);
    Body * GetBody(int _index); // new
    int GetBodyCount(); // new
    int GetFreeBodyCount();
    int GetFixedBodyCount();
    int GetFreeBodyIndex(FreeBody * _b);
    int GetFixedBodyIndex(FixedBody * _b);
    Body * GetFirstBody();
    void AddBody(FreeBody * _body);
    void AddBody(FixedBody * _body);
    double ComputeDistance(Body * _body1, Body * _body2);
    int GetNumberOfLinks();
    void Get(Input * _input, int _index);
    void Write(ostream & _os);

    int IsManipulator();

    void ConfigureJoint(double * _s, int _dof);

    // These two functions work in the same way as those in Body.h
    // If GetCenterOfMass() is called for the first time, then
    // ComputeCenterOfMass() is called automatically, and the
    // computed value is stored in this class for the next time.

    void ComputeCenterOfMass();
    Vector3D GetCenterOfMass();

    void FindBoundingBox();
    double * GetBoundingBox();
    double GetMaxAxisRange();

    // the maximum size of this multibody
    double GetBoundingSphereRadius();

    // Area Methods
    int GetNumBodies();
    double GetFixArea();
    double GetFreeArea();
    double GetArea();
    vector<double> GetFixAreas();
    vector<double> GetFreeAreas();

protected:
private:
    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------
    Environment * environment;
    int FixedBodyCount;
    FixedBody ** fixedBody;
    int FreeBodyCount;
    FreeBody ** freeBody;
    //Equation motionEquation;

    Vector3D CenterOfMass;
    bool CenterOfMassAvailable;

    double boundingBox[6];
    double maxAxisRange;


    // Area Stuff
    int numBodies;              // Total number of Bodies
    double fixArea;             // Area of FixedBodies
    double freeArea;            // Area of FreeBodies
    double area;                // Total Area of Bodies
    vector<double> fixAreas;    // Vector of Areas of FixedBodies
    vector<double> freeAreas;   // Vector of Areas of FreeBodies

};

//===================================================================
//  Inline Functions
//===================================================================

//-------------------------------------------------------------------
//  GetCenterOfMass
//-------------------------------------------------------------------
inline Vector3D MultiBody::GetCenterOfMass(){
    if (!CenterOfMassAvailable) {
        ComputeCenterOfMass();
    }
    return CenterOfMass;
}

//-------------------------------------------------------------------
//  GetFixedBodyCount
//-------------------------------------------------------------------
inline int MultiBody::GetFixedBodyCount() {
    return FixedBodyCount;
}

//-------------------------------------------------------------------
//  GetFreeBodyCount
//-------------------------------------------------------------------
inline int MultiBody::GetFreeBodyCount() {
    return FreeBodyCount;
}

//-------------------------------------------------------------------
//  GetBodyCount
//-------------------------------------------------------------------
inline int MultiBody::GetBodyCount() {
    return FreeBodyCount+FixedBodyCount;
}
//-------------------------------------------------------------------
//  GetBody
//-------------------------------------------------------------------
inline Body * MultiBody::GetBody(int _index) {
    if(_index < 0 || _index >= FreeBodyCount+FixedBodyCount) {
        cout << "Error in MultiBody::GetBody !!" << endl;
        exit(-1);
    } else
    if (_index < FixedBodyCount) {
        return fixedBody[_index];
    } else {
        return freeBody[_index-FixedBodyCount];
    }
}

//-------------------------------------------------------------------
//  GetFixedBody
//-------------------------------------------------------------------
inline FixedBody * MultiBody::GetFixedBody(int _index) {
    if (_index < FixedBodyCount)
        return fixedBody[_index];
    else
        return 0;
}

//-------------------------------------------------------------------
//  GetFreeBody
//-------------------------------------------------------------------
inline FreeBody * MultiBody::GetFreeBody(int _index) {
    if (_index < FreeBodyCount)
        return freeBody[_index];
    else
        return 0;
}

//-------------------------------------------------------------------
//  GetFreeBodyIndex
//-------------------------------------------------------------------
inline int MultiBody::GetFreeBodyIndex(FreeBody * _b) {
    for (int i=0; i < FreeBodyCount; i++)
        if (_b == GetFreeBody(i))
	    return i;
    // error
    return -1;
}

//-------------------------------------------------------------------
//  GetFixedBodyIndex
//-------------------------------------------------------------------
inline int MultiBody::GetFixedBodyIndex(FixedBody * _b) {
    for (int i=0; i < FixedBodyCount; i++)
        if (_b == GetFixedBody(i))
	    return i;
    // error
    return -1;
}

//-------------------------------------------------------------------
//  IsManipulator
//  Function: Determine if the MultiBody at hand is a manipulator.
//            If there is no free body attached to it,
//            it is considered to be a manipulator
//
//  Output:   True/False
//-------------------------------------------------------------------
inline int MultiBody::IsManipulator() {
    return (FreeBodyCount > 0) ? 1 : 0;
}


#endif
