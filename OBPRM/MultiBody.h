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
#include "AttFixedBody.h"
#include "AttFreeBody.h"
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
    AttFreeBody * GetAttFreeBody(int _index);
    AttFixedBody * GetAttFixedBody(int _index);
    Body * GetBody(int _index); // new
    int GetBodyCount(); // new
    int GetAttFreeBodyCount();
    int GetAttFixedBodyCount();
    int GetAttFreeBodyIndex(AttFreeBody * _b);
    int GetAttFixedBodyIndex(AttFixedBody * _b);
    void Draw();
    Body * GetFirstBody();
    void AddBody(AttFreeBody * _body);
    void AddBody(AttFixedBody * _body);
    double ComputeDistance(Body * _body1, Body * _body2);
    int GetNumberOfLinks();
    void Get(Input * _input, int _index);
    void Write(ostream & _os);

    int IsManipulator();

    void ConfigureJoint(double * _s, int _dof);

    void ComputeCenterOfMass();
    Vector3D GetCenterOfMass();

    void FindBoundingBox();
    double * GetBoundingBox();
    double GetMaxAxisRange();

    void ComputeCOM();
    Vector3D GetCOM();          // Get original center of mass

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
    int attFixedBodyCount;
    AttFixedBody ** attFixedBody;
    int attFreeBodyCount;
    AttFreeBody ** attFreeBody;
    //Equation motionEquation;

    Vector3D CenterOfMass;
    bool CenterOfMassAvailable;

    double boundingBox[6];
    double maxAxisRange;
    Vector3D com;       	// Original center of mass


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
//  GetAttFixedBodyCount
//-------------------------------------------------------------------
inline int MultiBody::GetAttFixedBodyCount() {
    return attFixedBodyCount;
}

//-------------------------------------------------------------------
//  GetAttFreeBodyCount
//-------------------------------------------------------------------
inline int MultiBody::GetAttFreeBodyCount() {
    return attFreeBodyCount;
}

//-------------------------------------------------------------------
//  GetBodyCount
//  02/23/99 Guang Song
//-------------------------------------------------------------------
inline int MultiBody::GetBodyCount() {
    return attFreeBodyCount+attFixedBodyCount;
}
//-------------------------------------------------------------------
//  GetBody
//  02/23/99 Guang Song
//-------------------------------------------------------------------
inline Body * MultiBody::GetBody(int _index) {
    if(_index < 0 || _index >= attFreeBodyCount+attFixedBodyCount) {
        cout << "Error in MultiBody::GetBody !!" << endl;
        exit(-1);
    } else
    if (_index < attFixedBodyCount) {
        return attFixedBody[_index];
    } else {
        return attFreeBody[_index-attFixedBodyCount];
    }
}

//-------------------------------------------------------------------
//  GetAttFixedBody
//-------------------------------------------------------------------
inline AttFixedBody * MultiBody::GetAttFixedBody(int _index) {
    if (_index < attFixedBodyCount)
        return attFixedBody[_index];
    else
        return 0;
}

//-------------------------------------------------------------------
//  GetAttFreeBody
//-------------------------------------------------------------------
inline AttFreeBody * MultiBody::GetAttFreeBody(int _index) {
    if (_index < attFreeBodyCount)
        return attFreeBody[_index];
    else 
        return 0;
}

//-------------------------------------------------------------------
//  GetAttFreeBodyIndex
//-------------------------------------------------------------------
inline int MultiBody::GetAttFreeBodyIndex(AttFreeBody * _b) {
    for (int i=0; i < attFreeBodyCount; i++)
        if (_b == GetAttFreeBody(i))
	    return i;
    // error
    return -1;
}

//-------------------------------------------------------------------
//  GetAttFixedBodyIndex
//-------------------------------------------------------------------
inline int MultiBody::GetAttFixedBodyIndex(AttFixedBody * _b) {
    for (int i=0; i < attFixedBodyCount; i++)
        if (_b == GetAttFixedBody(i))
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
//
//  Added  5/21/98  Wookho Son
//-------------------------------------------------------------------
inline int MultiBody::IsManipulator() {
    return (attFreeBodyCount > 0) ? 1 : 0;
}


#endif
