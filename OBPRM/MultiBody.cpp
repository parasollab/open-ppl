// $Id$
/////////////////////////////////////////////////////////////////////
//  MultiBody.c
//
//  Created   3/ 1/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#include <vector.h>
#include <stdlib.h>
#include <math.h>

/////////////////////////////////////////////////////////////////////////////////
// header files for C-space Toolkit
/////////////////////////////////////////////////////////////////////////////////
//#include <sys/time.h>
#ifdef HPUX
#include <sys/io.h>
#endif
#ifdef USE_CSTK
#include <three_d.h>
#include <wrappers.h>
#include <distance.h>
#include <witnesses.h>
#endif

//////////////////////////////////////////////////////////////////////////////////
#include "MultiBody.h"
#include "Input.h"

#define MAXCONTACT  10

//
// Global variable to be used for contact checking by C-Space Toolkit
//


//===================================================================
//  ComputePUMAInverseKinematics
//===================================================================
void MultiBody::ComputePUMAInverseKinematics(Transformation & _t, double _a2, double _d3, double _a3, double _d4, double theta[8][6]) {
    //---------------------------------------------------------------
    //  Compute theta1
    //---------------------------------------------------------------
    double root = sqrt(_t.position.getX()*_t.position.getX() + _t.position.getY()*_t.position.getY() - _d3*_d3);
    theta[0][0] = atan2(_t.position.getY(), _t.position.getX()) - atan2(_d3, root);
    theta[4][0] = atan2(_t.position.getY(), _t.position.getX()) - atan2(_d3, root);
    int i;
    for (i=1; i < 4; i++) {
        theta[i][0] = theta[0][0];
        theta[i+4][0] = theta[0][0];
    }
    //---------------------------------------------------------------
    //  Compute theta3
    //---------------------------------------------------------------
    double K = (_t.position.getX()*_t.position.getX() + _t.position.getY()*_t.position.getY() + _t.position.getZ()*_t.position.getZ() - _a2*_a2 - _a3*_a3 - _d3*_d3 - _d4*_d4)/(2*_a2);
    theta[0][2] = atan2(_a3, _d4) - atan2(K,  sqrt(_a3*_a3 + _d4*_d4 - K*K));
    theta[1][2] = theta[0][2];
    theta[2][2] = atan2(_a3, _d4) - atan2(K, -sqrt(_a3*_a3 + _d4*_d4 - K*K));
    theta[3][2] = theta[1][2];
    theta[4][2] = theta[0][2];
    theta[5][2] = theta[0][2];
    theta[6][2] = theta[1][2];
    theta[7][2] = theta[1][2];
    //---------------------------------------------------------------
    //  Compute theta2
    //---------------------------------------------------------------
    double s1, c1, s3, c3;
    for (i=0; i < 8; i += 2) {
        s1 = sin(theta[i][0]);
        c1 = cos(theta[i][0]);
        s3 = sin(theta[i][2]);
        c3 = cos(theta[i][2]);
        theta[i][1] = atan2((-_a3 - _a2*c3)*_t.position.getZ() - (c1*_t.position.getX() + s1*_t.position.getY())*(_d4 - _a2*s3),
                              (_a2*s3 - _d4)*_t.position.getZ() - (_a3 + _a2*c3)*(c1*_t.position.getX() + s1*_t.position.getY()))
                        - theta[i][2];
        theta[i+1][1] = theta[i][1];
    }
    //---------------------------------------------------------------
    //  Compute theta4
    //---------------------------------------------------------------
    double r13 = _t.orientation.matrix[1][3];
    double r23 = _t.orientation.matrix[2][3];
    double r33 = _t.orientation.matrix[3][3];
    double s23, c23;
    for (i=0; i < 8; i += 2) {
        s1 = sin(theta[i][0]);
        c1 = cos(theta[i][0]);
        s23 = sin(theta[i][1] + theta[i][2]);
        c23 = cos(theta[i][1] + theta[i][2]);
        theta[i][3] = atan2(-r13*s1 + r23*c1,
                            -(r13*c1 + r23*s1)*c23 + r33*s23);
        theta[i+1][3] = theta[i][3] + 180.0;
    }
    //---------------------------------------------------------------
    //  Compute theta5
    //---------------------------------------------------------------
    double s4, c4;
    for (i=0; i < 8; i += 2) {
        s1 = sin(theta[i][0]);
        c1 = cos(theta[i][0]);
        s23 = sin(theta[i][1] + theta[i][2]);
        c23 = cos(theta[i][1] + theta[i][2]);
        s4 = sin(theta[i][3]);
        c4 = cos(theta[i][3]);
        theta[i][4] = atan2(-r13*(c1*c23*c4 + s1*s4) - r23*(s1*c23*c4 - c1*s4) + r33*(s23*c4),
                            -(r13*c1 + r23*s1)*s23 - r33*c23);
        theta[i+1][4] = -theta[i][4];
    }
    //---------------------------------------------------------------
    //  Compute theta6
    //---------------------------------------------------------------
    double s5, c5;
    double r11 = _t.orientation.matrix[1][1];
    double r21 = _t.orientation.matrix[2][1];
    double r31 = _t.orientation.matrix[3][1];
    for (i=0; i < 8; i += 2) {
        s1 = sin(theta[i][0]);
        c1 = cos(theta[i][0]);
        s23 = sin(theta[i][1] + theta[i][2]);
        c23 = cos(theta[i][1] + theta[i][2]);
        s4 = sin(theta[i][3]);
        c4 = cos(theta[i][3]);
        s5 = sin(theta[i][4]);
        c5 = cos(theta[i][4]);
        theta[i][5] = atan2(-r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4 + c1*c4) + r31*s23*s4,
                            r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5 + c23*s5));
        theta[i+1][5] = theta[i][5] + 180.0;
    }
}

//===================================================================
//  Constructors and Destructor
//===================================================================
MultiBody::MultiBody(Environment * _owner) {
    environment = _owner;
    FreeBodyCount = 0;
    FixedBodyCount = 0;
    freeBody = 0;
    fixedBody = 0;
    CenterOfMassAvailable = false;
    bInternal = false;
}

MultiBody::~MultiBody() {
    int i;
    for (i=0; i < FreeBodyCount; i++)
	delete freeBody[i];
    for (i=0; i < FixedBodyCount; i++)
	delete fixedBody[i];
}

//===================================================================
//  ConfigureJoint
//
//  Function: Configure the joint by the given amount of displacement
//
//===================================================================
void MultiBody::ConfigureJoint(double * _s, int _dof) {
   int  i;

   for (i=0; i< _dof; i++)
	GetFreeBody(i)->GetForwardConnection(0)->GetDHparameters().theta += _s[i];
}

//===================================================================
//  GetFirstBody
//
//  Function: Get the very first body in a MultiBody.
//            It is a fixed base body, if the MultiBody is a manipualtor
//            Otherwise, it is the body itself.
//
//  Output: The pointer to the body
//
//===================================================================
Body * MultiBody::GetFirstBody() {
    //  I assume that the first body in the list is the anchor body.
    //  That is, all other bodies in the MultiBody are linked sequentially
    //  in a "forward" direction (with possible branches) from this anchor.
    if (FixedBodyCount > 0) {
	return fixedBody[0];
    } else if (FreeBodyCount > 0) {
        return freeBody[0];
    } else {
        return 0;
    }
}

//===================================================================
//  AddBody
//===================================================================
void MultiBody::AddBody(FixedBody * _body) {
    FixedBodyCount++;
    fixedBody = (FixedBody **)realloc(fixedBody, FixedBodyCount * sizeof(FixedBody *));
    fixedBody[FixedBodyCount-1] = _body;
}

void MultiBody::AddBody(FreeBody * _body) {
    FreeBodyCount++;
    freeBody = (FreeBody **)realloc(freeBody, FreeBodyCount * sizeof(FreeBody *));
    freeBody[FreeBodyCount-1] = _body;
}



//===================================================================
//  GetNumberOfLinks
//
//  Output: The number of links in "this" MultiBody
//===================================================================
int MultiBody::GetNumberOfLinks() {
    Body * b = GetFirstBody();

    if (!b)
        return 0;
    int i = 1;
    while (b->ForwardConnectionCount() > 0) {
	b = b->GetForwardConnection(0)->GetNextBody();
        i++;
    }

    return i-1;
}

//===================================================================
//  ComputeDistance
//===================================================================
double MultiBody::ComputeDistance(Body * _body1, Body * _body2) {
    return 0.0;
}

//===================================================================
//  Get
//===================================================================
void MultiBody::Get(Input * _input, int _multibodyIndex) {
    //---------------------------------------------------------------
    // Get bodies
    //---------------------------------------------------------------
	GetBodyInfoFromInput(_input,_multibodyIndex);

    //---------------------------------------------------------------
    // Get links
    //---------------------------------------------------------------
	GetLinkInfoFromInput(_input,_multibodyIndex);

    //---------------------------------------------------------------
	//calaulate help data
    //---------------------------------------------------------------
    FindBoundingBox();
    ComputeCenterOfMass();
}

//===================================================================
//  Get body info
//===================================================================
void MultiBody::GetBodyInfoFromInput(Input * _input, int _multibodyIndex)
{
	#if VERBOSE
    cout << "BodyCount = " << _input->BodyCount[_multibodyIndex] << endl;
    cout << "FixedBodyCount = " << _input->FixedBodyCount[_multibodyIndex] << endl;
#endif

    double fixSum = 0;
    double freeSum = 0;
    this->bInternal =  _input->bBodyInternal[_multibodyIndex];
    for(int i=0; i < _input->BodyCount[_multibodyIndex]; i++) 
	{
		if (!_input->isFree[_multibodyIndex][i])
		{
			FixedBody * fix = new FixedBody(this);
			fix->Get(_input, _multibodyIndex, i);
			fixAreas.push_back(fix->GetPolyhedron().area);
			fixSum += fix->GetPolyhedron().area;
			AddBody(fix);
		}
		else
		{
			FreeBody * free = new FreeBody(this);
			free->Get(_input, _multibodyIndex, i);
			freeAreas.push_back(free->GetPolyhedron().area);
			freeSum += free->GetPolyhedron().area;
			AddBody(free);
		}
    }
    fixArea = fixSum;
    freeArea = freeSum;
    area = fixArea + freeArea;

#if VERBOSE
    cout << "FreeBodyCount = " << _input->FreeBodyCount[_multibodyIndex] << endl;
#endif
}


//===================================================================
//  Get link info
//===================================================================
void MultiBody::GetLinkInfoFromInput(Input * _input, int _multibodyIndex)
{
    Connection * c;

    int bodyIndex0, realbodyIndex0;

#if VERBOSE
    cout << "connectionCount = " << _input->connectionCount[_multibodyIndex] << endl;
#endif

    for(int i=0; i < _input->connectionCount[_multibodyIndex]; i++) 
	{
		//Get connection info about first body in this connection
		//from Input instance
        bodyIndex0 = _input->previousBodyIndex[_multibodyIndex][i];

        realbodyIndex0 = _input->BodyIndex[_multibodyIndex][bodyIndex0];
        if (!_input->isFree[_multibodyIndex][bodyIndex0]){

			//Set first body in connection
			c = new Connection(GetFixedBody(realbodyIndex0));
			//set second body in connection using Get!!
			c->Get(_input, _multibodyIndex, i);
			
			// Set up both backward and forward connectionships
			GetFixedBody(realbodyIndex0)->Link(c);
		}
		else{
			c = new Connection(GetFreeBody(realbodyIndex0));
			c->Get(_input, _multibodyIndex, i);
			
			// Set up both backward and forward connectionships
			GetFreeBody(realbodyIndex0)->Link(c);
		}
    }
}

//===================================================================
//  Write
//===================================================================
void MultiBody::Write(ostream & _os) {
    //---------------------------------------------------------------
    // Write tag
    //---------------------------------------------------------------
    _os << "MultiBody" << endl;
    //---------------------------------------------------------------
    // Write bodies
    //---------------------------------------------------------------
    int i, j;
    _os << FixedBodyCount << endl;
    for (i=0; i < FixedBodyCount; i++)
        GetFixedBody(i)->Write(_os);
    _os << FreeBodyCount << endl;
    for (i=0; i < FreeBodyCount; i++)
        GetFreeBody(i)->Write(_os);
    //---------------------------------------------------------------
    // Write links
    //---------------------------------------------------------------
    for (i=0; i < FixedBodyCount; i++) {
        _os << GetFixedBody(i)->ForwardConnectionCount() << endl;
        for (j=0; j < GetFixedBody(i)->ForwardConnectionCount(); j++)
	    GetFixedBody(i)->GetForwardConnection(j)->Write(_os);
    }
    for (i=0; i < FreeBodyCount; i++) {
        _os << GetFreeBody(i)->ForwardConnectionCount() << endl;
        for (j=0; j < GetFreeBody(i)->ForwardConnectionCount(); j++)
	    GetFreeBody(i)->GetForwardConnection(j)->Write(_os);
    }
}

//===================================================================
//  ComputeCenterOfMass
//
//  The degree of approximation in calculating center of mass is
//  the same as in Body.cpp. To be more accurate, we need to
//  modify this function to consider the mass of each body.
// 
//===================================================================
void MultiBody::ComputeCenterOfMass(){

  int  nfree  = GetFreeBodyCount();
  int  nfixed = GetFixedBodyCount();

  if ((nfree+nfixed) == 0) {
        cout << "\nERROR: No MultiBodies to take MultiBody::CenterOfMass from...\n";
  }else{
  	Vector3D sum(0,0,0);
        int i;
  	for(i=0; i<nfree; i++){
		sum = sum +GetFreeBody(i)->GetCenterOfMass();
  	}
  	for(    i=0; i<nfixed; i++){
		sum = sum +GetFixedBody(i)->GetCenterOfMass();
  	}
  	CenterOfMass = sum/(nfree+nfixed);
  	CenterOfMassAvailable = true;
  }
}


//===================================================================
//  FindBoundingBox
//===================================================================
void MultiBody::FindBoundingBox(){
	
    int i, nfree, nfixed;
    double * tmp;
	
    nfree = GetFreeBodyCount();
    nfixed = GetFixedBodyCount();
	
	///////////////////////////////////////////////////////////
	//Check Free Bodys' Boudning Box
    double minx, miny, minz, maxx, maxy, maxz;
    i = 0;
    if(nfree > 0){
		this->GetFreeBody(i)->FindBoundingBox();
		tmp = this->GetFreeBody(i)->GetBoundingBox();
		minx = tmp[0]; maxx = tmp[1];
		miny = tmp[2]; maxy = tmp[3];
		minz = tmp[4]; maxz = tmp[5];
		
		for(i = 1 ; i < nfree ; i++){
			this->GetFreeBody(i)->FindBoundingBox();
			tmp = this->GetFreeBody(i)->GetBoundingBox();
			minx = min(minx,tmp[0]); maxx = max(maxx,tmp[1]);
			miny = min(miny,tmp[2]); maxy = max(maxy,tmp[3]);
			minz = min(minz,tmp[4]); maxz = max(maxz,tmp[5]);
		}
    }
	
	///////////////////////////////////////////////////////////
	//Check Fixed Bodys' Boudning Box
    i = 0;
    if(nfixed > 0){
		this->GetFixedBody(i)->FindBoundingBox();
		tmp = this->GetFixedBody(i)->GetBoundingBox();
		minx = tmp[0]; maxx = tmp[1];
        miny = tmp[2]; maxy = tmp[3];
        minz = tmp[4]; maxz = tmp[5];
		
		for(i = 1 ; i < nfixed ; i++){
			this->GetFixedBody(i)->FindBoundingBox();
			tmp = this->GetFixedBody(i)->GetBoundingBox();
            minx = min(minx,tmp[0]); maxx = max(maxx,tmp[1]);
            miny = min(miny,tmp[2]); maxy = max(maxy,tmp[3]);
            minz = min(minz,tmp[4]); maxz = max(maxz,tmp[5]);
        }
    }

	///////////////////////////////////////////////////////////
	//Pack
    boundingBox[0] = minx; boundingBox[1] = maxx;
    boundingBox[2] = miny; boundingBox[3] = maxy;
    boundingBox[4] = minz; boundingBox[5] = maxz;
	
	///////////////////////////////////////////////////////////
	//By the way

    // Find maxAxisRange
    double rangex, rangey, rangez;
    rangex = maxx - minx;
    rangey = maxy - miny;
    rangez = maxz - minz;
    maxAxisRange = max(rangex, max(rangey,rangez));
}


//===================================================================
//  GetBoundingBox
//===================================================================
double * MultiBody::GetBoundingBox(){
    return boundingBox;
}

//===================================================================
//  GetMaxAxisRange
//===================================================================
double MultiBody::GetMaxAxisRange(){
    return maxAxisRange;
}


//===================================================================
//	Area Methods
//===================================================================

//===================================================================
//  GetNumBodies
//===================================================================
int MultiBody::GetNumBodies(){
    return numBodies;
}


//===================================================================
//  GetFixArea
//===================================================================
double MultiBody::GetFixArea(){
    return fixArea;
}


//===================================================================
//  GetFreeArea
//===================================================================
double MultiBody::GetFreeArea(){
    return freeArea;
}


//===================================================================
//  GetArea
//===================================================================
double MultiBody::GetArea(){
    return area;
}


//===================================================================
//  GetFixAreas
//===================================================================
vector<double> MultiBody::GetFixAreas(){
    return fixAreas;
}


//===================================================================
//  GetFreeAreas
//===================================================================
vector<double> MultiBody::GetFreeAreas(){
    return freeAreas;
}


// the maximum size of this multibody
double MultiBody::GetBoundingSphereRadius() {
   double result = GetBody(0)->GetPolyhedron().maxRadius;
   for(int i=1; i<GetBodyCount(); i++)
	result += GetBody(i)->GetPolyhedron().maxRadius * 2.0;

   return result;
}


//the minimum size of the multibody
double MultiBody::GetInsideSphereRadius() {
  double result = GetBody(0)->GetPolyhedron().minRadius;
  for(int i=1; i<GetBodyCount(); i++)
    if (GetBody(i)->GetPolyhedron().minRadius > result)
      result = GetBody(i)->GetPolyhedron().minRadius;

  return result;
}

//===================================================================
// CalculateArea
//===================================================================
void MultiBody::CalculateArea(){
  double fixSum = 0;
  double freeSum = 0;

  for(int i=0; i<FixedBodyCount; i++) {
    fixAreas.push_back(fixedBody[i]->GetPolyhedron().area);
    fixSum += fixedBody[i]->GetPolyhedron().area;
  }

  for(int i=0; i<FreeBodyCount; i++) {
    freeAreas.push_back(freeBody[i]->GetPolyhedron().area);
    freeSum += freeBody[i]->GetPolyhedron().area;
  }

  fixArea = fixSum;
  freeArea = freeSum;
  area = fixArea + freeArea;
}
