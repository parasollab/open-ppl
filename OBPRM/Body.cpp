// $Id$
/////////////////////////////////////////////////////////////////////
//  Body.c
//
//  Created   3/ 1/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#include "Body.h"
#include "Input.h"

#include <string.h>
#include <stdlib.h>

//===================================================================
//  Constructors and Destructor
//===================================================================
Body::Body(MultiBody * _owner) {
    multibody = _owner;
    contactCount = 0;
    forwardConnectionCount = 0;
    backwardConnectionCount = 0;
    forwardConnection = 0;
    backwardConnection = 0;
    CenterOfMassAvailable = false;
}

Body::Body(MultiBody * _owner, GMSPolyhedron & _polyhedron) {
    multibody = _owner;
    polyhedron = _polyhedron;
    contactCount = 0;
    forwardConnectionCount = 0;
    backwardConnectionCount = 0;
    forwardConnection = 0;
    backwardConnection = 0;
    CenterOfMassAvailable = false;
}

Body::~Body() {
    int i;
    for (i=0; i < forwardConnectionCount; i++) {
        delete forwardConnection[i];
    }
    for (i=0; i < backwardConnectionCount; i++) {
        delete backwardConnection[i];
    }
}

//===================================================================
//  GetWorldPolyhedron
//
//  Function: Transform the vertices and normals of the polyhedron
//            w.r.t. the world frame
//
//  Output: The polyhedron transformed w.r.t the world frame
//
//===================================================================
GMSPolyhedron & Body::GetWorldPolyhedron() {

    int i;
    for (i=0; i < polyhedron.numVertices; i++)  // Transform the vertices
        worldPolyhedron.vertexList[i] = worldTransformation * polyhedron.vertexList[i];
    for (i=0; i < polyhedron.numPolygons; i++)  // Transform the normals
        worldPolyhedron.polygonList[i].normal = worldTransformation.orientation * polyhedron.polygonList[i].normal;

    return worldPolyhedron;
}

//===================================================================
//  ChangeWorldPolyhedron
//
//=================================================================
void Body::ChangeWorldPolyhedron() {

    int i;
    for (i=0; i < polyhedron.numVertices; i++)  // Transform the vertices
        worldPolyhedron.vertexList[i] = worldTransformation * polyhedron.vertexList[i];
    for (i=0; i < polyhedron.numPolygons; i++)  // Transform the normals
        worldPolyhedron.polygonList[i].normal = worldTransformation.orientation * polyhedron.polygonList[i].normal;
}


//=============================================================
// GetPolyhedron
//
// Function: return the local coordinates of the body.
//
//=============================================================
GMSPolyhedron & Body::GetPolyhedron() {
    return polyhedron;
}


//===================================================================
//  Link
//  Function: Link "this" body to the given other body by setting up
//            a connectionship between them using the given DH
//            parameters and the transformation (from the distal
//	      joint of "this" body to the center of gravity of the other).
//
//===================================================================
void Body::Link(Body * _otherBody, const Transformation & _transformationToBody2, const
DHparameters &_dhparameters, const Transformation & _transformationToDHFrame) {
    Connection * c = new Connection(this, _otherBody, _transformationToBody2,
                                    _dhparameters, _transformationToDHFrame);
    Link(c);
}

//==================================================================
//  Link
//  Function: Link "this" body to the given other body by using the
//            given connection.
//            Establish a forward and backward connecntionship
//==================================================================
void Body::Link(Connection * _c) {
    AddForwardConnection(_c);
    // Establish a backward connectionship for the next body
    _c->GetNextBody()->AddBackwardConnection(_c);
}

//===================================================================
//  AddForwardConnection
//  Function: Set up a forward connectionship with the given connection
//===================================================================
void Body::AddForwardConnection(Connection * _connection) {
    forwardConnectionCount++;
    forwardConnection = (Connection **)realloc(forwardConnection, forwardConnectionCount * sizeof(Connection *));
    forwardConnection[forwardConnectionCount-1] = _connection;
}

//===================================================================
//  AddBackwardConnection
//  Function: Set up a backward connectionship with the given connection
//===================================================================
void Body::AddBackwardConnection(Connection * _connection) {
    backwardConnectionCount++;
    backwardConnection = (Connection **)realloc(backwardConnection, backwardConnectionCount * sizeof(Connection *));
    backwardConnection[backwardConnectionCount-1] = _connection;
}

//===================================================================
//  RemoveForwardConnection
//===================================================================
void Body::RemoveForwardConnection(Connection * _connection, int _delete) {
    int i, j;
    for (i=0; i < forwardConnectionCount; i++) {
        if (forwardConnection[i] == _connection)
	    break;
    }
    if (i < forwardConnectionCount) {
        if (_delete)
            delete forwardConnection[i];
        if (i < forwardConnectionCount) {
            forwardConnectionCount--;
            for (j = i; j < forwardConnectionCount; j++)
	        forwardConnection[j] = forwardConnection[j+1];
	    forwardConnection = (Connection **)realloc(forwardConnection, forwardConnectionCount * sizeof(Connection *));
        }
    } else {
        // Error
    }
}

//===================================================================
//  RemoveBackwardConnection
//===================================================================
void Body::RemoveBackwardConnection(Connection * _connection, int _delete) {
    int i, j;
    for (i=0; i < backwardConnectionCount; i++) {
        if (backwardConnection[i] == _connection)
	    break;
    }
    if (i < backwardConnectionCount) {
        if ( _delete )
            delete backwardConnection[i];
        if (i < backwardConnectionCount) {
            backwardConnectionCount--;
            for (j = i; j < backwardConnectionCount; j++)
	        backwardConnection[j] = backwardConnection[j+1];
	    backwardConnection = (Connection **)realloc(backwardConnection, backwardConnectionCount * sizeof(Connection *));
        }
    } else {
        // Error
    }
}

//===================================================================
//  Read
//===================================================================
void Body::ReadBYU(cd_predefined cdtype, istream & _is) {
    //---------------------------------------------------------------
    // Read polyhedron
    //---------------------------------------------------------------
    polyhedron.ReadBYU(_is);
    worldPolyhedron = polyhedron;

    FindBoundingBox();

    //---------------------------------------------------------------
    // Now create auxilary data structure for collision detection
    //---------------------------------------------------------------
    buildCDstructure(cdtype);
}


void Body::Read(Input *input, char * _fileName) {

    //---------------------------------------------------------------
    // Read polyhedron
    //---------------------------------------------------------------
    Read(_fileName);


    //---------------------------------------------------------------
    // Now create auxilary data structure for collision detection
    //---------------------------------------------------------------
    buildCDstructure(input->cdtype, input->nprocs);
}

void Body::buildCDstructure(cd_predefined cdtype, int nprocs) {

#ifdef USE_VCLIP
    if (cdtype == VCLIP) {
	GMSPolyhedron poly = GetPolyhedron();
	Polyhedron *vpoly = new Polyhedron;
	for(int v = 0 ; v < poly.numVertices ; v++){
		vpoly->addVertex("",
			Vect3(poly.vertexList[v].getX(),
			      poly.vertexList[v].getY(),
			      poly.vertexList[v].getZ()
			));
	}

	vpoly->buildHull();
	vclipBody = new PolyTree;
	vclipBody->setPoly(vpoly);
    } else
#endif
#ifdef USE_CSTK
    if (cdtype == CSTK){

        for (int p=0; p<nprocs; ++p)
           cstkBody[p] = From_GMS_to_CSTK();
    } else
#endif
#ifdef USE_RAPID
      if (cdtype == RAPID){
	GMSPolyhedron poly = GetPolyhedron();

	rapidBody = new RAPID_model;
	rapidBody->BeginModel();
	for(int q=0; q < poly.numPolygons; q++) {
	    int vertexNum[3];
	    double point[3][3];
	    for(int i=0; i<3; i++) {
	       vertexNum[i] = poly.polygonList[q].vertexList[i];
	       Vector3D &tmp = poly.vertexList[vertexNum[i]];
	       for(int j=0; j<3; j++)
	           point[i][j] = tmp[j];
	    }
	    rapidBody->AddTri(point[0], point[1], point[2], q);
	}
	rapidBody->EndModel();

    } else
#endif
#ifdef USE_PQP
      if (cdtype == PQP){
	GMSPolyhedron poly = GetPolyhedron();

	pqpBody = new PQP_Model;
	pqpBody->BeginModel();
	for(int q=0; q < poly.numPolygons; q++) {
	    int vertexNum[3];
	    double point[3][3];
	    for(int i=0; i<3; i++) {
	       vertexNum[i] = poly.polygonList[q].vertexList[i];
	       Vector3D &tmp = poly.vertexList[vertexNum[i]];
	       for(int j=0; j<3; j++)
	           point[i][j] = tmp[j];
	    }
	    pqpBody->AddTri(point[0], point[1], point[2], q);
	}
	pqpBody->EndModel();

    } else
#endif
    {
#ifndef NO_CD_USE
	cout <<"\n\n\tERROR: all other cd type's undefined\n\n";
	cout <<"\n  you gave me <" << cdtype << ">";
#endif

#ifdef USE_VCLIP
	cout <<"\n\nbut VCLIP = " << VCLIP;
#endif
#ifdef USE_RAPID
	cout <<"\n\nbut RAPID = " << RAPID;
#endif
#ifdef USE_CSTK
	cout <<"\n\nbut CSTK  = " << CSTK ;
#endif
#ifdef USE_PQP
	cout <<"\n\nbut RAPID = " << PQP;
#endif

#ifndef NO_CD_USE
	exit(-1);
#endif
    }
}

void Body::Read(char * _fileName) {

    //---------------------------------------------------------------
    // Read polyhedron
    //---------------------------------------------------------------
    polyhedron.Read(_fileName);
    worldPolyhedron = polyhedron;

    FindBoundingBox();
}


//===================================================================
//  Write
//===================================================================
void Body::Write(ostream & _os) {
    //---------------------------------------------------------------
    // Write polyhedron filename
    //---------------------------------------------------------------
    _os << polyhedronFileName << " ";
}


//===================================================================
//  InitializeContact
//===================================================================
void Body::InitializeContact() {
  // Reinitialize the contact count
  contactCount = 0;
}

//===================================================================
//  ComputeCenterOfMass
//  
//  This function is automatically caeed by GetCenterOfMass()
//  if it has never been computed. After computing it,
//  this function will not be called again: rigid body.
//
//  This way of computing center of mass is physically not true.
//  This assumes that each vertex carries the same mass, and
//  edges are weightless. To be more accurate, we need to
//  be modify this to consider the length of edges, which is
//  still an approximation.
//===================================================================
void Body::ComputeCenterOfMass(){
  GMSPolyhedron poly = GetWorldPolyhedron();
  if (poly.numVertices < 1) {
	cout << "\nERROR: No Vertices to take Body::CenterOfMass from...\n";
  }else{
  	Vector3D sum(0,0,0);
  	for (int i=0; i<poly.numVertices; i++) {
    		sum = sum + poly.vertexList[i];
  	}
  	CenterOfMass = sum/poly.numVertices;
  	CenterOfMassAvailable = true;
  }
}

//===================================================================
//  FindBoundingBox
//===================================================================
void Body::FindBoundingBox(){

    GMSPolyhedron poly;
    poly = GetWorldPolyhedron();

    double minx, miny, minz, maxx, maxy, maxz;
    minx = maxx = poly.vertexList[0].getX();
    miny = maxy = poly.vertexList[0].getY();
    minz = maxz = poly.vertexList[0].getZ();
    for(int i = 1 ; i < poly.numVertices ; i++){
        if(poly.vertexList[i].getX() < minx) minx = poly.vertexList[i].getX();
        else if(maxx < poly.vertexList[i].getX()) maxx = poly.vertexList[i].getX();

        if(poly.vertexList[i].getY() < miny) miny = poly.vertexList[i].getY();
        else if(maxy < poly.vertexList[i].getY()) maxy = poly.vertexList[i].getY();

        if(poly.vertexList[i].getZ() < minz) minz = poly.vertexList[i].getZ();
        else if(maxz < poly.vertexList[i].getZ()) maxz = poly.vertexList[i].getZ();
    }

    boundingBox[0] = minx; boundingBox[1] = maxx;
    boundingBox[2] = miny; boundingBox[3] = maxy;
    boundingBox[4] = minz; boundingBox[5] = maxz;
}


#ifdef USE_VCLIP
PolyTree * Body::GetVclipBody(){
    return vclipBody;
}
#endif

//-----------------------------------
#ifdef USE_RAPID
RAPID_model * Body::GetRapidBody() {
    return rapidBody;
}
#endif

#ifdef USE_PQP
PQP_Model * Body::GetPqpBody() {
    return pqpBody;
}
#endif

//===================================================================
//  GetCstkBody
//===================================================================
#ifdef USE_CSTK
void * Body::GetCstkBody(){
    return cstkBody[0];
}
void * Body::GetCstkBody(int proc_id){
    return cstkBody[proc_id];
}

#endif


#ifdef USE_CSTK

/////////////////////////////////////////////////////////////////////
//
// Convert GMS model to CSTK model.
//
/////////////////////////////////////////////////////////////////////
void** Body::From_GMS_to_cstk()
{
    GMSPolyhedron poly = GetWorldPolyhedron();

    int i, numVerts, numTris, vnum[3];
    void *tverts[3];
    void **verts, **tris;

    // Read in number of vertices
    numVerts = poly.numVertices;

    // Read in vertices
    verts = (void **) new char[sizeof(void *) * numVerts];
    for(i = 0 ; i < numVerts ; i++){
        cstkReal pt[3];

    	pt[0] = poly.vertexList[i].getX();
    	pt[1] = poly.vertexList[i].getY();
    	pt[2] = poly.vertexList[i].getZ();
    	verts[i] = cstkMake3DVertex(pt);
    }

    // Read in number of triangles
    numTris = poly.numPolygons;

    // Read in triangles
    tris = (void **) new char[sizeof(void *) * (numTris + 1)];
    for(i = 0 ; i < numTris ; i++){
    	vnum[0] = poly.polygonList[i].vertexList[0];
    	vnum[1] = poly.polygonList[i].vertexList[1];
    	vnum[2] = poly.polygonList[i].vertexList[2];

    	tverts[0] = verts[vnum[0]];
    	tverts[1] = verts[vnum[1]];
    	tverts[2] = verts[vnum[2]];
    	tris[i] = cstkMake3DPolygon(tverts, 3);
    }
    tris[numTris] = NULL;

    delete verts; // Just an array of (void *), so no destructors called.

    return tris;
}


/////////////////////////////////////////////////////////////////////
//
// Convert GMS body to CSTK body.
//
/////////////////////////////////////////////////////////////////////
void * Body::From_GMS_to_CSTK()
{
    void **tmpPolys = NULL;
    tmpPolys = From_GMS_to_cstk();

    int triCount;
    for (triCount = 0; tmpPolys[triCount]; triCount++);
    void *sub = cstkMakeBodyFromPolys(tmpPolys, triCount);
    delete tmpPolys;   // Just an array of (void *), so no destructors called.
    void *robot = cstkMakeLMovableBody(sub);

    return robot;
}

#endif
