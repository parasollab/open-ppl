/////////////////////////////////////////////////////////////////////
//  Body.cpp
/////////////////////////////////////////////////////////////////////

#include "Body.h"
#include <sstream>

//===================================================================
//  Constructors and Destructor
//===================================================================
Body::Body(MultiBody* _owner) :
  multibody(_owner), 
  CenterOfMassAvailable(false) 
{}

Body::Body(MultiBody* _owner, GMSPolyhedron & _polyhedron) :
  multibody(_owner), 
  polyhedron(_polyhedron), 
  worldPolyhedron(_polyhedron), 
  CenterOfMassAvailable(false) 
{}

Body::Body(const Body& b) :
  multibody(b.multibody),
  worldTransformation(b.worldTransformation),
  isBase(b.isBase),
  baseType(b.baseType),
  baseMovementType(b.baseMovementType),
  polyhedron(b.polyhedron),
  worldPolyhedron(b.worldPolyhedron),
  CenterOfMassAvailable(b.CenterOfMassAvailable),
  CenterOfMass(b.CenterOfMass),
  bb_polyhedron(b.bb_polyhedron),
  bb_world_polyhedron(b.bb_world_polyhedron),
  forwardConnection(b.forwardConnection),
  backwardConnection(b.backwardConnection)
{
  for(int i=0; i<6; ++i)
    boundingBox[i] = b.boundingBox[i];

#ifdef USE_VCLIP
    vclipBody = b.vclipBody;
#endif
#ifdef USE_RAPID
    rapidBody = b.rapidBody;
#endif
#ifdef USE_PQP
    pqpBody = b.pqpBody;
#endif
#ifdef USE_SOLID
    solidBody = b.solidBody;
#endif
}

Body::~Body() {
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
  for(size_t i=0; i<polyhedron.m_vertexList.size(); ++i)
    worldPolyhedron.m_vertexList[i] = worldTransformation * polyhedron.m_vertexList[i];
  for(size_t i=0; i<polyhedron.m_polygonList.size(); ++i)
    worldPolyhedron.m_polygonList[i].m_normal = worldTransformation.m_orientation * polyhedron.m_polygonList[i].m_normal;
  return worldPolyhedron;
}

//===================================================================
//  GetWorldBoundingBox
//
//  Function: Transform the vertices and normals of the BoundingBox
//            w.r.t. the world frame
//
//  Output: The polyhedron transformed w.r.t the world frame
//
//===================================================================
GMSPolyhedron & Body::GetWorldBoundingBox() {
  for(size_t i=0; i<bb_polyhedron.m_vertexList.size(); ++i) // Transform the vertices
      bb_world_polyhedron.m_vertexList[i] = worldTransformation * bb_polyhedron.m_vertexList[i];
  return bb_world_polyhedron;
}


//===================================================================
//  ChangeWorldPolyhedron
//
//=================================================================
void Body::ChangeWorldPolyhedron() {
  for(size_t i=0; i<polyhedron.m_vertexList.size(); i++)  // Transform the vertices
    worldPolyhedron.m_vertexList[i] = worldTransformation * polyhedron.m_vertexList[i];
  for(size_t i=0; i<polyhedron.m_polygonList.size(); i++)  // Transform the normals
    worldPolyhedron.m_polygonList[i].m_normal = worldTransformation.m_orientation * polyhedron.m_polygonList[i].m_normal;
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

GMSPolyhedron & Body::GetBoundingBoxPolyhedron() {
  return bb_polyhedron;
}


//===================================================================
//  Link
//  Function: Link "this" body to the given other body by setting up
//            a connectionship between them using the given DH
//            parameters and the transformation (from the distal
//	      joint of "this" body to the center of gravity of the other).
//
//===================================================================
void Body::Link(const shared_ptr<Body>& _otherBody, const Transformation & _transformationToBody2, const
DHparameters &_dhparameters, const Transformation & _transformationToDHFrame) {
    Connection c(shared_ptr<Body>(this), _otherBody, _transformationToBody2,
                 _dhparameters, _transformationToDHFrame);
    Link(c);
}

//==================================================================
//  Link
//  Function: Link "this" body to the given other body by using the
//            given connection.
//            Establish a forward and backward connecntionship
//==================================================================
void Body::Link(Connection _c) {
    AddForwardConnection(_c);
    // Establish a backward connectionship for the next body
    _c.GetNextBody()->AddBackwardConnection(_c);
}

//===================================================================
//  AddForwardConnection
//  Function: Set up a forward connectionship with the given connection
//===================================================================
void Body::AddForwardConnection(Connection _connection) {
  forwardConnection.push_back(_connection);
}

//===================================================================
//  AddBackwardConnection
//  Function: Set up a backward connectionship with the given connection
//===================================================================
void Body::AddBackwardConnection(Connection _connection) {
  backwardConnection.push_back(_connection);
}


//===================================================================
//  Read
//===================================================================
void Body::ReadBYU(istream & _is) {
    //---------------------------------------------------------------
    // Read polyhedron
    //---------------------------------------------------------------
    polyhedron.ReadBYU(_is);
    worldPolyhedron = polyhedron;

    FindBoundingBox();
}

#ifdef USE_VCLIP
#include "CfgTypes.h"
#endif
void Body::buildCDstructure(cd_predefined cdtype) {

#ifdef USE_VCLIP
  if (cdtype == VCLIP) {
    GMSPolyhedron poly = GetPolyhedron();
    Polyhedron* vpoly = new Polyhedron;

    if(CfgType().PosDOF()==2){
      for(size_t v = 0 ; v < poly.m_vertexList.size() ; v++){
        vpoly->addVertex("",
            Vect3(poly.m_vertexList[v].getX(),
              poly.m_vertexList[v].getY(),
              poly.m_vertexList[v].getZ()-0.001
              ));
      }
      for(size_t v = 0 ; v < poly.m_vertexList.size() ; v++){
        vpoly->addVertex("",
            Vect3(poly.m_vertexList[v].getX(),
              poly.m_vertexList[v].getY(),
              poly.m_vertexList[v].getZ()+0.001
              ));
      }
    }
    else{
      for(size_t v = 0 ; v < poly.m_vertexList.size() ; v++){
        vpoly->addVertex("",
            Vect3(poly.m_vertexList[v].getX(),
              poly.m_vertexList[v].getY(),
              poly.m_vertexList[v].getZ()
              ));
      }
    }

    vpoly->buildHull();
    vclipBody = shared_ptr<PolyTree>(new PolyTree);
    vclipBody->setPoly(vpoly);

  } else
#endif
#ifdef USE_RAPID
      if (cdtype == RAPID){
	GMSPolyhedron poly = GetPolyhedron();

	rapidBody = shared_ptr<RAPID_model>(new RAPID_model);
	rapidBody->BeginModel();
	for(size_t q=0; q < poly.m_polygonList.size(); q++) {
	    int vertexNum[3];
	    double point[3][3];
	    for(int i=0; i<3; i++) {
	       vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
	       Vector3D &tmp = poly.m_vertexList[vertexNum[i]];
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

	pqpBody = shared_ptr<PQP_Model>(new PQP_Model);
	pqpBody->BeginModel();
	for(size_t q=0; q < poly.m_polygonList.size(); q++) {
	    int vertexNum[3];
	    double point[3][3];
	    for(int i=0; i<3; i++) {
	       vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
	       Vector3D &tmp = poly.m_vertexList[vertexNum[i]];
	       for(int j=0; j<3; j++)
	           point[i][j] = tmp[j];
	    }
	    pqpBody->AddTri(point[0], point[1], point[2], q);
	}
	pqpBody->EndModel();

    } else
#endif
#ifdef USE_SOLID
      if (cdtype == SOLID){
 


	GMSPolyhedron poly = GetWorldPolyhedron();

	vertex = new MT_Point3[3*poly.m_polygonList.size()];
	
	for(size_t q=0; q < poly.m_polygonList.size(); q++) {
	    int vertexNum[3];
	    float point[3][3];
	    for(int i=0; i<3; i++) {
	       vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
	       Vector3D tmp = poly.m_vertexList[vertexNum[i]];
	       for(int j=0; j<3; j++)
	           vertex[3*q+i][j] = tmp[j];
	    }

	}


	base = DT_NewVertexBase(vertex[0],sizeof(vertex[0]));

        DT_ShapeHandle shape = DT_NewComplexShape(base);
	for(size_t q=0; q < poly.m_polygonList.size(); q++) {
	    int vertexNum[3];
	    float point[3][3];
	    for(int i=0; i<3; i++) {
	       vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
	       Vector3D tmp = poly.m_vertexList[vertexNum[i]];
	       for(int j=0; j<3; j++)
	           point[i][j] = tmp[j];
	    }
	    
	    DT_Begin();
	    DT_VertexIndex(3*q+0);
	    DT_VertexIndex(3*q+1);
	    DT_VertexIndex(3*q+2);
	    DT_End();

	}
	DT_EndComplexShape();

        DT_ObjectHandle object = DT_CreateObject(NULL,shape);

        solidBody = shared_ptr<DT_ObjectHandle>(new DT_ObjectHandle(object));



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
#ifdef USE_PQP
	cout <<"\n\nbut RAPID = " << PQP;
#endif
#ifdef USE_SOLID
	cout <<"\n\nbut SOLID = " << SOLID;
#endif
#ifndef NO_CD_USE
	exit(-1);
#endif
    }
}

#ifdef USE_SOLID
void Body::UpdateVertexBase(){
 



        GMSPolyhedron poly = GetWorldPolyhedron();


        for(size_t q=0; q < poly.m_polygonList.size(); q++) {
            int vertexNum[3];
            for(int i=0; i<3; i++) {
               vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
               Vector3D &tmp = poly.m_vertexList[vertexNum[i]];
                   vertex[3*q+i][0]=tmp[0];
                   vertex[3*q+i][1]=tmp[1];
                   vertex[3*q+i][2]=tmp[2];
            }
        }



        DT_ChangeVertexBase(base,vertex[0]);

}
#endif

void Body::Read(string _fileName) {
    //---------------------------------------------------------------
    // Read polyhedron
    //---------------------------------------------------------------
    polyhedron.Read(_fileName);
    worldPolyhedron = polyhedron;

    GMSPolyhedron poly;
    poly = GetPolyhedron();

    double minx, miny, minz, maxx, maxy, maxz;
    minx = maxx = poly.m_vertexList[0].getX();
    miny = maxy = poly.m_vertexList[0].getY();
    minz = maxz = poly.m_vertexList[0].getZ();
    for(size_t i = 1 ; i < poly.m_vertexList.size() ; i++){
        if(poly.m_vertexList[i].getX() < minx) minx = poly.m_vertexList[i].getX();
        else if(maxx < poly.m_vertexList[i].getX()) maxx = poly.m_vertexList[i].getX();

        if(poly.m_vertexList[i].getY() < miny) miny = poly.m_vertexList[i].getY();
        else if(maxy < poly.m_vertexList[i].getY()) maxy = poly.m_vertexList[i].getY();

        if(poly.m_vertexList[i].getZ() < minz) minz = poly.m_vertexList[i].getZ();
        else if(maxz < poly.m_vertexList[i].getZ()) maxz = poly.m_vertexList[i].getZ();
    }

//    boundingBox[0] = minx; boundingBox[1] = maxx;
//    boundingBox[2] = miny; boundingBox[3] = maxy;
//    boundingBox[4] = minz; boundingBox[5] = maxz;
      bb_polyhedron.m_vertexList = vector<Vector3D>(8);
      bb_world_polyhedron.m_vertexList = vector<Vector3D>(8);
      bb_polyhedron.m_vertexList[0] = Vector3D(minx,miny,minz);
      bb_polyhedron.m_vertexList[1] = Vector3D(minx,miny,maxz);
      bb_polyhedron.m_vertexList[2] = Vector3D(minx,maxy,minz);
      bb_polyhedron.m_vertexList[3] = Vector3D(minx,maxy,maxz);
      bb_polyhedron.m_vertexList[4] = Vector3D(maxx,miny,minz);
      bb_polyhedron.m_vertexList[5] = Vector3D(maxx,miny,maxz);
      bb_polyhedron.m_vertexList[6] = Vector3D(maxx,maxy,minz);
      bb_polyhedron.m_vertexList[7] = Vector3D(maxx,maxy,maxz);

///////////
    FindBoundingBox();
}

//===================================================================
//  Write
//===================================================================
void Body::Write(ostream & _os) {
  static int numBody = 0;
  ostringstream oss;
  oss<<"Obj"<<numBody++<<".g";
  _os<<oss.str()<<" ";
  ofstream ofs(oss.str().c_str());
  polyhedron.WriteBYU(ofs);
  ofs.close();
}


//===================================================================
//  ComputeCenterOfMass
//  
//  This function is automatically called by GetCenterOfMass()
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
  if (poly.m_vertexList.empty()) {
	cout << "\nERROR: No Vertices to take Body::CenterOfMass from...\n";
  }else{
  	Vector3D sum(0,0,0);
  	for (size_t i=0; i<poly.m_vertexList.size(); i++) {
    		sum = sum + poly.m_vertexList[i];
  	}
  	CenterOfMass = sum/poly.m_vertexList.size();
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
    minx = maxx = poly.m_vertexList[0].getX();
    miny = maxy = poly.m_vertexList[0].getY();
    minz = maxz = poly.m_vertexList[0].getZ();
    for(size_t i = 1 ; i < poly.m_vertexList.size() ; i++){
        if(poly.m_vertexList[i].getX() < minx) minx = poly.m_vertexList[i].getX();
        else if(maxx < poly.m_vertexList[i].getX()) maxx = poly.m_vertexList[i].getX();

        if(poly.m_vertexList[i].getY() < miny) miny = poly.m_vertexList[i].getY();
        else if(maxy < poly.m_vertexList[i].getY()) maxy = poly.m_vertexList[i].getY();

        if(poly.m_vertexList[i].getZ() < minz) minz = poly.m_vertexList[i].getZ();
        else if(maxz < poly.m_vertexList[i].getZ()) maxz = poly.m_vertexList[i].getZ();
    }

    boundingBox[0] = minx; boundingBox[1] = maxx;
    boundingBox[2] = miny; boundingBox[3] = maxy;
    boundingBox[4] = minz; boundingBox[5] = maxz;
}


#ifdef USE_VCLIP
shared_ptr<PolyTree> Body::GetVclipBody(){
    return vclipBody;
}
#endif

//-----------------------------------
#ifdef USE_RAPID
shared_ptr<RAPID_model> Body::GetRapidBody() {
    return rapidBody;
}
#endif

#ifdef USE_PQP
shared_ptr<PQP_Model> Body::GetPqpBody() {
    return pqpBody;
}
#endif

#ifdef USE_SOLID
shared_ptr<DT_ObjectHandle> Body::GetSolidBody() {
    return solidBody;
}
#endif


bool Body::operator==(const Body& b) const
{
  return (worldTransformation == b.worldTransformation) &&
         (polyhedron == b.polyhedron) &&
         (worldPolyhedron == b.worldPolyhedron) &&
         (CenterOfMassAvailable == b.CenterOfMassAvailable) &&
         (CenterOfMass == b.CenterOfMass) &&
         (boundingBox[0] == b.boundingBox[0]) &&
         (boundingBox[1] == b.boundingBox[1]) &&
         (boundingBox[2] == b.boundingBox[2]) &&
         (boundingBox[3] == b.boundingBox[3]) &&
         (boundingBox[4] == b.boundingBox[4]) &&
         (boundingBox[5] == b.boundingBox[5]) &&
         (bb_polyhedron == b.bb_polyhedron) &&
         (bb_world_polyhedron == b.bb_world_polyhedron) &&
         (forwardConnection == b.forwardConnection) &&
         (backwardConnection == b.backwardConnection);
}


/** isAdjacent:
 to check if two Body share same joint(adjacent) for a robot. */
bool Body::isAdjacent(shared_ptr<Body> otherBody) 
{
  for(vector<Connection>::iterator C = forwardConnection.begin(); C != forwardConnection.end(); ++C)
    if(C->GetNextBody() == otherBody)
      return true;
  for(vector<Connection>::iterator C = backwardConnection.begin(); C != backwardConnection.end(); ++C)
    if(C->GetPreviousBody() == otherBody)
      return true;
  return(*this == *(otherBody.get())); // if the two are the same, return true too.
}


bool Body::isWithinIHelper(Body* body1, Body* body2, int i, Body* prevBody){
  if(*body1 == *body2){
    return true;
  }
  if(i==0){
    return false;
  }
  typedef vector<Connection>::iterator CIT;
  for(CIT C = body1->forwardConnection.begin(); C != body1->forwardConnection.end(); ++C)
    //if(*(C->GetNextBody().get())==*prevBody)
      if(isWithinIHelper(C->GetNextBody().get(), body2, i-1, body1) )
        return true;
  for(CIT C =body1->backwardConnection.begin(); C != body1->backwardConnection.end(); ++C){
    for(CIT C2 = C->GetPreviousBody()->forwardConnection.begin(); C2!=C->GetPreviousBody()->forwardConnection.end(); C2++){
      if(*(C2->GetNextBody()) == *body2)
        return true;
    }
    //if(*(C->GetPreviousBody().get())==*prevBody)
      if(isWithinIHelper(C->GetPreviousBody().get(),body2,i-1, body1) )
        return true;
  }
  
  return false;
}

bool Body::isWithinI(shared_ptr<Body> otherBody, int i)
{
  
  if(*this == *(otherBody.get()))
    return true;
  if(i==0){
    return false;
  }
  return isWithinIHelper(this,otherBody.get(),i,NULL);
}


/** WorldTransformation
 If worldTransformation has been calculated(updated), this method should be used
 to avoid redundant calculation.
*/ 
Transformation & Body::WorldTransformation() 
{
  return worldTransformation;
}


///  GetBoundingBox
double * Body::GetBoundingBox()
{
  return boundingBox;
}


///  GetCenterOfMass
Vector3D Body::GetCenterOfMass()
{
  if(!CenterOfMassAvailable) 
    ComputeCenterOfMass();
  return CenterOfMass;
}


///  GetMultiBody
MultiBody* Body::GetMultiBody() 
{
  return multibody;
}


///  ForwardConnectionCount
int Body::ForwardConnectionCount() const 
{
  return forwardConnection.size();
}


///  BackwardConnectionCount
int Body::BackwardConnectionCount() const 
{
  return backwardConnection.size();
}


///  GetForwardConnection
Connection & Body::GetForwardConnection(size_t _index) 
{
  if (_index < forwardConnection.size())
    return forwardConnection[_index];
  else
  {
    cerr << "Error, in Body::GetForwardConnection: requesting connection outside of bounds\n\n";
    exit(-1);
  }
}


///  GetBackwardConnection
Connection & Body::GetBackwardConnection(size_t _index) 
{
  if (_index < backwardConnection.size())
    return backwardConnection[_index];
  else
  {
    cerr << "Error, in Body::GetBackwardConnection: requesting connection outside of bounds\n\n";
    exit(-1);
  }
}


/**
  PutWorldTransformation
  Function: Assign the given transformation as a transformation
            w.r.t the world for "this" body
*/
void Body::PutWorldTransformation(Transformation & _worldTransformation)
{
  worldTransformation = _worldTransformation;
}
