#include "Body.h"
#include <sstream>

Body::Body(MultiBody* _owner) :
  m_multibody(_owner),
  m_isBase(false),
  m_baseType(Robot::PLANAR),
  m_baseMovementType(Robot::TRANSLATIONAL),
  m_centerOfMassAvailable(false), m_worldPolyhedronAvailable(false) {
    fill(m_boundingBox, m_boundingBox+6, 0);
  }

Body::Body(MultiBody* _owner, GMSPolyhedron& _polyhedron) :
  m_multibody(_owner),
  m_isBase(false),
  m_baseType(Robot::PLANAR),
  m_baseMovementType(Robot::TRANSLATIONAL),
  m_polyhedron(_polyhedron),
  m_worldPolyhedron(_polyhedron),
  m_centerOfMassAvailable(false),
  m_worldPolyhedronAvailable(false) {
    fill(m_boundingBox, m_boundingBox+6, 0);
  }

Body::Body(const Body& _b) :
  m_filename(_b.m_filename),
  m_multibody(_b.m_multibody),
  m_worldTransformation(_b.m_worldTransformation),
  m_isBase(_b.m_isBase),
  m_baseType(_b.m_baseType),
  m_baseMovementType(_b.m_baseMovementType),
  m_polyhedron(_b.m_polyhedron),
  m_worldPolyhedron(_b.m_worldPolyhedron),
  m_centerOfMassAvailable(_b.m_centerOfMassAvailable),
  m_centerOfMass(_b.m_centerOfMass),
  m_worldPolyhedronAvailable(_b.m_worldPolyhedronAvailable),
  m_bbPolyhedron(_b.m_bbPolyhedron),
  m_bbWorldPolyhedron(_b.m_bbWorldPolyhedron),
  m_forwardConnection(_b.m_forwardConnection),
  m_backwardConnection(_b.m_backwardConnection) {

  for(int i=0; i<6; ++i)
    m_boundingBox[i] = _b.m_boundingBox[i];

#ifdef USE_VCLIP
  vclipBody = _b.vclipBody;
#endif
#ifdef USE_RAPID
  rapidBody = _b.rapidBody;
#endif
#ifdef USE_PQP
  pqpBody = _b.pqpBody;
#endif
#ifdef USE_SOLID
  solidBody = _b.solidBody;
#endif
}

Body::~Body() {
   m_multibody=NULL;
}

bool
Body::operator==(const Body& _b) const {
  return (m_worldTransformation == _b.m_worldTransformation) &&
    (m_polyhedron == _b.m_polyhedron) &&
    (m_worldPolyhedron == _b.m_worldPolyhedron) &&
    (m_centerOfMass == _b.m_centerOfMass) &&
    (m_boundingBox[0] == _b.m_boundingBox[0]) &&
    (m_boundingBox[1] == _b.m_boundingBox[1]) &&
    (m_boundingBox[2] == _b.m_boundingBox[2]) &&
    (m_boundingBox[3] == _b.m_boundingBox[3]) &&
    (m_boundingBox[4] == _b.m_boundingBox[4]) &&
    (m_boundingBox[5] == _b.m_boundingBox[5]) &&
    (m_bbPolyhedron == _b.m_bbPolyhedron) &&
    (m_bbWorldPolyhedron == _b.m_bbWorldPolyhedron) &&
    (m_forwardConnection == _b.m_forwardConnection) &&
    (m_backwardConnection == _b.m_backwardConnection);
}

GMSPolyhedron&
Body::GetWorldPolyhedron() {
  if(!m_worldPolyhedronAvailable) {
    for(size_t i=0; i<m_polyhedron.m_vertexList.size(); ++i)
      m_worldPolyhedron.m_vertexList[i] = m_worldTransformation * m_polyhedron.m_vertexList[i];
    for(size_t i=0; i<m_polyhedron.m_polygonList.size(); ++i)
      m_worldPolyhedron.m_polygonList[i].m_normal = m_worldTransformation.rotation() * m_polyhedron.m_polygonList[i].m_normal;
    m_worldPolyhedronAvailable=true;
  }
  return m_worldPolyhedron;
}

GMSPolyhedron&
Body::GetWorldBoundingBox() {
  for(size_t i=0; i<m_bbPolyhedron.m_vertexList.size(); ++i)
    m_bbWorldPolyhedron.m_vertexList[i] = m_worldTransformation * m_bbPolyhedron.m_vertexList[i];
  return m_bbWorldPolyhedron;
}

Vector3d
Body::GetCenterOfMass(){
  if(!m_centerOfMassAvailable)
    ComputeCenterOfMass();
  return m_centerOfMass;
}

Connection&
Body::GetForwardConnection(size_t _index) {
  if (_index < m_forwardConnection.size())
    return m_forwardConnection[_index];
  else{
    cerr << "Error, in Body::GetForwardConnection: requesting connection outside of bounds\n\n";
    exit(-1);
  }
}

Connection&
Body::GetBackwardConnection(size_t _index) {
  if (_index < m_backwardConnection.size())
    return m_backwardConnection[_index];
  else{
    cerr << "Error, in Body::GetBackwardConnection: requesting connection outside of bounds\n\n";
    exit(-1);
  }
}

void
Body::ChangeWorldPolyhedron() {
  for(size_t i=0; i<m_polyhedron.m_vertexList.size(); i++)  // Transform the vertices
    m_worldPolyhedron.m_vertexList[i] = m_worldTransformation * m_polyhedron.m_vertexList[i];
  for(size_t i=0; i<m_polyhedron.m_polygonList.size(); i++)  // Transform the normals
    m_worldPolyhedron.m_polygonList[i].m_normal = m_worldTransformation.rotation() * m_polyhedron.m_polygonList[i].m_normal;
}

//===================================================================
//  Read
//===================================================================
void
Body::ReadBYU(istream& _is) {
  m_polyhedron.ReadBYU(_is);
  m_worldPolyhedron = m_polyhedron;
  FindBoundingBox();
}

void
Body::Read(string _fileName) {
  SetFileName(_fileName);

  m_polyhedron.Read(_fileName);
  m_worldPolyhedron = m_polyhedron;
  GMSPolyhedron poly;
  poly = GetPolyhedron();
  double minx, miny, minz, maxx, maxy, maxz;
  minx = maxx = poly.m_vertexList[0][0];
  miny = maxy = poly.m_vertexList[0][1];
  minz = maxz = poly.m_vertexList[0][2];
  for(size_t i = 1 ; i < poly.m_vertexList.size() ; i++){
    if(poly.m_vertexList[i][0] < minx)
      minx = poly.m_vertexList[i][0];
    else if(maxx < poly.m_vertexList[i][0])
      maxx = poly.m_vertexList[i][0];

    if(poly.m_vertexList[i][1] < miny)
      miny = poly.m_vertexList[i][1];
    else if(maxy < poly.m_vertexList[i][1])
      maxy = poly.m_vertexList[i][1];

    if(poly.m_vertexList[i][2] < minz)
      minz = poly.m_vertexList[i][2];
    else if(maxz < poly.m_vertexList[i][2])
      maxz = poly.m_vertexList[i][2];
  }

  m_bbPolyhedron.m_vertexList = vector<Vector3d>(8);
  m_bbWorldPolyhedron.m_vertexList = vector<Vector3d>(8);
  m_bbPolyhedron.m_vertexList[0] = Vector3d(minx,miny,minz);
  m_bbPolyhedron.m_vertexList[1] = Vector3d(minx,miny,maxz);
  m_bbPolyhedron.m_vertexList[2] = Vector3d(minx,maxy,minz);
  m_bbPolyhedron.m_vertexList[3] = Vector3d(minx,maxy,maxz);
  m_bbPolyhedron.m_vertexList[4] = Vector3d(maxx,miny,minz);
  m_bbPolyhedron.m_vertexList[5] = Vector3d(maxx,miny,maxz);
  m_bbPolyhedron.m_vertexList[6] = Vector3d(maxx,maxy,minz);
  m_bbPolyhedron.m_vertexList[7] = Vector3d(maxx,maxy,maxz);

  FindBoundingBox();
}

void
Body::Write(ostream& _os) {
  static int numBody = 0;
  ostringstream oss;
  oss << "Obj" << numBody++ << ".g";
  _os << oss.str() << " ";
  ofstream ofs(oss.str().c_str());
  m_polyhedron.WriteBYU(ofs);
  ofs.close();
}

void
Body::ComputeCenterOfMass(){
  GMSPolyhedron poly = GetWorldPolyhedron();
  if (poly.m_vertexList.empty()) {
    cout << "\nERROR: No Vertices to take Body::centerOfMass from...\n";
  }
  else{
    Vector3d sum(0,0,0);
    for (size_t i=0; i<poly.m_vertexList.size(); i++) {
      sum = sum + poly.m_vertexList[i];
    }
    m_centerOfMass = sum/poly.m_vertexList.size();
    m_centerOfMassAvailable = true;
  }
}

void
Body::FindBoundingBox(){
  GMSPolyhedron poly;
  m_worldPolyhedronAvailable = false;
  poly = GetWorldPolyhedron();
  double minx, miny, minz, maxx, maxy, maxz;
  minx = maxx = poly.m_vertexList[0][0];
  miny = maxy = poly.m_vertexList[0][1];
  minz = maxz = poly.m_vertexList[0][2];
  for(size_t i = 1 ; i < poly.m_vertexList.size() ; i++){
    if(poly.m_vertexList[i][0] < minx)
      minx = poly.m_vertexList[i][0];
    else if(maxx < poly.m_vertexList[i][0])
      maxx = poly.m_vertexList[i][0];

    if(poly.m_vertexList[i][1] < miny)
      miny = poly.m_vertexList[i][1];
    else if(maxy < poly.m_vertexList[i][1])
      maxy = poly.m_vertexList[i][1];

    if(poly.m_vertexList[i][2] < minz)
      minz = poly.m_vertexList[i][2];
    else if(maxz < poly.m_vertexList[i][2])
      maxz = poly.m_vertexList[i][2];
  }
  m_boundingBox[0] = minx; m_boundingBox[1] = maxx;
  m_boundingBox[2] = miny; m_boundingBox[3] = maxy;
  m_boundingBox[4] = minz; m_boundingBox[5] = maxz;
}

bool
Body::IsAdjacent(shared_ptr<Body> _otherBody) {
  for(vector<Connection>::iterator C = m_forwardConnection.begin(); C != m_forwardConnection.end(); ++C)
    if(C->GetNextBody() == _otherBody)
      return true;
  for(vector<Connection>::iterator C = m_backwardConnection.begin(); C != m_backwardConnection.end(); ++C)
    if(C->GetPreviousBody() == _otherBody)
      return true;
  return(*this == *(_otherBody.get()));
}

bool
Body::IsWithinI(shared_ptr<Body> _otherBody, int _i){
  if(*this == *(_otherBody.get()))
    return true;
  if(_i==0){
    return false;
  }
  return IsWithinIHelper(this,_otherBody.get(),_i,NULL);
}

bool
Body::IsWithinIHelper(Body* _body1, Body* _body2, int _i, Body* _prevBody){
  if(*_body1 == *_body2){
    return true;
  }
  if(_i==0){
    return false;
  }
  typedef vector<Connection>::iterator CIT;
  for(CIT C = _body1->m_forwardConnection.begin(); C != _body1->m_forwardConnection.end(); ++C)
    if(IsWithinIHelper(C->GetNextBody().get(), _body2, _i-1, _body1) )
      return true;
  for(CIT C =_body1->m_backwardConnection.begin(); C != _body1->m_backwardConnection.end(); ++C) {
    shared_ptr<Body> prev = C->GetPreviousBody();
    for(CIT C2 = prev->m_forwardConnection.begin(); C2 != prev->m_forwardConnection.end(); ++C2){
      if(*(C2->GetNextBody()) == *_body2)
        return true;
    }
    if(IsWithinIHelper(C->GetPreviousBody().get(),_body2,_i-1, _body1) )
      return true;
  }
  return false;
}

////////////////////////////////////////
// Collision Detection Methods
// /////////////////////////////////
void
Body::BuildCDStructure(cd_predefined _cdtype) {
#ifdef USE_VCLIP
  if (_cdtype == VCLIP) {
    GMSPolyhedron poly = GetPolyhedron();
    Polyhedron* vpoly = new Polyhedron;
    for(size_t v = 0 ; v < poly.m_vertexList.size() ; v++){
      vpoly->addVertex("",
          Vect3(poly.m_vertexList[v][0],
            poly.m_vertexList[v][1],
            poly.m_vertexList[v][2]
            ));
    }
    vpoly->buildHull();
    vclipBody = shared_ptr<PolyTree>(new PolyTree);
    vclipBody->setPoly(vpoly);
  }
  else
#endif
#ifdef USE_RAPID
    if (_cdtype == RAPID){
      GMSPolyhedron poly = GetPolyhedron();
      rapidBody = shared_ptr<RAPID_model>(new RAPID_model);
      rapidBody->BeginModel();
      for(size_t q=0; q < poly.m_polygonList.size(); q++) {
        int vertexNum[3];
        double point[3][3];
        for(int i=0; i<3; i++) {
          vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
          Vector3d &tmp = poly.m_vertexList[vertexNum[i]];
          for(int j=0; j<3; j++)
            point[i][j] = tmp[j];
        }
        rapidBody->AddTri(point[0], point[1], point[2], q);
      }
      rapidBody->EndModel();
    }
    else
#endif
#ifdef USE_PQP
      if (_cdtype == PROXIMITYQUERYPACKAGE){
        GMSPolyhedron poly = GetPolyhedron();
        pqpBody = shared_ptr<PQP_Model>(new PQP_Model);
        pqpBody->BeginModel();
        for(size_t q=0; q < poly.m_polygonList.size(); q++) {
          int vertexNum[3];
          double point[3][3];
          for(int i=0; i<3; i++) {
            vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
            Vector3d &tmp = poly.m_vertexList[vertexNum[i]];
            for(int j=0; j<3; j++)
              point[i][j] = tmp[j];
          }
          pqpBody->AddTri(point[0], point[1], point[2], q);
        }
        pqpBody->EndModel();
      }
      else
#endif
#ifdef USE_SOLID
        if (_cdtype == SOLID){
          GMSPolyhedron poly = GetWorldPolyhedron();
          vertex = new MT_Point3[3*poly.m_polygonList.size()];
          for(size_t q=0; q < poly.m_polygonList.size(); q++) {
            int vertexNum[3];
            float point[3][3];
            for(int i=0; i<3; i++) {
              vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
              Vector3d tmp = poly.m_vertexList[vertexNum[i]];
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
              Vector3d tmp = poly.m_vertexList[vertexNum[i]];
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
        }
        else
#endif
        {
#ifndef NO_CD_USE
          cerr <<"\n\n\tERROR: all other cd type's undefined\n\n";
          cerr <<"\n  you gave me <" << _cdtype << ">";
#endif

#ifdef USE_VCLIP
          cerr <<"\n\nbut VCLIP = " << VCLIP;
#endif
#ifdef USE_RAPID
          cerr <<"\n\nbut RAPID = " << RAPID;
#endif
#ifdef USE_PQP
          cerr <<"\n\nbut PQP = " << PROXIMITYQUERYPACKAGE;
#endif
#ifdef USE_SOLID
          cerr <<"\n\nbut SOLID = " << SOLID;
#endif
#ifndef NO_CD_USE
          exit(-1);
#endif
        }
}

#ifdef USE_SOLID
void
Body::UpdateVertexBase(){
  GMSPolyhedron poly = GetWorldPolyhedron();
  for(size_t q=0; q < poly.m_polygonList.size(); q++) {
    int vertexNum[3];
    for(int i=0; i<3; i++) {
      vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
      Vector3d &tmp = poly.m_vertexList[vertexNum[i]];
      vertex[3*q+i][0]=tmp[0];
      vertex[3*q+i][1]=tmp[1];
      vertex[3*q+i][2]=tmp[2];
    }
  }
  DT_ChangeVertexBase(base,vertex[0]);
}
#endif

///////////////////////////////////////////////////////////////////////////////
//  Connection Methods
///////////////////////////////////////////////////////////////////////////////
void
Body::Link(const shared_ptr<Body>& _otherBody, const Transformation & _transformationToBody2, const
    DHparameters &_dhparameters, const Transformation & _transformationToDHFrame) {
  Connection c(shared_ptr<Body>(this), _otherBody, _transformationToBody2,
      _dhparameters, _transformationToDHFrame);
  Link(c);
}

void
Body::Link(const Connection& _c) {
  AddForwardConnection(_c);
  _c.GetNextBody()->AddBackwardConnection(_c);
  m_worldPolyhedronAvailable=false;
  m_centerOfMassAvailable=false;
}

