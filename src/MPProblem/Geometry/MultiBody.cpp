#include "MultiBody.h"

#include <numeric>

#include "Cfg/Cfg.h"
#include "MPProblem/Boundary.h"

void
MultiBody::
ComputePUMAInverseKinematics(Transformation & _t,
    double _a2, double _d3, double _a3, double _d4, double _theta[8][6]) {
  //---------------------------------------------------------------
  //  Compute theta1
  //---------------------------------------------------------------
  const Vector3d& pos = _t.translation();
  double root = sqrt(pos[0]*pos[0] + pos[1]*pos[1] - _d3*_d3);
  _theta[0][0] = atan2(pos[1], pos[0]) - atan2(_d3, root);
  _theta[4][0] = atan2(pos[1], pos[0]) - atan2(_d3, root);
  for(size_t i = 1; i < 4; i++) {
    _theta[i][0] = _theta[0][0];
    _theta[i+4][0] = _theta[0][0];
  }
  //---------------------------------------------------------------
  //  Compute theta3
  //---------------------------------------------------------------
  double K = (pos.normsqr() - _a2*_a2 - _a3*_a3 - _d3*_d3 - _d4*_d4)/(2*_a2);
  _theta[0][2] = atan2(_a3, _d4) - atan2(K,  sqrt(_a3*_a3 + _d4*_d4 - K*K));
  _theta[1][2] = _theta[0][2];
  _theta[2][2] = atan2(_a3, _d4) - atan2(K, -sqrt(_a3*_a3 + _d4*_d4 - K*K));
  _theta[3][2] = _theta[1][2];
  _theta[4][2] = _theta[0][2];
  _theta[5][2] = _theta[0][2];
  _theta[6][2] = _theta[1][2];
  _theta[7][2] = _theta[1][2];
  //---------------------------------------------------------------
  //  Compute theta2
  //---------------------------------------------------------------
  double s1, c1, s3, c3;
  for(size_t i = 0; i < 8; i += 2) {
    s1 = sin(_theta[i][0]);
    c1 = cos(_theta[i][0]);
    s3 = sin(_theta[i][2]);
    c3 = cos(_theta[i][2]);
    _theta[i][1] = atan2((-_a3 - _a2*c3)*pos[2] - (c1*pos[0] + s1*pos[1])*(_d4 - _a2*s3),
        (_a2*s3 - _d4)*pos[2] - (_a3 + _a2*c3)*(c1*pos[0] + s1*pos[1]))
      - _theta[i][2];
    _theta[i+1][1] = _theta[i][1];
  }
  //---------------------------------------------------------------
  //  Compute theta4
  //---------------------------------------------------------------
  double r13 = _t.rotation().matrix()[0][2];
  double r23 = _t.rotation().matrix()[1][2];
  double r33 = _t.rotation().matrix()[2][2];
  double s23, c23;
  for(size_t i = 0; i < 8; i += 2) {
    s1 = sin(_theta[i][0]);
    c1 = cos(_theta[i][0]);
    s23 = sin(_theta[i][1] + _theta[i][2]);
    c23 = cos(_theta[i][1] + _theta[i][2]);
    _theta[i][3] = atan2(-r13*s1 + r23*c1,
        -(r13*c1 + r23*s1)*c23 + r33*s23);
    _theta[i+1][3] = _theta[i][3] + 180.0;
  }
  //---------------------------------------------------------------
  //  Compute theta5
  //---------------------------------------------------------------
  double s4, c4;
  for(size_t i = 0; i < 8; i += 2) {
    s1 = sin(_theta[i][0]);
    c1 = cos(_theta[i][0]);
    s23 = sin(_theta[i][1] + _theta[i][2]);
    c23 = cos(_theta[i][1] + _theta[i][2]);
    s4 = sin(_theta[i][3]);
    c4 = cos(_theta[i][3]);
    _theta[i][4] = atan2(-r13*(c1*c23*c4 + s1*s4) - r23*(s1*c23*c4 - c1*s4) + r33*(s23*c4),
        -(r13*c1 + r23*s1)*s23 - r33*c23);
    _theta[i+1][4] = -_theta[i][4];
  }
  //---------------------------------------------------------------
  //  Compute theta6
  //---------------------------------------------------------------
  double s5, c5;
  double r11 = _t.rotation().matrix()[0][0];
  double r21 = _t.rotation().matrix()[1][0];
  double r31 = _t.rotation().matrix()[2][0];
  for(size_t i = 0; i < 8; i += 2) {
    s1 = sin(_theta[i][0]);
    c1 = cos(_theta[i][0]);
    s23 = sin(_theta[i][1] + _theta[i][2]);
    c23 = cos(_theta[i][1] + _theta[i][2]);
    s4 = sin(_theta[i][3]);
    c4 = cos(_theta[i][3]);
    s5 = sin(_theta[i][4]);
    c5 = cos(_theta[i][4]);
    _theta[i][5] = atan2(-r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4 + c1*c4) + r31*s23*s4,
        r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5 + c23*s5));
    _theta[i+1][5] = _theta[i][5] + 180.0;
  }
}


MultiBody::
MultiBody() :
  m_comAvailable(false), m_bodyType(PASSIVE),
  m_maxAxisRange(0) {
    fill(m_boundingBox, m_boundingBox+6, 0);
  }

MultiBody::
~MultiBody() {
}

void
MultiBody::
Initialize(string _modelFile, const Transformation& _where, BodyType _type) {
  m_bodyType = _type;

  if(IsActive() || IsSurface())
    throw RunTimeException(WHERE,
        "Function not implemented for active or surface MultiBodys");

  if(IsPassive() || IsInternal()) {
    //create a fixed body
    shared_ptr<FixedBody> fix(new FixedBody(this, _modelFile));
    fix->Read();

    Transformation worldTransform(_where);
    fix->PutWorldTransformation(worldTransform);

    //add fixed body to multibody
    AddBody(fix);

    FindBoundingBox();
    ComputeCenterOfMass();
  }
}


//-------------------------------------------------------------------
//  GetFreeBody
//-------------------------------------------------------------------
shared_ptr<FreeBody> MultiBody::GetFreeBody(int _index) const
{
  if(_index < (int)freeBody.size())
    return freeBody[_index];
  else
    return shared_ptr<FreeBody>();
}

//-------------------------------------------------------------------
//  GetFreeBodyCount
//-------------------------------------------------------------------
int MultiBody::GetFreeBodyCount() const
{
  return freeBody.size();
}

//-------------------------------------------------------------------
//  GetFreeBodyIndex
//-------------------------------------------------------------------
int MultiBody::GetFreeBodyIndex(const FreeBody& _b) const {
  for(size_t i=0; i<freeBody.size(); ++i)
    if(&_b == freeBody[i].get())
      return i;
  return -1;
}
int MultiBody::GetFreeBodyIndex(const shared_ptr<FreeBody>& _b) const {
  for(size_t i=0; i<freeBody.size(); ++i)
    if(_b == freeBody[i])
      return i;
  return -1;
}

//===================================================================
//  AddBody
//===================================================================
void MultiBody::AddBody(const shared_ptr<FreeBody>& _body)
{
  freeBody.push_back(_body);
}


//-------------------------------------------------------------------
//  GetFixedBody
//-------------------------------------------------------------------
shared_ptr<FixedBody> MultiBody::GetFixedBody(int _index) const
{
  if(_index < (int)fixedBody.size())
    return fixedBody[_index];
  else
    return shared_ptr<FixedBody>();
}

//===================================================================
//  AddBody
//===================================================================
void MultiBody::AddBody(const shared_ptr<FixedBody>& _body)
{
  fixedBody.push_back(_body);
}

//-------------------------------------------------------------------
//  GetBody
//-------------------------------------------------------------------
shared_ptr<Body> MultiBody::GetBody(int _index) const
{
  if(_index < 0 || _index >= (int)(freeBody.size() + fixedBody.size())) {
    cout << "Error in MultiBody::GetBody !!" << endl;
    exit(-1);
  }
  else
    if(_index < (int)fixedBody.size())
      return fixedBody[_index];
    else
      return freeBody[_index - fixedBody.size()];
}

//-------------------------------------------------------------------
//  GetBodyCount
//-------------------------------------------------------------------
int MultiBody::GetBodyCount() const
{
  return freeBody.size() + fixedBody.size();
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
shared_ptr<Body> MultiBody::GetFirstBody() const
{
  //  I assume that the first body in the list is the anchor body.
  //  That is, all other bodies in the MultiBody are linked sequentially
  //  in a "forward" direction (with possible branches) from this anchor.
  if(!fixedBody.empty())
    return fixedBody.front();
  else
    if(!freeBody.empty())
      return freeBody.front();
    else
      return shared_ptr<Body>();
}


//===================================================================
//  GetNumberOfLinks
//
//  Output: The number of links in "this" MultiBody
//===================================================================
int MultiBody::GetNumberOfLinks() const
{
  shared_ptr<Body> bb = GetFirstBody();
  if(bb == shared_ptr<Body>())
    return 0;

  shared_ptr<Body> b = bb;
  int i = 1;
  while (b->ForwardConnectionCount() > 0) {
    b = b->GetForwardConnection(0).GetNextBody();
    i++;
  }
  return i-1;
}

//-------------------------------------------------------------------
//  IsManipulator
///  Function: Determine if the MultiBody at hand is a manipulator.
///            If there is no free body attached to it,
///            it is considered to be a manipulator
///
///  Output:   True/False
//-------------------------------------------------------------------
bool MultiBody::IsManipulator() const
{
  return !freeBody.empty();
}


//-------------------------------------------------------------------
//  GetCenterOfMass
//-------------------------------------------------------------------
Vector3d MultiBody::GetCenterOfMass()
{
  if(!m_comAvailable)
    ComputeCenterOfMass();
  return CenterOfMass;
}

//===================================================================
//  GetMaxAxisRange
//===================================================================
double MultiBody::GetMaxAxisRange() const
{
  return m_maxAxisRange;
}

//===================================================================
//  GetBoundingBox
//===================================================================
const double * MultiBody::GetBoundingBox() const
{
  return m_boundingBox;
}

//===================================================================
//  GetBoundingSphereRadius
//    the maximum size of this multibody
//===================================================================
double MultiBody::GetBoundingSphereRadius() const
{
  double result = GetBody(0)->GetPolyhedron().m_maxRadius;
  for(int i=1; i<GetBodyCount(); ++i)
    result += GetBody(i)->GetPolyhedron().m_maxRadius * 2.0;
  return result;
}

//===================================================================
//  GetInsideSphere Radius
//    the minimum size of the multibody
//===================================================================
double MultiBody::GetInsideSphereRadius() const
{
  double result = GetBody(0)->GetPolyhedron().m_minRadius;
  for(int i=1; i<GetBodyCount(); ++i)
    if(GetBody(i)->GetPolyhedron().m_minRadius > result)
      result = GetBody(i)->GetPolyhedron().m_minRadius;
  return result;
}


class IsConnectionGloballyFirst {
  public:
    bool operator()(const shared_ptr<Connection>& _a, const shared_ptr<Connection>& _b) const {
      return _a->GetGlobalIndex() < _b->GetGlobalIndex();
    }
} connectionComparator;

//===================================================================
//  Read
//===================================================================
void
MultiBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  string multibodyType = ReadFieldString(_is, _cbs,
      "Failed reading multibody type."
      " Options are: active, passive, internal, or surface.");
  //need to update to use enum like Body and Connections
  if(multibodyType == "PASSIVE")
    m_bodyType = PASSIVE;
  else if(multibodyType == "ACTIVE")
    m_bodyType = ACTIVE;
  else if(multibodyType == "SURFACE")
    m_bodyType = SURFACE;
  else if(multibodyType == "INTERNAL")
    m_bodyType = INTERNAL;
  else
    throw ParseException(_cbs.Where(),
        "Unknown MultiBody type '" + multibodyType + "'."
        " Options are: 'active', 'passive', 'internal', or 'surface'.");

  if(IsActive()) {

    m_baseIndex = -1;

    size_t bodyCount = ReadField<size_t>(_is, _cbs,
        "Failed reading body count.");

    for(size_t i=0; i < bodyCount && _is; ++i) {
      //read the free body
      shared_ptr<FreeBody> free(new FreeBody(this));
      free->Read(_is, _cbs);

      //add object to multibody
      AddBody(free);

      if(free->IsBase() && m_baseIndex == size_t(-1)) {
        m_baseIndex = i;
        m_baseBody = free;
        m_baseType = free->GetBase();
        m_baseMovement = free->GetBaseMovement();
      }
    }

    if(m_baseIndex == size_t(-1))
      throw ParseException(_cbs.Where(), "Active body has no base.");

    //get connection info
    string connectionTag = ReadFieldString(_is, _cbs,
        "Failed reading connections tag.");
    if(connectionTag != "CONNECTIONS")
      throw ParseException(_cbs.Where(),
          "Unknwon connections tag '" + connectionTag + "'."
          " Should read 'Connections'.");
    size_t connectionCount = ReadField<size_t>(_is, _cbs,
        "Failed reading number of connections.");

    for(size_t i = 0; i < connectionCount; ++i) {
      //add connection info to multibody connection map
      shared_ptr<Connection> c(new Connection(this));
      m_joints.push_back(c);
      m_joints.back()->Read(_is, _cbs);
    }
    sort(m_joints.begin(), m_joints.end(), connectionComparator);

  }
  else{ //Passive, Surface, Internal
    if(IsSurface())
      m_label = ReadFieldString(_is, _cbs, "Failed reading surface tag.");

    //all are same type, namely fixed body
    shared_ptr<FixedBody> fix(new FixedBody(this));
    fix->Read(_is, _cbs);

    //add fixed body to multibody
    AddBody(fix);
  }

  FindBoundingBox();
  ComputeCenterOfMass();
}

void
MultiBody::
InitializeDOFs(ostream* _os) {

  size_t dof = 0;

  if(_os) {
    *_os << "DoF List: " << endl;
    *_os << "\tRobot with base index " << m_baseIndex;
    *_os << " (" << m_baseBody->GetFileName() << "):" << endl;
  }

  if(m_baseType == Body::PLANAR) {
    m_dofTypes.push_back(POS);
    m_dofTypes.push_back(POS);

    if(_os) {
      *_os << "\t\t" << dof++ << ": X position" << endl;
      *_os << "\t\t" << dof++ << ": Y position" << endl;
    }

    if(m_baseMovement == Body::ROTATIONAL) {
      m_dofTypes.push_back(ROT);

      if(_os)
        *_os << "\t\t" << dof++ << ": Rotation about Z" << endl;
    }
  }
  if(m_baseType == Body::VOLUMETRIC) {
    m_dofTypes.push_back(POS);
    m_dofTypes.push_back(POS);
    m_dofTypes.push_back(POS);

    if(_os) {
      *_os << "\t\t" << dof++ << ": X position" << endl;
      *_os << "\t\t" << dof++ << ": Y position" << endl;
      *_os << "\t\t" << dof++ << ": Z position" << endl;
    }
    if(m_baseMovement == Body::ROTATIONAL) {
      m_dofTypes.push_back(ROT);
      m_dofTypes.push_back(ROT);
      m_dofTypes.push_back(ROT);

      if(_os) {
        *_os << "\t\t" << dof++ << ": Rotation about X" << endl;
        *_os << "\t\t" << dof++ << ": Rotation about Y" << endl;
        *_os << "\t\t" << dof++ << ": Rotation about Z" << endl;
      }
    }
  }

  for(auto& joint : m_joints) {
    switch(joint->GetConnectionType()) {
      case Connection::REVOLUTE:
        m_dofTypes.push_back(JOINT);

        if(_os) {
          *_os << "\t\t" << dof++ << ": ";
          *_os << "Rotational joint from body " << joint->GetPreviousBodyIndex();
          *_os << " (" << joint->GetPreviousBody()->GetFileName() << ")";
          *_os << " to body " << joint->GetNextBodyIndex();
          *_os << " (" << joint->GetNextBody()->GetFileName() << ")" << endl;
        }
        break;

      case Connection::SPHERICAL:
        m_dofTypes.push_back(JOINT);
        m_dofTypes.push_back(JOINT);

        if(_os) {
          *_os << "\t\t" << dof++;
          *_os << "/" << dof++ << ": ";
          *_os << "Spherical joint from body " << joint->GetPreviousBodyIndex();
          *_os << " (" << joint->GetPreviousBody()->GetFileName() << ")";
          *_os << " to body " << joint->GetNextBodyIndex();
          *_os << " (" << joint->GetNextBody()->GetFileName() << ")" << endl;
        }
        break;

      case Connection::NONACTUATED:
        break;
    }
  }
}

void MultiBody::buildCDstructure(cd_predefined cdtype)
{
  for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I)
    (*I)->BuildCDStructure(cdtype);
  for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I)
    (*I)->BuildCDStructure(cdtype);
}


//===================================================================
//  Write
//===================================================================
void MultiBody::Write(ostream & _os)
{
  if(fixedBody.size() > 0){
    switch(m_bodyType){
      case INTERNAL:
        _os << "INTERNAL" << endl;
        break;
      case SURFACE:
        _os << "SURFACE " << m_label << endl;
        break;
      case PASSIVE:
        _os << "PASSIVE" << endl;
        break;
      default:
        cerr << "Multibody::Write Error: unrecognized fixed body type. Valid options are PASSIVE, INTERNAL, and SURFACE"
          << endl;
        exit(1);
        break;
    }
    for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I){
      _os << **I << endl;
    }
  }
  else if(freeBody.size() > 0){
    _os << "ACTIVE" << endl;
    _os << freeBody.size() << endl;
    for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I){
      _os << **I << endl;
    }
    _os << "Connection" << endl;
    size_t numConnection = 0;
    for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I) {
      numConnection += (*I)->ForwardConnectionCount();
    }
    _os << numConnection << endl;
    for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I) {
      for(int j=0; j < (*I)->ForwardConnectionCount(); j++)
        _os << (*I)->GetForwardConnection(j);
    }
  }
}


//===================================================================
//  ConfigureJoint
//
//  Function: Configure the joint by the given amount of displacement
//
//===================================================================
void MultiBody::ConfigureJoint(double * _s, int _dof)
{
  for(size_t i = 0; i<(size_t)_dof; ++i)
    freeBody[i]->GetForwardConnection(0).GetDHparameters().m_theta = _s[i];
}

//===================================================================
//  ComputeCenterOfMass
//
//  The degree of approximation in calculating center of mass is
//  the same as in Body.cpp. To be more accurate, we need to
//  modify this function to consider the mass of each body.
//
//===================================================================
void MultiBody::ComputeCenterOfMass()
{
  if(freeBody.empty() && fixedBody.empty())
  {
    cout << "\nERROR: No MultiBodies to take MultiBody::CenterOfMass from...\n";
  }
  else
  {
    Vector3d sum(0,0,0);
    for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I)
      sum = sum + (*I)->GetCenterOfMass();
    for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I)
      sum = sum + (*I)->GetCenterOfMass();
    CenterOfMass = sum / (freeBody.size() + fixedBody.size());
    m_comAvailable = true;
  }
}


//===================================================================
//  FindBoundingBox
//===================================================================
void MultiBody::FindBoundingBox() {
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = maxx = maxy = maxz = 0;

  ///////////////////////////////////////////////////////////
  //Check Free Bodys' Boudning Box
  if(!freeBody.empty())
  {
    freeBody.front()->FindBoundingBox();
    double* tmp = freeBody.front()->GetBoundingBox();
    minx = tmp[0]; maxx = tmp[1];
    miny = tmp[2]; maxy = tmp[3];
    minz = tmp[4]; maxz = tmp[5];

    for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin()+1; I != freeBody.end(); ++I)
    {
      (*I)->FindBoundingBox();
      tmp = (*I)->GetBoundingBox();
      minx = min(minx, tmp[0]); maxx = max(maxx, tmp[1]);
      miny = min(miny, tmp[2]); maxy = max(maxy, tmp[3]);
      minz = min(minz, tmp[4]); maxz = max(maxz, tmp[5]);
    }
  }

  ///////////////////////////////////////////////////////////
  //Check Fixed Bodys' Boudning Box
  if(!fixedBody.empty())
  {
    fixedBody.front()->FindBoundingBox();
    double* tmp = fixedBody.front()->GetBoundingBox();
    minx = tmp[0]; maxx = tmp[1];
    miny = tmp[2]; maxy = tmp[3];
    minz = tmp[4]; maxz = tmp[5];

    for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin()+1; I != fixedBody.end(); ++I)
    {
      (*I)->FindBoundingBox();
      tmp = (*I)->GetBoundingBox();
      minx = min(minx, tmp[0]); maxx = max(maxx, tmp[1]);
      miny = min(miny, tmp[2]); maxy = max(maxy, tmp[3]);
      minz = min(minz, tmp[4]); maxz = max(maxz, tmp[5]);
    }
  }

  ///////////////////////////////////////////////////////////
  //Pack
  m_boundingBox[0] = minx; m_boundingBox[1] = maxx;
  m_boundingBox[2] = miny; m_boundingBox[3] = maxy;
  m_boundingBox[4] = minz; m_boundingBox[5] = maxz;

  ///////////////////////////////////////////////////////////
  // Find m_maxAxisRange
  double rangex = maxx - minx;
  double rangey = maxy - miny;
  double rangez = maxz - minz;
  m_maxAxisRange = max(rangex, max(rangey,rangez));
}


#ifdef USE_SOLID
void MultiBody::UpdateVertexBase() {
  for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I)
    (*I)->UpdateVertexBase();
  for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I)
    (*I)->UpdateVertexBase();
}
#endif

struct vertex_index_distance {
  size_t first_index, second_index;
  double distance;

  vertex_index_distance(size_t i1 = MAX_INT, size_t i2 = MAX_INT, double d = MAX_INT) : first_index(i1), second_index(i2), distance(d) {}
  ~vertex_index_distance() {}

  bool operator<(const vertex_index_distance& v) const { return distance < v.distance; }
};

struct first_index_equals : public unary_function<vertex_index_distance, bool> {
  size_t index;

  first_index_equals(size_t i) : index(i) {}
  ~first_index_equals() {}

  bool operator()(const vertex_index_distance& v) const { return v.first_index == index; }
};

struct second_index_equals : public unary_function<vertex_index_distance, bool> {
  size_t index;

  second_index_equals(size_t i) : index(i) {}
  ~second_index_equals() {}

  bool operator()(const vertex_index_distance& v) const { return v.second_index == index; }
};

//==================================================================
//Polygonal Approximation
void MultiBody::PolygonalApproximation(vector<Vector3d>& result) {
  result.clear();

  int nfree = GetFreeBodyCount();

  if(nfree == 1)
  {
    //rigid body, return the line between the first 4 vertices and the second 4 vertices of the world bounding box
    GMSPolyhedron bbox = this->GetFreeBody(0)->GetWorldBoundingBox();

    Vector3d joint(0, 0, 0);
    for(size_t i = 0; i<4; ++i)
      joint = joint + bbox.m_vertexList[i];
    joint = joint / 4;
    result.push_back(joint);

    joint = Vector3d(0, 0, 0);
    for(size_t i = 4; i<8; ++i)
      joint = joint + bbox.m_vertexList[i];
    joint = joint / 4;
    result.push_back(joint);
  }
  else
  {
    if(nfree > 0)
    {
      for(int i=0; i<nfree-1; i++)
      {
        GMSPolyhedron first_bbox = this->GetFreeBody(i)->GetWorldBoundingBox();
        GMSPolyhedron second_bbox = this->GetFreeBody(i)->GetForwardConnection(0).GetNextBody()->GetWorldBoundingBox();

        //find the four closest pairs of points between first_bbox and second_bbox
        vector<Vector3d> first_vertices = first_bbox.m_vertexList;
        vector<Vector3d> second_vertices = second_bbox.m_vertexList;
        vector<vertex_index_distance> closest_distances;
        for(int num=0; num<4; ++num)
        {
          vector<vertex_index_distance> distances;
          for(size_t j=0; j<first_vertices.size(); ++j)
            for(size_t k=0; k<second_vertices.size(); ++k)
              distances.push_back(vertex_index_distance(j, k, (first_vertices[j] - second_vertices[k]).norm()));
          vector<vertex_index_distance>::const_iterator min = min_element(distances.begin(), distances.end());
          closest_distances.push_back(*min);
          //mark vertices as used by setting to MAX_INT and -MAX_INT
          first_vertices[min->first_index] = Vector3d(MAX_INT, MAX_INT, MAX_INT);
          second_vertices[min->second_index] = Vector3d(-MAX_INT, -MAX_INT, -MAX_INT);
        }

        //first body in linkage, add the endpoint of linkage 1 that is not closest to linkage 2
        if(i == 0)
        {
          Vector3d other_joint(0, 0, 0);
          int num = 0;
          for(size_t k = 0; num<4 && k<first_bbox.m_vertexList.size(); ++k)
            if(find_if(closest_distances.begin(), closest_distances.end(), first_index_equals(k)) == closest_distances.end())
            {
              other_joint = other_joint + first_bbox.m_vertexList[k];
              num++;
            }
          other_joint = other_joint / num;
          result.push_back(other_joint);
        }

        //compute the joint as the closest 4 vertices from linkage 1 and linkage 2
        Vector3d joint(0, 0, 0);
        for(size_t k = 0; k<4; ++k)
          joint = joint + first_bbox.m_vertexList[closest_distances[k].first_index];
        for(size_t k = 0; k<4; ++k)
          joint = joint + second_bbox.m_vertexList[closest_distances[k].second_index];
        joint = joint / 8;
        result.push_back(joint);

        //last body in linkage, add endpoint of linkage 2 that is not closest to linkage 1
        if(i == nfree-2)
        {
          Vector3d other_joint(0, 0, 0);
          int num = 0;
          for(size_t k = 0; num<4 && k<second_bbox.m_vertexList.size(); ++k)
            if(find_if(closest_distances.begin(), closest_distances.end(), second_index_equals(k)) == closest_distances.end())
            {
              other_joint = other_joint + second_bbox.m_vertexList[k];
              num++;
            }
          other_joint = other_joint / num;

          result.push_back(other_joint);
        }
      }
    }
  }
}

bool
MultiBody::
IsInternal() const{
  return m_bodyType == INTERNAL;
}

bool
MultiBody::
IsSurface() const {
  return m_bodyType == SURFACE;
}

bool
MultiBody::
IsActive() const{
  return m_bodyType == ACTIVE;
}

bool
MultiBody::
IsPassive() const{
  return m_bodyType == PASSIVE;
}

size_t
MultiBody::
PosDOF() const {
  switch(m_baseType) {
    case Body::PLANAR:
      return 2;
    case Body::VOLUMETRIC:
      return 3;
    default:
      return 0;
  }
}

void
MultiBody::
Configure(const vector<double>& _v) {
  int index = 0;
  int posIndex = index;
  double x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0;
  if(m_baseType != Body::FIXED) {
    x = _v[posIndex];
    y = _v[posIndex + 1];
    index += 2;
    if(m_baseType == Body::VOLUMETRIC) {
      index++;
      z = _v[posIndex + 2];
    }
    if(m_baseMovement == Body::ROTATIONAL) {
      if(m_baseType == Body::PLANAR) {
        index++;
        gamma = _v[posIndex + 2];
      }
      else {
        index += 3;
        alpha = _v[posIndex + 3];
        beta = _v[posIndex + 4];
        gamma = _v[posIndex + 5];
      }
    }
    // configure the robot according to current Cfg: joint parameters
    // (and base locations/orientations for free flying robots.)
    Transformation t1(Vector3d(x, y, z), Orientation(EulerAngle(gamma*PI, beta*PI, alpha*PI)));
    // update link i
    GetFreeBody(m_baseIndex)->Configure(t1);
  }
  typedef MultiBody::JointMap::iterator MIT;
  for(auto& joint : m_joints) {
    if(joint->GetConnectionType() != Connection::NONACTUATED) {
      size_t second = joint->GetNextBodyIndex();
      GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().m_theta = _v[index]*PI;
      index++;
      if(joint->GetConnectionType() == Connection::SPHERICAL) {
        GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().m_alpha = _v[index]*PI;
        index++;
      }
    }
  }  // config the robot
  for(auto& body : freeBody) {
    if(body->ForwardConnectionCount() == 0)  // tree tips: leaves.
      body->GetWorldTransformation();
  }
}

//generates random configuration within C-space
vector<double>
MultiBody::
GetRandomCfg(shared_ptr<Boundary>& _bounds) {
  vector<double> v;
  v.reserve(DOF());
  if(m_baseType == Body::PLANAR || m_baseType == Body::VOLUMETRIC) {
    Point3d p = _bounds->GetRandomPoint();
    size_t posDOF = m_baseType == Body::VOLUMETRIC ? 3 : 2;
    for(size_t i = 0; i < posDOF; i++)
      v.push_back(p[i]);
    if(m_baseMovement == Body::ROTATIONAL) {
      size_t oriDOF = m_baseType == Body::VOLUMETRIC ? 3 : 1;
      for(size_t i = 0; i < oriDOF; i++)
        v.push_back(2.0*DRand()-1.0);
    }
  }
  for(auto& joint : m_joints) {
    if(joint->GetConnectionType() == Connection::REVOLUTE) {
      pair<double, double> r = joint->GetJointLimits(0);
      double t = DRand()*(r.second-r.first)+r.first;
      v.push_back(t);
    }
    else if(joint->GetConnectionType() == Connection::SPHERICAL) {
      pair<double, double> r = joint->GetJointLimits(0);
      double t = DRand()*(r.second-r.first)+r.first;
      r = joint->GetJointLimits(1);
      double a = DRand()*(r.second-r.first)+r.first;
      v.push_back(t);
      v.push_back(a);
    }
  }
  return v;
}
