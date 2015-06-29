#include "ActiveMultiBody.h"

#include "MPProblem/Boundary.h"
#include "MPProblem/Geometry/FreeBody.h"

ActiveMultiBody::
ActiveMultiBody() : MultiBody() {
}

shared_ptr<FreeBody>
ActiveMultiBody::
GetFreeBody(size_t _index) const {
  if(_index < freeBody.size())
    return freeBody[_index];
  else
    return shared_ptr<FreeBody>();
}

size_t
ActiveMultiBody::
GetFreeBodyCount() const {
  return freeBody.size();
}

size_t
ActiveMultiBody::
GetFreeBodyIndex(const FreeBody& _b) const {
  for(size_t i=0; i<freeBody.size(); ++i)
    if(&_b == freeBody[i].get())
      return i;
  return -1;
}

size_t
ActiveMultiBody::
GetFreeBodyIndex(const shared_ptr<FreeBody>& _b) const {
  for(size_t i=0; i<freeBody.size(); ++i)
    if(_b == freeBody[i])
      return i;
  return -1;
}

void
ActiveMultiBody::
AddBody(const shared_ptr<FreeBody>& _body) {
  freeBody.push_back(_body);
  MultiBody::AddBody(_body);
}

shared_ptr<Body>
ActiveMultiBody::GetFirstBody() const {
  //  I assume that the first body in the list is the anchor body.
  //  That is, all other bodies in the MultiBody are linked sequentially
  //  in a "forward" direction (with possible branches) from this anchor.
  if(!freeBody.empty())
    return freeBody.front();
  else
    return shared_ptr<Body>();
}

size_t
ActiveMultiBody::
GetNumberOfLinks() const {
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

bool
ActiveMultiBody::
IsManipulator() const {
  return !freeBody.empty();
}

void
ActiveMultiBody::
InitializeDOFs(ostream* _os) {

  size_t dof = 0;

  if(_os) {
    *_os << "DoF List: " << endl;
    *_os << "\tRobot with base index " << m_baseIndex;
    *_os << " (" << m_baseBody->GetFileName() << "):" << endl;
  }

  if(m_baseType == Body::PLANAR) {
    m_dofTypes.push_back(DofType::Positional);
    m_dofTypes.push_back(DofType::Positional);

    if(_os) {
      *_os << "\t\t" << dof++ << ": X position" << endl;
      *_os << "\t\t" << dof++ << ": Y position" << endl;
    }

    if(m_baseMovement == Body::ROTATIONAL) {
      m_dofTypes.push_back(DofType::Rotational);

      if(_os)
        *_os << "\t\t" << dof++ << ": Rotation about Z" << endl;
    }
  }
  if(m_baseType == Body::VOLUMETRIC) {
    m_dofTypes.push_back(DofType::Positional);
    m_dofTypes.push_back(DofType::Positional);
    m_dofTypes.push_back(DofType::Positional);

    if(_os) {
      *_os << "\t\t" << dof++ << ": X position" << endl;
      *_os << "\t\t" << dof++ << ": Y position" << endl;
      *_os << "\t\t" << dof++ << ": Z position" << endl;
    }
    if(m_baseMovement == Body::ROTATIONAL) {
      m_dofTypes.push_back(DofType::Rotational);
      m_dofTypes.push_back(DofType::Rotational);
      m_dofTypes.push_back(DofType::Rotational);

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
        m_dofTypes.push_back(DofType::Joint);

        if(_os) {
          *_os << "\t\t" << dof++ << ": ";
          *_os << "Rotational joint from body " << joint->GetPreviousBodyIndex();
          *_os << " (" << joint->GetPreviousBody()->GetFileName() << ")";
          *_os << " to body " << joint->GetNextBodyIndex();
          *_os << " (" << joint->GetNextBody()->GetFileName() << ")" << endl;
        }
        break;

      case Connection::SPHERICAL:
        m_dofTypes.push_back(DofType::Joint);
        m_dofTypes.push_back(DofType::Joint);

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

void
ActiveMultiBody::
ConfigureJoint(double * _s, size_t _dof) {
  for(size_t i = 0; i < _dof; ++i)
    freeBody[i]->GetForwardConnection(0).GetDHparameters().m_theta = _s[i];
}

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

void
ActiveMultiBody::
PolygonalApproximation(vector<Vector3d>& result) {
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
size_t
ActiveMultiBody::
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
ActiveMultiBody::
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
ActiveMultiBody::
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

void
ActiveMultiBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
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
  sort(m_joints.begin(), m_joints.end(),
      [](const shared_ptr<Connection>& _a, const shared_ptr<Connection>& _b) {
      return _a->GetGlobalIndex() < _b->GetGlobalIndex();
      });

  FindBoundingBox();
  ComputeCenterOfMass();
}

void
ActiveMultiBody::
Write(ostream & _os) {
  _os << GetTagFromBodyType(GetBodyType()) << endl;
  _os << freeBody.size() << endl;
  for(auto& body : freeBody)
    _os << *body << endl;
  _os << "Connections" << endl;
  size_t numConnection = 0;
  for(auto& body : freeBody)
    numConnection += body->ForwardConnectionCount();
  _os << numConnection << endl;
  for(auto& body : freeBody)
    for(int j=0; j < body->ForwardConnectionCount(); j++)
      _os << body->GetForwardConnection(j);
}
