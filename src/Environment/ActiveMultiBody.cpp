#include "ActiveMultiBody.h"

#include "Boundary.h"
#include "FreeBody.h"

ActiveMultiBody::
ActiveMultiBody() : MultiBody() {
  m_multiBodyType = MultiBodyType::Active;
}

size_t
ActiveMultiBody::
NumFreeBody() const {
  return m_freeBody.size();
}

shared_ptr<FreeBody>
ActiveMultiBody::
GetFreeBody(size_t _index) const {
  if(_index < m_freeBody.size())
    return m_freeBody[_index];
  else
    throw RunTimeException(WHERE,
        "Cannot access FixedBody(" + ::to_string(_index) + ").");
}

void
ActiveMultiBody::
InitializeDOFs(shared_ptr<Boundary>& _b, ostream* _os) {

  size_t dof = 0;

  if(_os) {
    *_os << "DoF List: " << endl;
    *_os << "\tRobot with base index " << m_baseIndex;
    *_os << " (" << m_baseBody->GetFileName() << "):" << endl;
  }

  // Define a function to process free bodies.
  auto processBody = [&](const shared_ptr<FreeBody>& _freeBody) {
    auto bodyType = _freeBody->GetBodyType();
    auto movementType = _freeBody->GetMovementType();

    if(bodyType == FreeBody::BodyType::Planar) {
      m_dofTypes.push_back(DofType::Positional);
      m_dofTypes.push_back(DofType::Positional);
      m_dofInfo.push_back(DOFInfo("Base X Translation ",
            _b->GetRange(0).first, _b->GetRange(0).second));
      m_dofInfo.push_back(DOFInfo("Base Y Translation ",
            _b->GetRange(1).first, _b->GetRange(1).second));

      if(_os) {
        *_os << "\t\t" << dof++ << ": X position" << endl;
        *_os << "\t\t" << dof++ << ": Y position" << endl;
      }

      if(movementType == FreeBody::MovementType::Rotational) {
        m_dofTypes.push_back(DofType::Rotational);
        m_dofInfo.push_back(DOFInfo("Base Rotation ", -1.0, 1.0));

      if(_os)
              *_os << "\t\t" << dof++ << ": Rotation about Z" << endl;
      }
    }
    else if(bodyType == FreeBody::BodyType::Volumetric) {
      m_dofTypes.push_back(DofType::Positional);
      m_dofTypes.push_back(DofType::Positional);
      m_dofTypes.push_back(DofType::Positional);
      m_dofInfo.push_back(DOFInfo("Base X Translation ",
            _b->GetRange(0).first, _b->GetRange(0).second));
      m_dofInfo.push_back(DOFInfo("Base Y Translation ",
            _b->GetRange(1).first, _b->GetRange(1).second));
      m_dofInfo.push_back(DOFInfo("Base Z Translation ",
            _b->GetRange(2).first, _b->GetRange(2).second));

      if(_os) {
        *_os << "\t\t" << dof++ << ": X position" << endl;
        *_os << "\t\t" << dof++ << ": Y position" << endl;
        *_os << "\t\t" << dof++ << ": Z position" << endl;
      }
      if(movementType == FreeBody::MovementType::Rotational) {
        m_dofTypes.push_back(DofType::Rotational);
        m_dofTypes.push_back(DofType::Rotational);
        m_dofTypes.push_back(DofType::Rotational);
        m_dofInfo.push_back(DOFInfo("Base X Rotation ", -1.0, 1.0));
        m_dofInfo.push_back(DOFInfo("Base Y Rotation ", -1.0, 1.0));
        m_dofInfo.push_back(DOFInfo("Base Z Rotation ", -1.0, 1.0));

        if(_os) {
                *_os << "\t\t" << dof++ << ": Rotation about X" << endl;
                *_os << "\t\t" << dof++ << ": Rotation about Y" << endl;
                *_os << "\t\t" << dof++ << ": Rotation about Z" << endl;
        }
      }
    }
  };

  // Define a function to process joints.
  auto processJoint = [&](Joint& _joint) {
    switch(_joint->GetConnectionType()) {
      case Connection::JointType::Revolute:
        m_dofTypes.push_back(DofType::Joint);
        m_dofInfo.push_back(DOFInfo("Revolute Joint " +
              ::to_string(_joint->GetGlobalIndex()) + " Angle",
              _joint->GetJointLimits(0).first, _joint->GetJointLimits(0).second));

        if(_os) {
          *_os << "\t\t" << dof++ << ": ";
          *_os << "Rotational joint from body " << _joint->GetPreviousBodyIndex();
          *_os << " (" << _joint->GetPreviousBody()->GetFileName() << ")";
          *_os << " to body " << _joint->GetNextBodyIndex();
          *_os << " (" << _joint->GetNextBody()->GetFileName() << ")" << endl;
        }
        break;

      case Connection::JointType::Spherical:
        m_dofTypes.push_back(DofType::Joint);
        m_dofTypes.push_back(DofType::Joint);
        m_dofInfo.push_back(DOFInfo("Spherical Joint " +
              ::to_string(_joint->GetGlobalIndex()) + " Angle 1",
              _joint->GetJointLimits(0).first, _joint->GetJointLimits(0).second));
        m_dofInfo.push_back(DOFInfo("Spherical Joint " +
              ::to_string(_joint->GetGlobalIndex()) + " Angle 2",
              _joint->GetJointLimits(1).first, _joint->GetJointLimits(1).second));

        if(_os) {
          *_os << "\t\t" << dof++;
          *_os << "/" << dof++ << ": ";
          *_os << "Spherical joint from body " << _joint->GetPreviousBodyIndex();
          *_os << " (" << _joint->GetPreviousBody()->GetFileName() << ")";
          *_os << " to body " << _joint->GetNextBodyIndex();
          *_os << " (" << _joint->GetNextBody()->GetFileName() << ")" << endl;
        }
        break;

      case Connection::JointType::NonActuated:
        break;
    }
  };

  processBody(m_baseBody);

  for(auto& joint : m_joints)
    processJoint(joint);

  // add dofs of multiple free bodies without connections for composite C-Spaces.
  if(m_joints.empty() && m_freeBody.size() > 1)
    for(size_t i = 1; i < m_freeBody.size(); ++i)
      processBody(m_freeBody[i]);
}

size_t
ActiveMultiBody::
PosDOF() const {
  switch(m_baseType) {
    case FreeBody::BodyType::Planar:
      return 2;
    case FreeBody::BodyType::Volumetric:
      return 3;
    default:
      return 0;
  }
}

void
ActiveMultiBody::
Configure(const vector<double>& _v) {
  int index = 0;
  if(m_baseType != FreeBody::BodyType::Fixed) {
    Transformation t = GenerateModelTransformation(_v, index, m_baseType,
        m_baseMovement);
    m_baseBody->Configure(t);
  }
  for(auto& joint : m_joints) {
    if(joint->GetConnectionType() != Connection::JointType::NonActuated) {
      size_t second = joint->GetNextBodyIndex();
      GetFreeBody(second)->GetBackwardConnection(0).GetDHParameters().m_theta =
          _v[index++]*PI;
      if(joint->GetConnectionType() == Connection::JointType::Spherical)
        GetFreeBody(second)->GetBackwardConnection(0).GetDHParameters().m_alpha =
            _v[index++]*PI;
    }
  }
  if(m_joints.empty() && m_freeBody.size() > 1) {
    for(size_t i = 1; i < m_freeBody.size(); ++i) {
      auto freeBody = m_freeBody[i].get();
      Transformation t = GenerateModelTransformation(_v, index,
          freeBody->GetBodyType(), freeBody->GetMovementType());
      freeBody->Configure(t);
    }
  }
  // configure the robot
  for(auto& body : m_freeBody)
    if(body->ForwardConnectionCount() == 0)  // tree tips: leaves.
      body->GetWorldTransformation();
}

void
ActiveMultiBody::
Configure(const vector<double>& _v, const vector<double>& _t) {
  int index = 0, t_index = 0;
  if(m_baseType != FreeBody::BodyType::Fixed) {
    Transformation t = GenerateModelTransformation(_v, index, m_baseType,
        m_baseMovement);
    m_baseBody->Configure(t);
  }
  for(auto& joint : m_joints) {
    if(joint->GetConnectionType() != Connection::JointType::NonActuated) {
      size_t second = joint->GetNextBodyIndex();
      GetFreeBody(second)->GetBackwardConnection(0).GetDHParameters().m_theta =
          _v[index++]*PI;
      if(joint->GetConnectionType() == Connection::JointType::Spherical)
        GetFreeBody(second)->GetBackwardConnection(0).GetDHParameters().m_alpha =
            _v[index++]*PI;
      GetFreeBody(joint->GetNextBodyIndex())->GetBackwardConnection(0).
          GetDHParameters().m_theta = _t[t_index++];
    }
  }
  if(m_joints.empty() && m_freeBody.size() > 1) {
    for(size_t i = 1; i < m_freeBody.size(); ++i) {
      auto freeBody  = m_freeBody[i];
      Transformation t = GenerateModelTransformation(_v, index,
          freeBody->GetBodyType(), freeBody->GetMovementType());
      freeBody->Configure(t);
    }
  }
  // configure the robot
  for(auto& body : m_freeBody)
    if(body->ForwardConnectionCount() == 0)  // tree tips: leaves.
      body->GetWorldTransformation();
}

void
ActiveMultiBody::
ConfigureRender(const vector<double>& _v) {
  int index = 0;
  if(m_baseType != FreeBody::BodyType::Fixed) {
    Transformation t = GenerateModelTransformation(_v, index, m_baseType,
        m_baseMovement);
    m_baseBody->ConfigureRender(t);
  }
  for(auto& joint : m_joints) {
    if(joint->GetConnectionType() != Connection::JointType::NonActuated) {
      size_t second = joint->GetNextBodyIndex();
      GetFreeBody(second)->GetBackwardConnection(0).
        GetDHRenderParameters().m_theta = _v[index++]*PI;
      if(joint->GetConnectionType() == Connection::JointType::Spherical)
        GetFreeBody(second)->GetBackwardConnection(0).
          GetDHRenderParameters().m_alpha = _v[index++]*PI;
    }
  }
  if(m_joints.empty() && m_freeBody.size() > 1) {
    for(size_t i = 1; i < m_freeBody.size(); ++i) {
      auto freeBody = m_freeBody[i];
      Transformation t = GenerateModelTransformation(_v, index,
          freeBody->GetBodyType(), freeBody->GetMovementType());
      freeBody->ConfigureRender(t);
    }
  }
  // configure the robot
  for(auto& body : m_freeBody)
    if(body->ForwardConnectionCount() == 0)  // tree tips: leaves.
      body->GetRenderTransformation();
}

Transformation
ActiveMultiBody::
GenerateModelTransformation(const vector<double>& _v, int& _index,
    FreeBody::BodyType _bodyType, FreeBody::MovementType _movementType)
{
  Transformation t1;
  double x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0;
  x = _v[_index++];
  y = _v[_index++];
  if(_bodyType == FreeBody::BodyType::Volumetric) {
    z = _v[_index++];
  }
  if(_movementType == FreeBody::MovementType::Rotational) {
    if(_bodyType == FreeBody::BodyType::Planar) {
      gamma = _v[_index++];
    }
    else {
      alpha = _v[_index++];
      beta = _v[_index++];
      gamma = _v[_index++];
    }
  }
  t1 = Transformation(Vector3d(x, y, z),
      Orientation(EulerAngle(gamma * PI, beta * PI, alpha * PI)));
  return t1;
}

vector<double>
ActiveMultiBody::
GetRandomCfg(shared_ptr<Boundary>& _bounds) {
  vector<double> v;
  v.reserve(DOF());
  if(m_baseType == FreeBody::BodyType::Planar ||
      m_baseType == FreeBody::BodyType::Volumetric) {
    Point3d p = _bounds->GetRandomPoint();
    size_t posDOF = m_baseType == FreeBody::BodyType::Volumetric ? 3 : 2;
    for(size_t i = 0; i < posDOF; i++)
      v.push_back(p[i]);
    if(m_baseMovement == FreeBody::MovementType::Rotational) {
      size_t oriDOF = m_baseType == FreeBody::BodyType::Volumetric ? 3 : 1;
      for(size_t i = 0; i < oriDOF; i++)
        v.push_back(2.0*DRand()-1.0);
    }
  }
  for(auto& joint : m_joints) {
    if(joint->GetConnectionType() == Connection::JointType::Revolute) {
      pair<double, double> r = joint->GetJointLimits(0);
      double t = DRand()*(r.second-r.first)+r.first;
      v.push_back(t);
    }
    else if(joint->GetConnectionType() == Connection::JointType::Spherical) {
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

pair<vector<double>, vector<double>>
ActiveMultiBody::
GetCfgLimits(const shared_ptr<const Boundary>& _bounds) const {
  vector<double> min, max;
  min.reserve(DOF());
  max.reserve(DOF());
  if(m_baseType == FreeBody::BodyType::Planar ||
      m_baseType == FreeBody::BodyType::Volumetric) {
    size_t posDOF = m_baseType == FreeBody::BodyType::Volumetric ? 3 : 2;
    for(size_t i = 0; i < posDOF; i++) {
      pair<double, double> range = _bounds->GetRange(i);
      min.push_back(range.first);
      max.push_back(range.second);
    }
    if(m_baseMovement == FreeBody::MovementType::Rotational) {
      size_t oriDOF = m_baseType == FreeBody::BodyType::Volumetric ? 3 : 1;
      for(size_t i = 0; i < oriDOF; i++) {
        min.push_back(-1);
        max.push_back(1);
      }
    }
  }
  for(auto& joint : m_joints) {
    if(joint->GetConnectionType() == Connection::JointType::Revolute) {
      pair<double, double> r = joint->GetJointLimits(0);
      min.push_back(r.first);
      max.push_back(r.second);
    }
    else if(joint->GetConnectionType() == Connection::JointType::Spherical) {
      pair<double, double> r = joint->GetJointLimits(0);
      min.push_back(r.first);
      max.push_back(r.second);
      r = joint->GetJointLimits(1);
      min.push_back(r.first);
      max.push_back(r.second);
    }
  }
  return make_pair(move(min), move(max));
}

bool
ActiveMultiBody::
InCSpace(const vector<double>& _cfg, shared_ptr<Boundary>& _b) {
  size_t index = 0;
  if(m_baseType != FreeBody::BodyType::Fixed) {
    Vector3d p;
    p[0] = _cfg[index];
    p[1] = _cfg[index+1];
    index+=2;
    if(m_baseType == FreeBody::BodyType::Volumetric) {
      p[2] = _cfg[index];
      index++;
    }
    if(!_b->InBoundary(p))
      return false;
    if(m_baseMovement == FreeBody::MovementType::Rotational) {
      if(m_baseType == FreeBody::BodyType::Planar) {
        if(fabs(_cfg[index]) > 1)
          return false;
        index++;
      }
      else {
        for(size_t i = 0; i<3; ++i) {
          if(fabs(_cfg[index]) > 1)
            return false;
          index++;
        }
      }
    }
  }
  for(auto& joint : m_joints) {
    if(joint->GetConnectionType() != Connection::JointType::NonActuated) {
      if(_cfg[index] < joint->GetJointLimits(0).first ||
          _cfg[index] > joint->GetJointLimits(0).second)
        return false;
      index++;
      if(joint->GetConnectionType() == Connection::JointType::Spherical) {
        if(_cfg[index] < joint->GetJointLimits(1).first ||
            _cfg[index] > joint->GetJointLimits(1).second)
          return false;
        index++;
      }
    }
  }
  return true;
}

void
ActiveMultiBody::
PolygonalApproximation(vector<Vector3d>& _result) {
  _result.clear();

  size_t nfree = NumFreeBody();

  //If rigid body then return the line between the first 4 vertices and the
  //second 4 vertices of the world bounding box
  if(nfree == 1) {
    GMSPolyhedron& bbox = GetFreeBody(0)->GetWorldBoundingBox();

    Vector3d joint;
    for(size_t i = 0; i < 4; ++i)
      joint += bbox.m_vertexList[i];
    joint /= 4;
    _result.push_back(joint);

    joint(0, 0, 0);
    for(size_t i = 4; i < 8; ++i)
      joint += bbox.m_vertexList[i];
    joint /= 4;
    _result.push_back(joint);
  }
  else {
    for(size_t i = 0; i < nfree-1; ++i) {
      GMSPolyhedron& bbox1 = GetFreeBody(i)->GetWorldBoundingBox();
      GMSPolyhedron& bbox2 = GetFreeBody(i)->GetForwardConnection(0).GetNextBody()->GetWorldBoundingBox();

      //find the four closest pairs of points between bbox1 and bbox2
      vector<Vector3d>& vertices1 = bbox1.m_vertexList;
      vector<Vector3d>& vertices2 = bbox2.m_vertexList;
      typedef tuple<size_t, size_t, double> VertexIndexDistance;
      vector<VertexIndexDistance> closestDists;
      for(int num = 0; num < 4; ++num) {
        vector<VertexIndexDistance> distances;
        for(size_t j=0; j < vertices1.size(); ++j)
          for(size_t k=0; k < vertices2.size(); ++k)
            distances.push_back(VertexIndexDistance(j, k, (vertices1[j] - vertices2[k]).norm()));
        closestDists.push_back(*min_element(distances.begin(), distances.end(),
              [](const VertexIndexDistance& _v1, const VertexIndexDistance& _v2) {
              return get<2>(_v1) < get<2>(_v2);
              }));
      }

      //if first body in linkage then
      //add the endpoint of linkage 1 that is not closest to linkage 2
      if(i == 0) {
        Vector3d otherJoint;
        int num = 0;
        for(size_t k = 0; num < 4 && k < bbox1.m_vertexList.size(); ++k)
          if(find_if(closestDists.begin(), closestDists.end(),
                [&](const VertexIndexDistance& _v) {
                return get<0>(_v) == k;
                }) == closestDists.end()) {
            otherJoint += bbox1.m_vertexList[k];
            num++;
          }
        otherJoint /= num;
        _result.push_back(otherJoint);
      }

      //compute the joint as the closest 4 vertices from linkage 1 and linkage 2
      Vector3d joint;
      for(size_t k = 0; k < 4; ++k) {
        joint += bbox1.m_vertexList[get<0>(closestDists[k])];
        joint += bbox2.m_vertexList[get<1>(closestDists[k])];
      }
      joint /= 8;
      _result.push_back(joint);

      //if last body in linkage then
      //add endpoint of linkage 2 that is not closest to linkage 1
      if(i == nfree - 2) {
        Vector3d otherJoint;
        int num = 0;
        for(size_t k = 0; num < 4 && k < bbox2.m_vertexList.size(); ++k)
          if(find_if(closestDists.begin(), closestDists.end(),
                [&](const VertexIndexDistance& _v) {
                return get<1>(_v) == k;
                }) == closestDists.end()) {
            otherJoint += bbox2.m_vertexList[k];
            num++;
          }
        otherJoint = otherJoint / num;

        _result.push_back(otherJoint);
      }
    }
  }
}

void
ActiveMultiBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {

  size_t bodyCount = ReadField<size_t>(_is, _cbs,
      "Failed reading body count.");

  m_baseIndex = -1;
  for(size_t i=0; i < bodyCount && _is; ++i) {
    //read the free body
    shared_ptr<FreeBody> free(new FreeBody(this, i));
    free->Read(_is, _cbs);

    //add object to multibody
    AddBody(free);

    if(free->IsBase() && m_baseIndex == size_t(-1)) {
      m_baseIndex = i;
      m_baseBody = free;
      m_baseType = free->GetBodyType();
      m_baseMovement = free->GetMovementType();
    }
  }

  if(m_baseIndex == size_t(-1))
    throw ParseException(_cbs.Where(), "Active body has no base.");

  //get connection info
  string connectionTag = ReadFieldString(_is, _cbs,
      "Failed reading connections tag.");
    if(connectionTag == "ADJACENCIES") {
      size_t closuresCount = ReadField<size_t>(_is, _cbs,
          "Failed reading number of closing loop connections.");
      for(size_t m = 0; m < closuresCount && _is; ++m) {
        //read body indices
        int firstI = ReadField<int>(_is, _cbs, "Failed reading first closing loop index");
        int secondI = ReadField<int>(_is, _cbs, "Failed reading second closing loop index");
        for(auto& Body: m_freeBody) {
          Body->SetClosureIndices(make_pair(firstI, secondI));
        }
      }
      connectionTag = ReadFieldString(_is, _cbs, "Failed reading connections tag.");
    }
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

  FindMultiBodyInfo();
}

void
ActiveMultiBody::
Write(ostream & _os) {
  _os << GetTagFromMultiBodyType(m_multiBodyType) << endl;
  _os << m_freeBody.size() << endl;
  for(auto& body : m_freeBody)
    _os << *body << endl;
  _os << "Connections" << endl;
  size_t numConnection = 0;
  for(auto& body : m_freeBody)
    numConnection += body->ForwardConnectionCount();
  _os << numConnection << endl;
  for(auto& body : m_freeBody)
    for(size_t j = 0; j < body->ForwardConnectionCount(); j++)
      _os << body->GetForwardConnection(j);
}

void
ActiveMultiBody::
AddBody(const shared_ptr<FreeBody>& _body) {
  m_freeBody.push_back(_body);
  MultiBody::AddBody(_body);
}

void
ActiveMultiBody::
SetBaseBody(const shared_ptr<FreeBody>& _body) {
  m_baseBody = _body;
  m_baseIndex = 0;
  m_baseType = _body->GetBodyType();
  m_baseMovement = _body->GetMovementType();
}
