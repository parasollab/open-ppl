#include "ActiveMultiBody.h"

#include "FreeBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "ConfigurationSpace/Cfg.h"


/*------------------------------ Construction --------------------------------*/

ActiveMultiBody::
ActiveMultiBody() : MultiBody() {
  m_multiBodyType = MultiBodyType::Active;
}

/*----------------------------- MultiBody Info -------------------------------*/

const Vector3d&
ActiveMultiBody::
GetCenterOfMass() const {
  throw RunTimeException(WHERE, "There is an error in the center of mass "
      "computation for active multibodies - the COM is not updated as the a.m.b."
      " changes configuration. Please correct before using.");
  return MultiBody::GetCenterOfMass();
}


size_t
ActiveMultiBody::
NumFreeBody() const {
  return m_freeBody.size();
}


const FreeBody*
ActiveMultiBody::
GetFreeBody(const size_t _index) const {
  if(_index >= m_freeBody.size())
    throw RunTimeException(WHERE,
        "Cannot access FreeBody(" + ::to_string(_index) + ").");
  return m_freeBody[_index].get();
}


FreeBody*
ActiveMultiBody::
GetFreeBody(const size_t _index) {
  return const_cast<FreeBody*>(const_cast<const ActiveMultiBody*>(this)->
      GetFreeBody(_index));
}

/*------------------------------ Robot Info ----------------------------------*/

void
ActiveMultiBody::
InitializeDOFs(const Boundary* _b, ostream* _os) {
  m_dofTypes.clear();
  m_dofInfo.clear();

  size_t dof = 0;

  if(_os) {
    *_os << "DoF List: " << endl;
    *_os << "\tRobot with base index " << m_baseIndex;
    *_os << " (" << m_baseBody->GetFileName() << "):" << endl;
  }

  if(m_baseType == FreeBody::BodyType::Planar) {
    m_dofTypes.push_back(DofType::Positional);
    m_dofTypes.push_back(DofType::Positional);
    m_dofInfo.push_back(DofInfo("Base X Translation ",
          _b->GetRange(0).first, _b->GetRange(0).second));
    m_dofInfo.push_back(DofInfo("Base Y Translation ",
          _b->GetRange(1).first, _b->GetRange(1).second));

    if(_os) {
      *_os << "\t\t" << dof++ << ": X position" << endl;
      *_os << "\t\t" << dof++ << ": Y position" << endl;
    }

    if(m_baseMovement == FreeBody::MovementType::Rotational) {
      m_dofTypes.push_back(DofType::Rotational);
      m_dofInfo.push_back(DofInfo("Base Rotation ", -1.0, 1.0));

      if(_os)
        *_os << "\t\t" << dof++ << ": Rotation about Z" << endl;
    }
  }
  else if(m_baseType == FreeBody::BodyType::Volumetric) {
    m_dofTypes.push_back(DofType::Positional);
    m_dofTypes.push_back(DofType::Positional);
    m_dofTypes.push_back(DofType::Positional);
    m_dofInfo.push_back(DofInfo("Base X Translation ",
          _b->GetRange(0).first, _b->GetRange(0).second));
    m_dofInfo.push_back(DofInfo("Base Y Translation ",
          _b->GetRange(1).first, _b->GetRange(1).second));
    m_dofInfo.push_back(DofInfo("Base Z Translation ",
          _b->GetRange(2).first, _b->GetRange(2).second));

    if(_os) {
      *_os << "\t\t" << dof++ << ": X position" << endl;
      *_os << "\t\t" << dof++ << ": Y position" << endl;
      *_os << "\t\t" << dof++ << ": Z position" << endl;
    }
    if(m_baseMovement == FreeBody::MovementType::Rotational) {
      m_dofTypes.push_back(DofType::Rotational);
      m_dofTypes.push_back(DofType::Rotational);
      m_dofTypes.push_back(DofType::Rotational);
      m_dofInfo.push_back(DofInfo("Base X Rotation ", -1.0, 1.0));
      m_dofInfo.push_back(DofInfo("Base Y Rotation ", -1.0, 1.0));
      m_dofInfo.push_back(DofInfo("Base Z Rotation ", -1.0, 1.0));

      if(_os) {
        *_os << "\t\t" << dof++ << ": Rotation about X" << endl;
        *_os << "\t\t" << dof++ << ": Rotation about Y" << endl;
        *_os << "\t\t" << dof++ << ": Rotation about Z" << endl;
      }
    }
  }

  for(auto& joint : m_joints) {
    switch(joint->GetConnectionType()) {
      case Connection::JointType::Revolute:
        m_dofTypes.push_back(DofType::Joint);
        m_dofInfo.push_back(DofInfo("Revolute Joint " +
              ::to_string(joint->GetGlobalIndex()) + " Angle",
              joint->GetJointLimits(0).first, joint->GetJointLimits(0).second));

        if(_os) {
          *_os << "\t\t" << dof++ << ": ";
          *_os << "Rotational joint from body " << joint->GetPreviousBodyIndex();
          *_os << " (" << joint->GetPreviousBody()->GetFileName() << ")";
          *_os << " to body " << joint->GetNextBodyIndex();
          *_os << " (" << joint->GetNextBody()->GetFileName() << ")" << endl;
        }
        break;

      case Connection::JointType::Spherical:
        m_dofTypes.push_back(DofType::Joint);
        m_dofTypes.push_back(DofType::Joint);
        m_dofInfo.push_back(DofInfo("Spherical Joint " +
              ::to_string(joint->GetGlobalIndex()) + " Angle 1",
              joint->GetJointLimits(0).first, joint->GetJointLimits(0).second));
        m_dofInfo.push_back(DofInfo("Spherical Joint " +
              ::to_string(joint->GetGlobalIndex()) + " Angle 2",
              joint->GetJointLimits(1).first, joint->GetJointLimits(1).second));

        if(_os) {
          *_os << "\t\t" << dof++;
          *_os << "/" << dof++ << ": ";
          *_os << "Spherical joint from body " << joint->GetPreviousBodyIndex();
          *_os << " (" << joint->GetPreviousBody()->GetFileName() << ")";
          *_os << " to body " << joint->GetNextBodyIndex();
          *_os << " (" << joint->GetNextBody()->GetFileName() << ")" << endl;
        }
        break;

      case Connection::JointType::NonActuated:
        break;
    }
  }
}


size_t
ActiveMultiBody::
DOF() const noexcept {
  return m_dofTypes.size();
}


size_t
ActiveMultiBody::
PosDOF() const noexcept {
  switch(m_baseType) {
    case FreeBody::BodyType::Planar:
      return 2;
    case FreeBody::BodyType::Volumetric:
      return 3;
    default:
      return 0;
  }
}


size_t
ActiveMultiBody::
OrientationDOF() const noexcept {
  if(m_baseMovement != FreeBody::MovementType::Rotational)
    return 0;
  else
    return m_baseType == FreeBody::BodyType::Volumetric ? 3 : 1;
}


size_t
ActiveMultiBody::
JointDOF() const noexcept {
  return DOF() - PosDOF() - OrientationDOF();
}

/*------------------------- Configuration Methods ----------------------------*/

void
ActiveMultiBody::
Configure(const Cfg& _c) {
  Configure(_c.GetData());
}


void
ActiveMultiBody::
Configure(const vector<double>& _v) {
  int index = 0;

  // Configure the base.
  if(m_baseType != FreeBody::BodyType::Fixed) {
    double x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0;
    x = _v[0];
    y = _v[1];
    index += 2;
    if(m_baseType == FreeBody::BodyType::Volumetric) {
      index++;
      z = _v[2];
    }
    if(m_baseMovement == FreeBody::MovementType::Rotational) {
      if(m_baseType == FreeBody::BodyType::Planar) {
        ++index;
        gamma = _v[2];
      }
      else {
        index += 3;
        alpha = _v[3];
        beta = _v[4];
        gamma = _v[5];
      }
    }

    Transformation t1(Vector3d(x, y, z), Orientation(EulerAngle(gamma * PI,
        beta * PI, alpha * PI)));
    m_baseBody->Configure(t1);
  }

  // Configure the links.
  for(auto& joint : m_joints) {
    // Skip non-actuated joints.
    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    // Get the connection object.
    const size_t jointIndex = joint->GetNextBodyIndex();
    auto& connection = m_freeBody[jointIndex]->GetBackwardConnection(0);
    auto& dh = connection.GetDHParameters();

    // Adjust connection to reflect new configuration.
    dh.m_theta = _v[index++] * PI;
    if(joint->GetConnectionType() == Connection::JointType::Spherical)
      dh.m_alpha = _v[index++] * PI;
  }

  // The base transform has been updated, now update the links.
  UpdateLinks();
}


void
ActiveMultiBody::
Configure(const vector<double>& _v, const vector<double>& _t) {
  int index = 0, t_index = 0;

  // Configure the base.
  if(m_baseType != FreeBody::BodyType::Fixed) {
    double x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0;

    x = _v[0];
    y = _v[1];
    index += 2;
    if(m_baseType == FreeBody::BodyType::Volumetric) {
      index++;
      z = _v[2];
    }
    if(m_baseMovement == FreeBody::MovementType::Rotational) {
      if(m_baseType == FreeBody::BodyType::Planar) {
        ++index;
        gamma = _v[2];
      }
      else {
        index += 3;
        alpha = _v[3];
        beta = _v[4];
        gamma = _v[5];
      }
    }

    Transformation t1(Vector3d(x, y, z), Orientation(EulerAngle(gamma * PI,
        beta * PI, alpha * PI)));
    m_baseBody->Configure(t1);
  }

  // Configure the links.
  for(auto& joint : m_joints) {
    // Skip non-actuated joints.
    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    // Get the connection object's DHParameters.
    const size_t jointIndex = joint->GetNextBodyIndex();
    auto& connection = m_freeBody[jointIndex]->GetBackwardConnection(0);
    auto& dh = connection.GetDHParameters();

    // Adjust connection to reflect new configuration.
    ++index; // Skip the theta value in _v as we will use _t instead.
    dh.m_theta = _t[t_index++];
    if(joint->GetConnectionType() == Connection::JointType::Spherical)
      dh.m_alpha = _v[index++] * PI;
  }

  // The base transform has been updated, now update the links.
  UpdateLinks();
}


void
ActiveMultiBody::
ConfigureRender(const vector<double>& _v) {
  int index = 0;

  // Configure base.
  if(m_baseType != FreeBody::BodyType::Fixed) {
    double x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0;
    x = _v[0];
    y = _v[1];
    index += 2;
    if(m_baseType == FreeBody::BodyType::Volumetric) {
      index++;
      z = _v[2];
    }
    if(m_baseMovement == FreeBody::MovementType::Rotational) {
      if(m_baseType == FreeBody::BodyType::Planar) {
        index++;
        gamma = _v[2];
      }
      else {
        index += 3;
        alpha = _v[3];
        beta = _v[4];
        gamma = _v[5];
      }
    }

    Transformation t1(Vector3d(x, y, z),
        Orientation(EulerAngle(gamma*PI, beta*PI, alpha*PI)));
    m_baseBody->ConfigureRender(t1);
  }

  // Configure links.
  for(auto& joint : m_joints) {
    // Skip non-actuated joints.
    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    // Get connection object.
    const size_t jointIndex = joint->GetNextBodyIndex();
    auto& connection = m_freeBody[jointIndex]->GetBackwardConnection(0);
    auto& dh = connection.GetDHRenderParameters();

    // Adjust connection to reflect new configuration.
    dh.m_theta = _v[index++]*PI;
    if(joint->GetConnectionType() == Connection::JointType::Spherical)
      dh.m_alpha = _v[index++]*PI;
  }

  // configure the robot
  for(auto& body : m_freeBody)
    if(body->ForwardConnectionCount() == 0)  // tree tips: leaves.
      body->GetRenderTransformation();
}


vector<double>
ActiveMultiBody::
GetRandomCfg(const Boundary* const _b) {
  vector<double> v;
  v.reserve(DOF());

  // Sample any positional DOFs.
  if(PosDOF()) {
    Point3d p = _b->GetRandomPoint();
    for(size_t i = 0; i < PosDOF(); ++i)
      v.push_back(p[i]);
  }

  // Sample any rotational DOFs.
  for(size_t i = 0; i < OrientationDOF(); ++i)
    v.push_back(2. * DRand() - 1.);

  // Sample any joint DOFs.
  for(size_t i = 0; i < JointDOF(); ++i)
  for(auto& joint : m_joints) {
    if(joint->GetConnectionType() == Connection::JointType::Revolute) {
      pair<double, double> r = joint->GetJointLimits(0);
      double t = DRand() * (r.second - r.first) + r.first;
      v.push_back(t);
    }
    else if(joint->GetConnectionType() == Connection::JointType::Spherical) {
      pair<double, double> r = joint->GetJointLimits(0);
      double t = DRand() * (r.second - r.first) + r.first;
      r = joint->GetJointLimits(1);
      double a = DRand() * (r.second - r.first) + r.first;
      v.push_back(t);
      v.push_back(a);
    }
  }
  return v;
}


pair<vector<double>, vector<double>>
ActiveMultiBody::
GetCfgLimits(const Boundary* const _b) const {
  vector<double> min, max;
  min.reserve(DOF());
  max.reserve(DOF());

  // Get position limits.
  if(PosDOF()) {
    for(size_t i = 0; i < PosDOF(); ++i) {
      pair<double, double> range = _b->GetRange(i);
      min.push_back(range.first);
      max.push_back(range.second);
    }
  }

  // Get orientation limits.
  if(OrientationDOF()) {
    for(size_t i = 0; i < OrientationDOF(); ++i) {
      min.push_back(-1);
      max.push_back(1);
    }
  }

  // Get joint limits.
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
InCSpace(const vector<double>& _cfg, const Boundary* const _b) {
  size_t index = 0;

  // Check that the reference point lies in the boundary.
  if(PosDOF()) {
    Vector3d p;
    p[0] = _cfg[0];
    p[1] = _cfg[1];
    index += 2;
    if(m_baseType == FreeBody::BodyType::Volumetric) {
      p[2] = _cfg[index];
      index++;
    }
    if(!_b->InBoundary(p))
      return false;
  }

  // Check that all orientation DOF's lie within [-1, 1].
  for(size_t i = 0; i < OrientationDOF(); ++i)
    if(fabs(_cfg[index]) > 1)
      return false;
  index += OrientationDOF();

  // Check that all joint DOF's lie within their limits.
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

  // If we're still here, all the DOF values are OK.
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
    const GMSPolyhedron bbox = GetFreeBody(0)->GetWorldBoundingBox();

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
    for(size_t i = 0; i < nfree - 1; ++i) {
      const GMSPolyhedron bbox1 = m_freeBody[i]->GetWorldBoundingBox();
      const GMSPolyhedron bbox2 = m_freeBody[i]->GetForwardConnection(0).
          GetNextBody()->GetWorldBoundingBox();

      //find the four closest pairs of points between bbox1 and bbox2
      const vector<Vector3d>& vertices1 = bbox1.m_vertexList;
      const vector<Vector3d>& vertices2 = bbox2.m_vertexList;
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

/*--------------------------------- I/O --------------------------------------*/

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
  // Write type.
  _os << GetTagFromMultiBodyType(m_multiBodyType) << endl;

  // Write free body count and free bodies.
  _os << m_freeBody.size() << endl;
  for(auto& body : m_freeBody)
    _os << *body << endl;

  // Write connections.
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

/*--------------------------------- Helpers ----------------------------------*/

void
ActiveMultiBody::
UpdateLinks() {
  for(auto& body : m_freeBody)
    if(!body->IsBase())
      body->MarkDirty();

  /// @TODO I think we should be able to remove this forced recomputation and
  ///       let it resolve lazily, need to test though.
  for(auto& body : m_freeBody)
    if(body->ForwardConnectionCount() == 0)  // tree tips: leaves.
      body->GetWorldTransformation();
}

/*----------------------------------------------------------------------------*/
