#include "Environment.h"

#include "Cfg/Cfg.h"
#include "MPProblem/BoundingBox.h"
#include "MPProblem/BoundingSphere.h"
#include "MPProblem/MPProblemBase.h"
#include "MPProblem/Geometry/ActiveMultiBody.h"
#include "MPProblem/Geometry/FreeBody.h"
#include "MPProblem/Geometry/StaticMultiBody.h"
#include "MPProblem/Geometry/SurfaceMultiBody.h"

#define ENV_RES_DEFAULT 0.05

Environment::
Environment() :
  m_filename(""),
  m_saveDofs(false),
  m_positionRes(ENV_RES_DEFAULT),
  m_orientationRes(ENV_RES_DEFAULT),
  m_rdRes(ENV_RES_DEFAULT) {
  }

Environment::
Environment(XMLNode& _node) {

  m_filename = _node.Read("filename", true, "", "env filename");
  m_saveDofs = _node.Read("saveDofs", false, false, "save DoF flag");
  m_positionRes = _node.Read("positionRes", false, -1.0, 0.0, MAX_DBL, "position resolution");
  m_positionResFactor = _node.Read("positionResFactor", false, 0.05, 0.0, MAX_DBL, "position resolution factor");
  m_orientationRes = _node.Read("orientationRes", false, 0.05, 0.0, MAX_DBL, "orientation resolution");
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  m_rdRes = _node.Read("rdRes", false, .005, .00001, MAX_DBL, "reachable distance resolution");
#else
  m_rdRes = ENV_RES_DEFAULT;
#endif
  m_filename = MPProblemBase::GetPath(m_filename);
  Read(m_filename);
}

Environment::
~Environment() {}

void
Environment::
Read(string _filename) {

  if(!FileExists(_filename))
    throw ParseException(_filename, "File does not exist");

  m_filename = _filename;
  size_t sl = m_filename.rfind('/');
  m_modelDataDir = m_filename.substr(0, sl == string::npos ? 0 : sl);
  Body::m_modelDataDir = m_modelDataDir + "/";

  m_activeBodies.clear();
  m_obstacleBodies.clear();
  m_navigableSurfaces.clear();

  // open file
  //ifstream ifs(_filename.c_str());
  CountingStreamBuffer cbs(_filename);
  istream ifs(&cbs);

  //read boundary
  ReadBoundary(ifs, cbs);

  //read number of multibodies
  string mbds = ReadFieldString(ifs, cbs, "Failed reading multibodies tag.");
  if(mbds != "MULTIBODIES")
    throw ParseException(cbs.Where(),
        "Unknown multibodies tag '" + mbds + "'. Should read 'Multibodies'.");

  size_t multibodyCount = ReadField<size_t>(ifs, cbs,
      "Failed reading number of multibodies.");

  //parse and construct each multibody
  for(size_t m = 0; m < multibodyCount && ifs; ++m) {

    string multibodyType = ReadFieldString(ifs, cbs,
        "Failed reading multibody type."
        " Options are: active, passive, internal, or surface.");

    MultiBody::BodyType bodyType =
      MultiBody::GetBodyTypeFromTag(multibodyType, cbs.Where());

    switch(bodyType) {
      case MultiBody::BodyType::Active:
        {
          shared_ptr<ActiveMultiBody> mb(new ActiveMultiBody());
          mb->SetBodyType(bodyType);
          mb->Read(ifs, cbs);
          m_activeBodies.push_back(mb);
          break;
        }
      case MultiBody::BodyType::Internal:
      case MultiBody::BodyType::Passive:
        {
          shared_ptr<StaticMultiBody> mb(new StaticMultiBody());
          mb->SetBodyType(bodyType);
          mb->Read(ifs, cbs);
          m_obstacleBodies.push_back(mb);
          break;
        }
      case MultiBody::BodyType::Surface:
        {
          shared_ptr<SurfaceMultiBody> mb(new SurfaceMultiBody());
          mb->SetBodyType(bodyType);
          mb->Read(ifs, cbs);
          m_navigableSurfaces.push_back(mb);
          break;
        }
    }
  }

  if(m_activeBodies.empty())
    throw ParseException(cbs.Where(),
        "No active multibodies in the environment.");

  size_t size = m_activeBodies.size();
  Cfg::SetSize(size);
#ifdef PMPCfgMultiRobot
  CfgMultiRobot::m_numRobot = size;
#endif
  for(size_t i = 0; i < size; ++i) {
    if(m_saveDofs) {
      ofstream dofFile(m_filename + "." +
          ::to_string(i) + ".dof");
      m_activeBodies[i]->InitializeDOFs(&dofFile);
    }
    else
      m_activeBodies[i]->InitializeDOFs();

    Cfg::InitRobots(m_activeBodies[i], i);
  }

  ComputeResolution();
}

void
Environment::
Print(ostream& _os) const {
  _os << "Environment" << endl;
  _os << "\tpositionRes::" << m_positionRes << endl;
  _os << "\torientationRes::" << m_orientationRes << endl;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  _os << "\trdRes::" << m_rdRes << endl;
#endif
  _os << "\tboundary::" << *m_boundary << endl;
}

////////////////////////////////////////////////////////////////////////////////
/// @todo This will not output a readable env format.
void
Environment::
Write(ostream & _os) {
}

bool
Environment::
InBounds(const Cfg& _cfg, shared_ptr<Boundary> _b) {
  if(InCSpace(_cfg, _b))
    if(InWSpace(_cfg, _b))
      return true;
  return false;
}

bool
Environment::
InBounds(const CfgMultiRobot& _cfg, shared_ptr<Boundary> _b) {
  const vector<Cfg> c = _cfg.GetRobotsCollect();
  for(auto& cfg : c)
    if(!InBounds(cfg, _b))
      return false;
  return true;
}

void
Environment::
ResetBoundary(double _d, size_t _robotIndex) {

  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = numeric_limits<double>::max();
  maxx = maxy = maxz = -numeric_limits<double>::max();

  double robotRadius = GetActiveBody(_robotIndex)->GetBoundingSphereRadius();
  _d += robotRadius;

  for(auto& body : m_obstacleBodies) {
    body->FindBoundingBox();
    const double* tmp = body->GetBoundingBox();
    minx = min(minx, tmp[0]);  maxx = max(maxx, tmp[1]);
    miny = min(miny, tmp[2]);  maxy = max(maxy, tmp[3]);
    minz = min(minz, tmp[4]);  maxz = max(maxz, tmp[5]);
  }

  vector<pair<double, double> > obstBBX(3);
  obstBBX[0] = make_pair(minx, maxx);
  obstBBX[1] = make_pair(miny, maxy);
  obstBBX[2] = make_pair(minz, maxz);

  m_boundary->ResetBoundary(obstBBX, _d);
}

void
Environment::
ExpandBoundary(double _d, size_t _robotIndex) {

  double robotRadius = GetActiveBody(_robotIndex)->GetBoundingSphereRadius();
  _d += robotRadius;

  vector<pair<double, double> > originBBX(3);
  originBBX[0] = GetBoundary()->GetRange(0);
  originBBX[1] = GetBoundary()->GetRange(1);
  originBBX[2] = GetBoundary()->GetRange(2);

  m_boundary->ResetBoundary(originBBX, _d);
}

shared_ptr<ActiveMultiBody>
Environment::
GetActiveBody(size_t _index) const {
  if(_index < 0 || _index >= m_activeBodies.size())
    throw RunTimeException(WHERE,
        "Cannot access ActiveBody '" + ::to_string(_index) + "'.");
  return m_activeBodies[_index];
}

shared_ptr<StaticMultiBody>
Environment::
GetStaticBody(size_t _index) const {
  if(_index < 0 || _index >= m_obstacleBodies.size())
    throw RunTimeException(WHERE,
        "Cannot access StaticBody '" + ::to_string(_index) + "'.");
  return m_obstacleBodies[_index];
}

shared_ptr<SurfaceMultiBody>
Environment::
GetNavigableSurface(size_t _index) const {
  if(_index < 0 || _index >= m_navigableSurfaces.size())
    throw RunTimeException(WHERE,
        "Cannot access Navigable Surface '" + ::to_string(_index) + "'.");
  return m_navigableSurfaces[_index];
}

shared_ptr<MultiBody>
Environment::
GetRandomObstacle() const {
  if(m_obstacleBodies.empty())
    throw RunTimeException(WHERE, "No static multibodies to select from.");

  size_t rIndex = LRand() % m_obstacleBodies.size();
  return m_obstacleBodies[rIndex];
}

ssize_t
Environment::GetRandomNavigableSurfaceIndex() {
  size_t numSurfaces = GetNavigableSurfacesCount();
  ssize_t rindex = LRand() % (numSurfaces+1) - 1;
  return rindex;
}

/*
int
Environment::
AddObstacle(string _modelFileName, const Transformation& _where,
const vector<cd_predefined>& _cdTypes) {
  shared_ptr<MultiBody> mb(new MultiBody());

  mb->Initialize(_modelFileName, _where);

  for(vector<cd_predefined>::const_iterator cdIter = _cdTypes.begin(); cdIter != _cdTypes.end(); ++cdIter)
    mb->buildCDstructure(*cdIter);

  m_obstacleBodies.push_back(mb);

  return m_obstacleBodies.size()-1;
}

void
Environment::
RemoveObstacleAt(size_t position) {
  if (position < m_obstacleBodies.size()) {
    shared_ptr<MultiBody> mb = m_obstacleBodies.at(position);

    m_obstacleBodies.erase(m_obstacleBodies.begin()+position);
  }
  else
    cerr << "Environment::RemoveObstacleAt Warning: unable to remove obst at position " << position << endl;
}
*/

void
Environment::
BuildCDStructure(CollisionDetectionMethod* _cdMethod) {
  for(auto& body : m_activeBodies)
    body->BuildCDStructure(_cdMethod);
  for(auto& body : m_obstacleBodies)
    body->BuildCDStructure(_cdMethod);
}

void
Environment::
ReadBoundary(istream& _is, CountingStreamBuffer& _cbs) {
  string bndry = ReadFieldString(_is, _cbs, "Failed reading boundary tag.");
  if(bndry != "BOUNDARY")
    throw ParseException(_cbs.Where(),
        "Unknown boundary tag '" + bndry + "'. Should read 'Boundary'.");

  string btype = ReadFieldString(_is, _cbs,
      "Failed reading boundary type. Options are: box or sphere.");
  if(btype == "BOX")
    m_boundary = shared_ptr<BoundingBox>(new BoundingBox());
  else if(btype == "SPHERE")
    m_boundary = shared_ptr<BoundingSphere>(new BoundingSphere());
  else
    throw ParseException(_cbs.Where(), "Unknown boundary type '" + btype +
        "'. Options are: box or sphere.");

  m_boundary->Read(_is, _cbs);
}

void
Environment::
ComputeResolution() {
  double bodiesMinSpan = numeric_limits<double>::max();
  for(auto& body : m_activeBodies) {
    body->FindBoundingBox();
    bodiesMinSpan = min(bodiesMinSpan, body->GetMaxAxisRange());
  }

  for(auto& body : m_obstacleBodies) {
    body->FindBoundingBox();
    bodiesMinSpan = min(bodiesMinSpan, body->GetMaxAxisRange());
  }

  // Set to XML input resolution if specified, else compute resolution factor
  if(m_positionRes < 0)
    m_positionRes = bodiesMinSpan * m_positionResFactor;

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  //make sure to calculate the rdRes based upon the DOF of the robot
  m_rdRes *= Cfg::GetNumOfJoints();
#endif
}

bool
Environment::
InCSpace(const Cfg& _cfg, shared_ptr<Boundary> _b) {
  size_t activeBodyIndex = _cfg.GetRobotIndex();
  size_t index = 0;
  shared_ptr<ActiveMultiBody>& rit = m_activeBodies[activeBodyIndex];
  //typedef vector<Robot>::iterator RIT;
  //for(RIT rit = m_activeBodies[activeBodyIndex]->m_robots.begin();
  //    rit != m_activeBodies[activeBodyIndex]->m_robots.end(); rit++) {
    if(rit->m_baseType != Body::FIXED) {
      Vector3d p;
      p[0] = _cfg[index];
      p[1] = _cfg[index+1];
      index+=2;
      if(rit->m_baseType == Body::VOLUMETRIC) {
        p[2] = _cfg[index];
        index++;
      }
      if(!_b->InBoundary(p))
        return false;
      if(rit->m_baseMovement == Body::ROTATIONAL) {
        if(rit->m_baseType == Body::PLANAR) {
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
    typedef ActiveMultiBody::JointMap::iterator MIT;
    for(auto& joint : rit->m_joints) {
      if(joint->GetConnectionType() != Connection::NONACTUATED) {
        if(_cfg[index] < joint->GetJointLimits(0).first || _cfg[index] > joint->GetJointLimits(0).second)
          return false;
        index++;
        if(joint->GetConnectionType() == Connection::SPHERICAL) {
          if(_cfg[index] < joint->GetJointLimits(1).first || _cfg[index] > joint->GetJointLimits(1).second)
            return false;
          index++;
        }
      }
    }
  //}
  return true;
}

bool
Environment::
InWSpace(const Cfg& _cfg, shared_ptr<Boundary> _b) {

  shared_ptr<ActiveMultiBody> robot = m_activeBodies[_cfg.GetRobotIndex()];

  if(_b->GetClearance(_cfg.GetRobotCenterPosition()) < robot->GetBoundingSphereRadius()) { //faster, loose check
    // Robot is close to wall, have a strict check.
    _cfg.ConfigEnvironment(); // Config the robot in the environment.

    //check each part of the robot multibody for being inside of the boundary
    for(size_t m = 0; m < robot->GetFreeBodyCount(); ++m) {

      typedef vector<Vector3d>::const_iterator VIT;

      Transformation& worldTransformation = robot->GetFreeBody(m)->WorldTransformation();

      //first check just the boundary of the polyhedron
      GMSPolyhedron &bbPoly = robot->GetFreeBody(m)->GetBoundingBoxPolyhedron();
      bool bcheck = true;
      for(VIT v = bbPoly.m_vertexList.begin(); v != bbPoly.m_vertexList.end(); ++v) {
        if(!_b->InBoundary(worldTransformation * (*v))) {
          bcheck = false;
          break;
        }
      }

      //boundary of polyhedron is inside the boundary thus the whole geometry is
      if(bcheck)
        continue;

      //the boundary intersected. Now check the geometry itself.
      GMSPolyhedron &poly = robot->GetFreeBody(m)->GetPolyhedron();
      for(VIT v = poly.m_vertexList.begin(); v != poly.m_vertexList.end(); ++v)
        if(!_b->InBoundary(worldTransformation * (*v)))
          return false;
    }
  }
  return true;
}

