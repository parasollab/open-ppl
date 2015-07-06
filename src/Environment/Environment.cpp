#include "Environment.h"

#include "ActiveMultiBody.h"
#include "BoundingBox.h"
#include "BoundingSphere.h"
#include "FreeBody.h"
#include "NonHolonomicMultiBody.h"
#include "StaticMultiBody.h"
#include "SurfaceMultiBody.h"
#include "Cfg/Cfg.h"
#include "MPProblem/MPProblemBase.h"

Environment::
Environment() :
  m_filename(""),
  m_saveDofs(false),
  m_positionRes(0.05),
  m_orientationRes(0.05),
  m_rdRes(0.05),
  m_timeRes(0.01) {
  }

Environment::
Environment(XMLNode& _node) {

  m_filename = _node.Read("filename", true, "", "env filename");
  m_saveDofs = _node.Read("saveDofs", false, false, "save DoF flag");
  m_positionRes = _node.Read("positionRes", false, -1.0, 0.0, MAX_DBL,
      "position resolution");
  m_positionResFactor = _node.Read("positionResFactor", false,
      0.05, 0.0, MAX_DBL, "position resolution factor");
  m_orientationRes = _node.Read("orientationRes", false, 0.05, 0.0, MAX_DBL,
      "orientation resolution");
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  m_rdRes = _node.Read("rdRes", false, .005, .00001, MAX_DBL,
      "reachable distance resolution");
#else
  m_rdRes = 0.05;
#endif
  m_timeRes = _node.Read("timeRes", false, 0.01, 0.0, MAX_DBL,
      "Time resolution");
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

  m_robots.clear();
  m_obstacles.clear();
  m_surfaces.clear();

  // open file
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

    MultiBody::MultiBodyType bodyType =
      MultiBody::GetMultiBodyTypeFromTag(multibodyType, cbs.Where());

    switch(bodyType) {
      case MultiBody::MultiBodyType::Active:
        {
          shared_ptr<ActiveMultiBody> mb(new ActiveMultiBody());
          mb->Read(ifs, cbs);
          m_robots.push_back(mb);
          break;
        }
      case MultiBody::MultiBodyType::NonHolonomic:
        {
          shared_ptr<ActiveMultiBody> mb(new NonHolonomicMultiBody());
          mb->Read(ifs, cbs);
          m_robots.push_back(mb);
          break;
        }
      case MultiBody::MultiBodyType::Internal:
      case MultiBody::MultiBodyType::Passive:
        {
          shared_ptr<StaticMultiBody> mb(new StaticMultiBody(bodyType));
          mb->Read(ifs, cbs);
          m_obstacles.push_back(mb);
          break;
        }
      case MultiBody::MultiBodyType::Surface:
        {
          shared_ptr<SurfaceMultiBody> mb(new SurfaceMultiBody());
          mb->Read(ifs, cbs);
          m_surfaces.push_back(mb);
          break;
        }
    }
  }

  if(m_robots.empty())
    throw ParseException(cbs.Where(),
        "No active multibodies in the environment.");

  size_t size = m_robots.size();
  Cfg::SetSize(size);
#ifdef PMPCfgMultiRobot
  CfgMultiRobot::m_numRobot = size;
#endif
  for(size_t i = 0; i < size; ++i) {
    if(m_saveDofs) {
      ofstream dofFile(m_filename + "." + ::to_string(i) + ".dof");
      m_robots[i]->InitializeDOFs(&dofFile);
    }
    else
      m_robots[i]->InitializeDOFs();

    Cfg::InitRobots(m_robots[i], i);
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

void
Environment::
Write(ostream & _os) {
  WriteBoundary(_os);
  _os << endl << endl
    << "MultiBodies" << endl
    << m_robots.size() + m_obstacles.size() + m_surfaces.size()
    << endl << endl;
  for(const auto& body : m_robots) {
    body->Write(_os);
    _os << endl;
  }
  for(const auto& body : m_obstacles) {
    body->Write(_os);
    _os << endl;
  }
  for(const auto& body : m_surfaces) {
    body->Write(_os);
    _os << endl;
  }
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

  double robotRadius = m_robots[_robotIndex]->GetBoundingSphereRadius();
  _d += robotRadius;

  for(auto& body : m_obstacles) {
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

  double robotRadius = m_robots[_robotIndex]->GetBoundingSphereRadius();
  _d += robotRadius;

  vector<pair<double, double> > originBBX(3);
  originBBX[0] = GetBoundary()->GetRange(0);
  originBBX[1] = GetBoundary()->GetRange(1);
  originBBX[2] = GetBoundary()->GetRange(2);

  m_boundary->ResetBoundary(originBBX, _d);
}

shared_ptr<ActiveMultiBody>
Environment::
GetRobot(size_t _index) const {
  if(_index < 0 || _index >= m_robots.size())
    throw RunTimeException(WHERE,
        "Cannot access ActiveBody '" + ::to_string(_index) + "'.");
  return m_robots[_index];
}

shared_ptr<StaticMultiBody>
Environment::
GetObstacle(size_t _index) const {
  if(_index < 0 || _index >= m_obstacles.size())
    throw RunTimeException(WHERE,
        "Cannot access StaticBody '" + ::to_string(_index) + "'.");
  return m_obstacles[_index];
}

shared_ptr<SurfaceMultiBody>
Environment::
GetSurface(size_t _index) const {
  if(_index < 0 || _index >= m_surfaces.size())
    throw RunTimeException(WHERE,
        "Cannot access Navigable Surface '" + ::to_string(_index) + "'.");
  return m_surfaces[_index];
}

shared_ptr<StaticMultiBody>
Environment::
GetRandomObstacle() const {
  if(m_obstacles.empty())
    throw RunTimeException(WHERE, "No static multibodies to select from.");

  size_t rIndex = LRand() % m_obstacles.size();
  return m_obstacles[rIndex];
}

ssize_t
Environment::
GetRandomSurfaceIndex() {
  return LRand() % (m_surfaces.size() + 1) - 1;
}

size_t
Environment::
AddObstacle(const string& _modelFileName, const Transformation& _where,
    const vector<CollisionDetectionMethod*>& _cdMethods) {
  shared_ptr<StaticMultiBody> mb(
      new StaticMultiBody(MultiBody::MultiBodyType::Passive));
  mb->Initialize(_modelFileName, _where);

  for(const auto& cd : _cdMethods)
    mb->BuildCDStructure(cd);

  m_obstacles.push_back(mb);

  return m_obstacles.size()-1;
}

void
Environment::
RemoveObstacle(size_t position) {
  if(position < m_obstacles.size())
    m_obstacles.erase(m_obstacles.begin()+position);
  else
    cerr << "Environment::RemoveObstacleAt Warning: unable to remove obst at "
      "position " << position << endl;
}

void
Environment::
BuildCDStructure(CollisionDetectionMethod* _cdMethod) {
  for(auto& body : m_robots)
    body->BuildCDStructure(_cdMethod);
  for(auto& body : m_obstacles)
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
WriteBoundary(ostream& _os) {
  _os << "Boundary " << m_boundary->Type() << " ";
  m_boundary->Write(_os);
}

void
Environment::
ComputeResolution() {
  double bodiesMinSpan = numeric_limits<double>::max();
  for(auto& body : m_robots) {
    bodiesMinSpan = min(bodiesMinSpan, body->GetMaxAxisRange());
  }

  for(auto& body : m_obstacles) {
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
  return m_robots[activeBodyIndex]->InCSpace(_cfg.GetData(), _b);
}

bool
Environment::
InWSpace(const Cfg& _cfg, shared_ptr<Boundary> _b) {

  shared_ptr<ActiveMultiBody> robot = m_robots[_cfg.GetRobotIndex()];

  if(_b->GetClearance(_cfg.GetRobotCenterPosition()) < robot->GetBoundingSphereRadius()) { //faster, loose check
    // Robot is close to wall, have a strict check.
    _cfg.ConfigEnvironment(); // Config the robot in the environment.

    //check each part of the robot multibody for being inside of the boundary
    for(size_t m = 0; m < robot->NumFreeBody(); ++m) {

      typedef vector<Vector3d>::const_iterator VIT;

      Transformation& worldTransformation = robot->GetFreeBody(m)->WorldTransformation();

      //first check just the boundary of the polyhedron
      GMSPolyhedron& bbPoly = robot->GetFreeBody(m)->GetBoundingBoxPolyhedron();
      bool bcheck = true;
      for(const auto& v : bbPoly.m_vertexList) {
        if(!_b->InBoundary(worldTransformation * v)) {
          bcheck = false;
          break;
        }
      }

      //boundary of polyhedron is inside the boundary thus the whole geometry is
      if(bcheck)
        continue;

      //the boundary intersected. Now check the geometry itself.
      GMSPolyhedron& poly = robot->GetFreeBody(m)->GetPolyhedron();
      for(const auto& v : poly.m_vertexList)
        if(!_b->InBoundary(worldTransformation * v))
          return false;
    }
  }
  return true;
}

