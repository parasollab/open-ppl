#include "MPProblem/Environment/Environment.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Geometry/Bodies/NonHolonomicMultiBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"
#include "Geometry/Boundaries/BoundingBox.h"
#include "Geometry/Boundaries/BoundingBox2D.h"
#include "Geometry/Boundaries/BoundingSphere.h"
#include "Geometry/Boundaries/BoundingSphere2D.h"
#include "MPProblem/ConfigurationSpace/Cfg.h"
#include "MPProblem/MPProblemBase.h"


/*------------------------------- Construction -------------------------------*/

Environment::
Environment(XMLNode& _node) {
  m_filename = _node.Read("filename", true, "", "env filename");
  m_saveDofs = _node.Read("saveDofs", false, m_saveDofs, "save DoF flag");
  m_filename = MPProblemBase::GetPath(m_filename);
  Read(m_filename);
}

/*------------------------------------ I/O -----------------------------------*/

void
Environment::
Read(string _filename) {
  if(!FileExists(_filename))
    throw ParseException(_filename, "File does not exist");

  m_filename = _filename;
  size_t sl = m_filename.rfind('/');
  m_modelDataDir = m_filename.substr(0, sl == string::npos ? 0 : sl);
  Body::m_modelDataDir = m_modelDataDir + "/";

  m_obstacles.clear();

  // open file
  CountingStreamBuffer cbs(_filename);
  istream ifs(&cbs);

  //read boundary
  ReadBoundary(ifs, cbs);
  string resolution;
  while((resolution = ReadFieldString(ifs, cbs, "Failed reading resolution tag."))
      != "MULTIBODIES") {
    if(resolution == "POSITIONRES")
      m_positionRes = ReadField<double>(ifs, cbs, "Failed reading Position "
          "resolution\n");
    else if(resolution == "POSITIONRESFACTOR")
      m_positionResFactor = ReadField<double>(ifs, cbs, "Failed reading Position "
          "factor resolution\n");
    else if(resolution == "ORIENTATION")
      m_orientationRes = ReadField<double>(ifs, cbs, "Failed reading "
          "Orientation resolution\n");
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    else if(resolution == "RDRES")
      m_rdRes = ReadField<double>(ifs, cbs, "Failed reading Reachable Distance "
          "resolution");
#endif
#ifdef PMPState
    else if(resolution == "TIMERES")
      m_timeRes = ReadField<double>(ifs, cbs, "Failed reading Time resolution\n");
#endif
    else
      throw ParseException(cbs.Where(), "Unknown resolution tag '" + resolution
          + "'");
  }

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
          /// @TODO: Add support for holonomic dynamic obstacles
          break;
        }
      case MultiBody::MultiBodyType::NonHolonomic:
        {
          /// @TODO: Add support for non-holonomic dynamic obstacles
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
    }
  }
}


void
Environment::
Print(ostream& _os) const {
  _os << "Environment" << endl
      << "\tpositionRes::" << m_positionRes << endl
      << "\torientationRes::" << m_orientationRes << endl
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
      << "\trdRes::" << m_rdRes << endl
#endif
      << "\tboundary::" << *m_boundary << endl;
}


void
Environment::
Write(ostream & _os) {
  WriteBoundary(_os);
  _os << endl << endl
      << "Obstacles" << endl
      << m_obstacles.size()
      << endl << endl;
  for(const auto& body : m_obstacles) {
    body->Write(_os);
    _os << endl;
  }
}

/*-------------------------------- Resolutions -------------------------------*/

void
Environment::
ComputeResolution(vector<ActiveMultiBody*> _robots) {
  double bodiesMinSpan = numeric_limits<double>::max();
  for(auto& body : _robots)
    bodiesMinSpan = min(bodiesMinSpan, body->GetMaxAxisRange());

  for(auto& body : m_obstacles)
    bodiesMinSpan = min(bodiesMinSpan, body->GetMaxAxisRange());

  // Set to XML input resolution if specified, else compute resolution factor
  if(m_positionRes < 0)
    m_positionRes = bodiesMinSpan * m_positionResFactor;

#ifdef PMPState
  State::SetTimeRes(m_timeRes);
#endif
}

/*----------------------------- Boundary Functions ---------------------------*/

void
Environment::
ResetBoundary(double _d, ActiveMultiBody* _robot) {
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = numeric_limits<double>::max();
  maxx = maxy = maxz = -numeric_limits<double>::max();

  double robotRadius = _robot->GetBoundingSphereRadius();
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
ExpandBoundary(double _d, ActiveMultiBody* _robot) {
  double robotRadius = _robot->GetBoundingSphereRadius();
  _d += robotRadius;

  vector<pair<double, double>> originBBX(3);
  originBBX[0] = GetBoundary()->GetRange(0);
  originBBX[1] = GetBoundary()->GetRange(1);
  originBBX[2] = GetBoundary()->GetRange(2);

  m_boundary->ResetBoundary(originBBX, _d);
}

/*---------------------------- Obstacle Functions ----------------------------*/

StaticMultiBody*
Environment::
GetObstacle(size_t _index) const {
  if(_index < 0 || _index >= m_obstacles.size())
    throw RunTimeException(WHERE,
        "Cannot access StaticBody '" + ::to_string(_index) + "'.");
  return m_obstacles[_index].get();
}


StaticMultiBody*
Environment::
GetRandomObstacle() const {
  if(m_obstacles.empty())
    throw RunTimeException(WHERE, "No static multibodies to select from.");

  size_t rIndex = LRand() % m_obstacles.size();
  return m_obstacles[rIndex].get();
}


pair<size_t, shared_ptr<StaticMultiBody>>
Environment::
AddObstacle(const string& _dir, const string& _filename,
    const Transformation& _t) {
  shared_ptr<StaticMultiBody> mb(
      new StaticMultiBody(MultiBody::MultiBodyType::Passive));

  mb->Initialize(_dir == "" ? _filename : _dir + '/' + _filename, _t);
  mb->BuildCDStructure();

  m_obstacles.push_back(mb);
  return make_pair(m_obstacles.size() - 1, m_obstacles.back());
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
RemoveObstacle(shared_ptr<StaticMultiBody> _obst) {
  auto it = find(m_obstacles.begin(), m_obstacles.end(), _obst);
  if(it != m_obstacles.end())
    m_obstacles.erase(it);
  else
    cerr << "Environment::RemoveObstacleAt Warning: unable to remove obst."
         << endl;
}


map<Vector3d, vector<size_t>>
Environment::
ComputeObstacleVertexMap() const {
  map<Vector3d, vector<size_t>> out;

  // Iterate through all the obstacles and add their points to the map.
  for(size_t i = 0; i < NumObstacles(); ++i) {
    const auto& obstaclePoly = GetObstacle(i)->GetFixedBody(0)->
        GetWorldPolyhedron();
    for(const auto& v : obstaclePoly.GetVertexList())
      out[v].push_back(i);
  }

  return out;
}


void
Environment::
BuildCDStructure() {
  for(auto& body : m_obstacles)
    body->BuildCDStructure();
}

/*------------------------------- Helpers ------------------------------------*/

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
  else if(btype == "BOX2D")
    m_boundary = shared_ptr<BoundingBox2D>(new BoundingBox2D());
  else if(btype == "SPHERE")
    m_boundary = shared_ptr<BoundingSphere>(new BoundingSphere());
  else if(btype == "SPHERE2D")
    m_boundary = shared_ptr<BoundingSphere2D>(new BoundingSphere2D());
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

/*----------------------------------------------------------------------------*/
