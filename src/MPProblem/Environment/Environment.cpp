#include "MPProblem/Environment/Environment.h"

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Simulator/Conversions.h"

using namespace mathtool;


/*------------------------------- Construction -------------------------------*/

Terrain::
Terrain() = default;

Terrain::
Terrain(XMLNode& _node) {

  m_boundary = std::move(Boundary::Factory(_node));

  std::string color = _node.Read("color", false, "blue", "Color of the Terrain");
  m_wire = _node.Read("wire", false, true, "Render type");

  try {
    std::stringstream ss(color);
    ss >> this->m_color;
  }
  catch(const nonstd::exception&) {
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);
    m_color = StringToColor(color);
  }
}


Terrain::
Terrain(const Terrain& _terrain) {
  m_color = _terrain.m_color;
  m_boundary = _terrain.m_boundary->Clone();
  m_wire = _terrain.m_wire;
}


const glutils::color&
Terrain::
Color() const noexcept {
  return m_color;
}


const Boundary*
Terrain::
GetBoundary() const noexcept {
  return m_boundary.get();
}


bool
Terrain::
IsWired() const noexcept {
  return m_wire;
}


Environment::
Environment() = default;


Environment::
Environment(XMLNode& _node) {
  m_filename = GetPathName(_node.Filename())
             + _node.Read("filename", true, "", "env filename");

  // If the filename is an XML file we will read all of the environment
  // information from that file.
  const bool readXML = m_filename.substr(m_filename.rfind(".", std::string::npos))
                     == ".xml";

  if(readXML) {
    XMLNode envNode = XMLNode(m_filename, "Environment");
    // First read options from environment XML file.
    ReadXMLOptions(envNode);
    ReadXML(envNode);

    // Then read from problem XML node which may override some of the options from
    // the environment XML file.
    ReadXMLOptions(_node);
  }
  else {
    ReadXMLOptions(_node);
    Read(m_filename);
  }

  if(m_boundaryObstacle)
    CreateBoundaryObstacle();

}


Environment::
Environment(const Environment& _other) {
  *this = _other;
}


Environment::
Environment(Environment&& _other) = default;


Environment::
~Environment() = default;

std::unique_ptr<Environment>
Environment::
Clone(){
	return std::unique_ptr<Environment>(new Environment(*this));
}

/*-------------------------------- Assignment --------------------------------*/

Environment&
Environment::
operator=(const Environment& _other) {
  m_filename            = _other.m_filename;
  m_modelDataDir        = _other.m_modelDataDir;
  m_positionRes         = _other.m_positionRes;
  m_positionResFactor   = _other.m_positionResFactor;
  m_orientationRes      = _other.m_orientationRes;
  m_timeRes             = _other.m_timeRes;
  m_frictionCoefficient = _other.m_frictionCoefficient;
  m_gravity             = _other.m_gravity;
  m_terrains            = _other.m_terrains;

  // Copy the boundary.
  SetBoundary(_other.m_boundary->Clone());

  // Copy the obstacles.
  /// @note We deliberately duplicate the data to make sure the copies are
  ///       entirely independent. We may wish to revisit this decision at a
  ///       later time since it is probably safe to share data on static
  ///       obstacles. I am leaving it this way for now to minimize surprises.
  for(const auto& obstacle : _other.m_obstacles)
    m_obstacles.emplace_back(new MultiBody(*obstacle));

  return *this;
}


Environment&
Environment::
operator=(Environment&& _other) = default;

/*------------------------------------ I/O -----------------------------------*/

const std::string&
Environment::
GetEnvFileName() const noexcept {
  return m_filename;
}


void
Environment::
ReadXMLOptions(XMLNode& _node) {
  // Read the friction coefficient (to be used uniformly for now)
  m_frictionCoefficient = _node.Read("frictionCoefficient", false, 0., 0.,
      std::numeric_limits<double>::max(), "friction coefficient (uniform)");

  // Read in the gravity vector.
  {
    const std::string gravity = _node.Read("gravity", false, "0 0 0",
        "The gravity vector in this environment.");
    std::istringstream buffer(gravity);
    buffer >> m_gravity;
  }

  // If the position or orientation resolution is provided in the xml, overwrite
  // any previous value that could have been set in the env file.
  m_positionRes = _node.Read("positionRes", false, m_positionRes,
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
      "Positional resolution of environment");
  m_orientationRes = _node.Read("orientationRes", false, m_orientationRes,
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
      "Orientation resolution of environment");
  m_timeRes = _node.Read("timeRes", false, m_timeRes, .05, 10.,
      "Time resolution in seconds");

  m_boundaryObstacle = _node.Read("boundaryObstacle", false, m_boundaryObstacle,
      "Create a multibody obstacle for the boundary.");

  for(auto& child : _node) {
    if(child.Name() == "Terrain") {

      std::string capability = child.Read("capability", true, "", "The types of "
          "agents that can be within the terrain boundaries.");
      std::transform(capability.begin(), capability.end(), capability.begin(), ::tolower);

      for(auto& grandChild : child) {
        if(grandChild.Name() == "Boundary") {

          Terrain terrain(grandChild);

          m_terrains[capability].push_back(std::move(terrain));
        }
      }
    }
  }

  // Read the initial camera transformation. Assume slightly behind the z = 0
  // plane if not provided.
  {
    const std::string transformation = _node.Read("cameraInit", false,
        "0 0 20 0 0 0", "The initial transformation for the camera.");
    std::istringstream buffer(transformation);
    buffer >> m_initialCameraTransform;
  }
}


void
Environment::
ReadXML(XMLNode& _node) {
  size_t sl = m_filename.rfind('/');
  m_modelDataDir = m_filename.substr(0, sl == std::string::npos ? 0 : sl) + "/";
  Body::m_modelDataDir = m_modelDataDir;

  m_obstacles.clear();

  // Read and construct boundary, bodies, and other objects in the environment.
  for(auto& child : _node) {
    if(child.Name() == "Boundary") {
      m_boundary = Boundary::Factory(child);
    }
    else if(child.Name() == "MultiBody") {
      m_obstacles.emplace_back(new MultiBody(child));

      /// @TODO Add support for dynamic obstacles
      if(m_obstacles.back()->IsActive())
        throw ParseException(_node.Where(), "Dynamic obstacles are not yet "
            "supported.");
    }
  }
}


void
Environment::
Read(std::string _filename) {
  if(!FileExists(_filename))
    throw ParseException(_filename, "File does not exist");

  m_filename = _filename;
  size_t sl = m_filename.rfind('/');
  m_modelDataDir = m_filename.substr(0, sl == std::string::npos ? 0 : sl) + "/";
  Body::m_modelDataDir = m_modelDataDir;

  m_obstacles.clear();

  // open file
  CountingStreamBuffer cbs(_filename);
  std::istream ifs(&cbs);

  // read boundary
  std::string bndry = ReadFieldString(ifs, cbs, "Failed reading boundary tag.");
  if(bndry != "BOUNDARY")
    throw ParseException(cbs.Where(),
        "Unknown boundary tag '" + bndry + "'. Should read 'Boundary'.");
  std::string btype = ReadFieldString(ifs, cbs,
      "Failed reading boundary type. Options are: box or sphere.");
  InitializeBoundary(btype, cbs.Where());
  m_boundary->Read(ifs, cbs);

  // read resolutions
  std::string resolution;
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
    else if(resolution == "TIMERES")
      m_timeRes = ReadField<double>(ifs, cbs, "Failed reading Time resolution\n");
    else
      throw ParseException(cbs.Where(), "Unknown resolution tag '" + resolution
          + "'");
  }

  size_t multibodyCount = ReadField<size_t>(ifs, cbs,
      "Failed reading number of multibodies.");

  //parse and construct each multibody
  for(size_t m = 0; m < multibodyCount && ifs; ++m) {
    m_obstacles.emplace_back(new MultiBody(MultiBody::Type::Passive));
    m_obstacles.back()->Read(ifs, cbs);

    /// @TODO Add support for dynamic obstacles
    if(m_obstacles.back()->IsActive())
      throw ParseException(cbs.Where(), "Dynamic obstacles are not yet "
          "supported.");
  }
}


void
Environment::
Print(std::ostream& _os) const {
  _os << "Environment"
      << "\n\tpositionRes: " << m_positionRes
      << "\n\torientationRes: " << m_orientationRes
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
      << "\n\trdRes: " << m_rdRes
#endif
      << "\n\tboundary::" << *m_boundary
      << std::endl;
  for(const auto& obstacle : m_obstacles) {
    obstacle->Write(_os);
    _os << std::endl;
  }
}


void
Environment::
Write(std::ostream & _os) {
  _os << "Boundary ";
  m_boundary->Write(_os);
  _os << "\n\nObstacles\n"
      << m_obstacles.size()
      << "\n\n";
  for(const auto& body : m_obstacles) {
    body->Write(_os);
    _os << std::endl;
  }
}

/*-------------------------------- Resolutions -------------------------------*/

void
Environment::
ComputeResolution(const std::vector<std::unique_ptr<Robot>>& _robots) {
  if(m_positionRes >= 0.)
    return; // Do not compute it.

  double bodiesMinSpan = std::numeric_limits<double>::max();
  for(auto& robot : _robots)
    bodiesMinSpan = std::min(bodiesMinSpan,
                             robot->GetMultiBody()->GetMaxAxisRange());

  for(auto& body : m_obstacles)
    bodiesMinSpan = std::min(bodiesMinSpan, body->GetMaxAxisRange());

  // Set to XML input resolution if specified, else compute resolution factor
  m_positionRes = bodiesMinSpan * m_positionResFactor;

  /// @TODO Add an automatic computation of the orientation resolution here.
  ///       This should be done so that rotating the robot base by one orientation
  ///       resolution makes a point on the bounding sphere move by one position
  ///       resolution.

  // This is a very important parameter - always report a notice when we compute
  // it automatically.
  std::cout << "Automatically computed position resolution as " << m_positionRes
            << std::endl;
}


double
Environment::
GetPositionRes() const noexcept {
  return m_positionRes;
}


void
Environment::
SetPositionRes(double _res) noexcept {
  m_positionRes = _res;
}


double
Environment::
GetOrientationRes() const noexcept {
  return m_orientationRes;
}


void
Environment::
SetOrientationRes(double _res) noexcept {
  m_orientationRes = _res;
}


double
Environment::
GetTimeRes() const noexcept {
  return m_timeRes;
}

/*----------------------------- Boundary Functions ---------------------------*/

const Boundary*
Environment::
GetBoundary() const noexcept {
  return m_boundary.get();
}


void
Environment::
SetBoundary(std::unique_ptr<Boundary>&& _b) noexcept {
  m_boundary = std::move(_b);
}


void
Environment::
ResetBoundary(double _d, const MultiBody* const _multibody) {
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = std::numeric_limits<double>::max();
  maxx = maxy = maxz = std::numeric_limits<double>::lowest();

  double robotRadius = _multibody->GetBoundingSphereRadius();
  _d += robotRadius;

  for(auto& body : m_obstacles) {
    const double* tmp = body->GetBoundingBox();
    minx = std::min(minx, tmp[0]);
    maxx = std::max(maxx, tmp[1]);
    miny = std::min(miny, tmp[2]);
    maxy = std::max(maxy, tmp[3]);
    minz = std::min(minz, tmp[4]);
    maxz = std::max(maxz, tmp[5]);
  }

  std::vector<std::pair<double, double> > obstBBX(3);
  obstBBX[0] = std::make_pair(minx, maxx);
  obstBBX[1] = std::make_pair(miny, maxy);
  obstBBX[2] = std::make_pair(minz, maxz);

  m_boundary->ResetBoundary(obstBBX, _d);
}


void
Environment::
ResetBoundary(const std::vector<std::pair<double, double>>& _bbx,
    const double _margin) {
  m_boundary->ResetBoundary(_bbx, _margin);
}


void
Environment::
ExpandBoundary(double _d, const MultiBody* const _multibody) {
  double robotRadius = _multibody->GetBoundingSphereRadius();
  _d += robotRadius;

  std::vector<std::pair<double, double>> originBBX(3);
  for(size_t i = 0; i < 3; ++i) {
    const auto& r = GetBoundary()->GetRange(i);
    originBBX[i] = std::make_pair(r.min, r.max);
  }

  m_boundary->ResetBoundary(originBBX, _d);
}

/*---------------------------- Obstacle Functions ----------------------------*/

size_t
Environment::
NumObstacles() const noexcept {
  return m_obstacles.size();
}


MultiBody*
Environment::
GetObstacle(size_t _index) const {
  if(_index < 0 || _index >= m_obstacles.size())
    throw RunTimeException(WHERE) << "Cannot access obstacle '" << _index << "'.";
  return m_obstacles[_index].get();
}


MultiBody*
Environment::
GetRandomObstacle() const {
  if(m_obstacles.empty())
    throw RunTimeException(WHERE, "No static multibodies to select from.");

  size_t rIndex = LRand() % m_obstacles.size();
  return m_obstacles[rIndex].get();
}


size_t
Environment::
AddObstacle(const std::string& _dir, const std::string& _filename,
    const Transformation& _t) {
  const std::string filename = _dir.empty() ? _filename
                                            : _dir + '/' + _filename;

  // Make a multibody for this obstacle.
  std::unique_ptr<MultiBody> mb(new MultiBody(MultiBody::Type::Passive));

  // Make the obstacle geometry.
  Body body(mb.get());
  body.SetBodyType(Body::Type::Fixed);
  body.ReadGeometryFile(filename);
  body.Configure(_t);

  // Add the body to the multibody and finish initialization.
  const size_t index = mb->AddBody(std::move(body));
  mb->SetBaseBody(index);

  m_obstacles.push_back(std::move(mb));
  return m_obstacles.size() - 1;
}


void
Environment::
RemoveObstacle(const size_t _index) {
  const size_t count = m_obstacles.size();
  if(_index < count)
    m_obstacles.erase(m_obstacles.begin() + _index);
  else
    throw RunTimeException(WHERE) << "Cannot remove obstacle with index "
                                  << _index << ", only " << count
                                  << " obstacles in the environment.";
}


void
Environment::
RemoveObstacle(MultiBody* const _obst) {
  for(auto iter = m_obstacles.begin(); iter != m_obstacles.end(); ++iter) {
    if(iter->get() != _obst)
      continue;
    m_obstacles.erase(iter);
    return;
  }

  throw RunTimeException(WHERE) << "Cannot remove obstacle " << _obst
                                << ", does not match any obstacles in the "
                                << "environment.";
}


std::map<Vector3d, std::vector<size_t>>
Environment::
ComputeObstacleVertexMap() const {
  std::map<Vector3d, std::vector<size_t>> out;

  // Iterate through all the obstacles and add their points to the map.
  for(size_t i = 0; i < NumObstacles(); ++i) {
    MultiBody* const obst = GetObstacle(i);
    for(size_t j = 0; j < obst->GetNumBodies(); ++j) {
      const auto& obstaclePoly = obst->GetBody(j)->GetWorldPolyhedron();
      for(const auto& v : obstaclePoly.GetVertexList())
        out[v].push_back(i);
    }
  }

  return out;
}


bool
Environment::
UsingBoundaryObstacle() const noexcept {
  return m_boundaryObstacle;
}

/*-------------------------- Physical Properties -----------------------------*/

double
Environment::
GetFrictionCoefficient() const noexcept {
  return m_frictionCoefficient;
}


const Vector3d&
Environment::
GetGravity() const noexcept {
  return m_gravity;
}


/*-------------------------- Terrain Functions -----------------------------*/

const Environment::TerrainMap&
Environment::
GetTerrains() const noexcept {
  return m_terrains;
}

/*--------------------------- Simulation Functions ---------------------------*/

const mathtool::Transformation&
Environment::
GetInitialCameraTransformation() const noexcept {
  return m_initialCameraTransform;
}

/*------------------------------- Helpers ------------------------------------*/

void
Environment::
InitializeBoundary(std::string _type, const std::string _where) {
  std::transform(_type.begin(), _type.end(), _type.begin(), ::tolower);

  if(_type == "box")
    m_boundary = std::unique_ptr<Boundary>(new WorkspaceBoundingBox(3));
  else if(_type == "box2d")
    m_boundary = std::unique_ptr<Boundary>(new WorkspaceBoundingBox(2));
  else if(_type == "sphere")
    m_boundary = std::unique_ptr<Boundary>(new WorkspaceBoundingSphere(3));
  else if(_type == "sphere2d")
    m_boundary = std::unique_ptr<Boundary>(new WorkspaceBoundingSphere(2));
  else
    throw ParseException(_where, "Unknown boundary type '" + _type +
        "'. Options are: box, box2d, sphere, or sphere2d.");
}


void
Environment::
CreateBoundaryObstacle() {
  // Make a multibody for the boundary obstacle.
  std::unique_ptr<MultiBody> mb(new MultiBody(MultiBody::Type::Passive));

  // Make the boundary body.
  Body body(mb.get());
  body.SetBodyType(Body::Type::Fixed);
  body.SetPolyhedron(m_boundary->MakePolyhedron());

  // Add the body to the multibody and finish initialization.
  const size_t index = mb->AddBody(std::move(body));
  mb->SetBaseBody(index);

  // Ensure that we don't double-add the boundary obstacle.
  const bool alreadyAdded = !m_obstacles.empty() and
    mb->GetBase()->GetPolyhedron() ==
    m_obstacles[0]->GetBase()->GetPolyhedron();
  if(alreadyAdded) {
    std::cout << "Boundary obstacle already exists." << std::endl;
    return;
  }
  else
    std::cout << "Created boundary obstacle." << std::endl;

  // Insert the boundary at obstacle index 0.
  m_obstacles.insert(m_obstacles.begin(), std::move(mb));
}

/*----------------------------------------------------------------------------*/


//IROS Hacks

bool
Environment::
IsolateTerrain(Cfg start, Cfg goal){
  if(start.GetRobot()->GetCapability() != goal.GetRobot()->GetCapability() or
      start.GetRobot()->GetCapability() == "")
    return false;
  for(auto& terrain : m_terrains[start.GetRobot()->GetCapability()]){
    if(terrain.GetBoundary()->InBoundary(start) and terrain.GetBoundary()->InBoundary(goal)){
      this->SetBoundary(std::move(terrain.GetBoundary()->Clone()));
      return true;
    }
  }
  return false;
}

void
Environment::
RestoreBoundary(){
  this->SetBoundary(std::move(m_originalBoundary));
}

void
Environment::
SaveBoundary(){
  m_originalBoundary = this->GetBoundary()->Clone();
}
