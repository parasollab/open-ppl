#ifndef OBSTACLE_BASED_SAMPLER_H_
#define OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Obstacle-based sampling
///
/// OBPRM samples by pushing random configurations along a random ray until they
/// change validity, keeping the best free configuration.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ObstacleBasedSampler : public SamplerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    ObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
        int _free = 1, int _coll = 0, double _step = 0,
        bool _useBBX = true, string _pointSelection = "cspace");

    ObstacleBasedSampler(XMLNode& _node);

    void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    // Generates and adds shells to their containers
    void GenerateShells(const Boundary* const _boundary,
        CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    ////////////////////////////////////////////////////////////////
    // The following was added after unification with WOBPRM code //
    ////////////////////////////////////////////////////////////////

    // Returns a CfgType at the center of mass of the MultiBody
    CfgType ChooseCenterOfMass(MultiBody* _mBody);

    // Returns a CfgType at a random vertex of the MultiBody
    CfgType ChooseRandomVertex(MultiBody* _mBody, bool _isFreeBody);

    // Returns a point inside the triangle determined by the vectors
    Vector3d ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r);

    // Chooses a random point on a random triangle (weighted by area) in the
    // MultiBody
    CfgType ChooseRandomWeightedTriangle(MultiBody* _mBody,
        bool _isFreeBody);

    // Chooses a random point in a random triangle in the MultiBody
    CfgType ChooseRandomTriangle(MultiBody* _mBody, bool _isFreeBody);

    // Chooses a random extreme vertex of the MultiBody
    CfgType ChooseExtremeVertex(MultiBody* _mBody, bool _isFreeBody);

    // Checks m_pointSelection and returns an appropriate CfgType
    virtual CfgType ChooseASample(CfgType& _cfg);

  private:

    // Returns a CfgType with the coordinates specified in the vector and no
    // rotation
    CfgType GetCfgWithParams(const Vector3d& _v);

    string m_vcLabel, m_dmLabel; // Validity checker method, distance metric method
    int m_nShellsFree, m_nShellsColl; // Number of free and collision shells
    double m_stepSize; // Step size along the random ray
    bool m_useBBX; // Is the bounding box an obstacle?
    string m_pointSelection; // Needed for the WOBPRM
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ObstacleBasedSampler<MPTraits>::
ObstacleBasedSampler(string _vcLabel, string _dmLabel,
    int _free, int _coll, double _step, bool _useBBX, string _pointSelection) :
    m_vcLabel(_vcLabel), m_dmLabel(_dmLabel), m_nShellsFree(_free),
    m_nShellsColl(_coll), m_stepSize(_step), m_useBBX(_useBBX),
    m_pointSelection(_pointSelection) {
  this->SetName("ObstacleBasedSampler");
}


template <typename MPTraits>
ObstacleBasedSampler<MPTraits>::
ObstacleBasedSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("ObstacleBasedSampler");

  m_useBBX = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity test method");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance metric method");
  m_pointSelection = _node.Read("pointSelection", false,
      "cspace", "Point selection strategy");
  m_nShellsColl = _node.Read("nShellsColl", true, 3, 0, 10,
      "Number of collision shells");
  m_nShellsFree = _node.Read("nShellsFree", true, 3, 0, 10,
      "Number of free shells");
  m_stepSize = _node.Read("stepSize", true, 0.0, 0.0, 10.0,
      "Step size used in increment of cfg position towards or away from"
      "obstacles");

  // Check if the read point_selection is valid
  if(m_pointSelection != "cspace" && m_pointSelection != "cM" &&
      m_pointSelection != "rV" && m_pointSelection != "rT" &&
      m_pointSelection != "rW" && m_pointSelection != "eV" &&
      m_pointSelection != "rV_rT" && m_pointSelection != "rV_rW" &&
      m_pointSelection != "all")
    throw ParseException(_node.Where(),
        "Invalid point selection type '" + m_pointSelection + "'."
        "Choices are: 'cspace', 'cM', 'rV', 'rT', 'rW', 'eV', 'rV_rT', 'rV_rW',"
        " and 'all'.");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tnShellsFree = " << m_nShellsFree << endl;
  _os << "\tnShellsColl = " << m_nShellsColl << endl;
  _os << "\tstepSize = " << m_stepSize << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
  _os << "\tuseBBX = " << m_useBBX << endl;
  _os << "\tpointSelectionStrategy = " << m_pointSelection << endl;
}


template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
GenerateShells(const Boundary* const _boundary,
    CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee = this->GetNameAndLabel() + "::GenerateShells()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  if(this->m_debug)
    cout << "nShellsColl = " << m_nShellsColl << endl;

  // Add free shells
  for(int i = 0; i < m_nShellsFree; i++) {
    // If the shell is valid
    if(_cFree.InBounds(_boundary) and vc->IsValid(_cFree, callee))
      _result.push_back(_cFree);
    // Get next shell
    _cFree += _incr;
  }

  // Reverse direction of _incr
  _incr = -_incr;

  // Add collision shells
  for(int i = 0; i < m_nShellsColl; i++) {
    // If the shell is valid
    if(_cColl.InBounds(_boundary) and !vc->IsValid(_cColl, callee))
      _collision.push_back(_cColl);
    // Get next shell
    _cColl += _incr;
  }
}


template <typename MPTraits>
bool
ObstacleBasedSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  Environment* env = this->GetEnvironment();
  // If the step size is unreasonable, set it to the minimum
  if(m_stepSize <= 0.0)
    m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());

  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto robot = this->GetTask()->GetRobot();

  // Old state
  CfgType c1 = ChooseASample(_cfg);
  bool c1BBox = c1.InBounds(_boundary);
  bool c1Free = vc->IsValid(c1, callee);

  // New state
  CfgType c2 = c1;
  bool c2BBox = c1BBox;
  bool c2Free = c1Free;

  // Create a random ray
  CfgType r(robot);
  r.GetRandomRay(m_stepSize, dm);
  if(r == CfgType(robot)) {
    if(this->m_debug)
      cerr << "Random ray in OBPRM Sampler is 0-valued!\
        Continuing with loop." << endl;
    return false;
  }

  // Loop until the new state is outside the bounds or the validity changes
  while(c2BBox && (c1Free == c2Free)) {
    // Copy new data to old state
    c1 = c2;
    c1BBox = c2BBox;
    c1Free = c2Free;
    // Update new state
    c2 += r;
    c2BBox = c2.InBounds(_boundary);
    c2Free = vc->IsValid(c2, callee);
  }

  // Construct Shells and generate _collision and _result
  if (!m_useBBX) { // If we treat the bounding box as an obstacle
    if(c2BBox) { // If the new state (c2) is in the bounding box
      if(c1Free) { // If the old state (c1) is  in the free C space
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        _collision.push_back(c2);
      }
      else { // The new state (c2) is in the free C space
        // Process configurations
        GenerateShells(_boundary, c2, c1, r, _result, _collision);
        _collision.push_back(c1);
      }
      return true;
    }
  } else {
    if (c1BBox) {
      if (c1Free) { // If the old state (c1) is in the free C space
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        _collision.push_back(c2);
        return true;
      }
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////
// The following was added after unification with WOBPRM code //
// ////////////////////////////////////////////////////////////////

// Returns a CfgType at the center of mass of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseCenterOfMass(MultiBody* _mBody) {
  return GetCfgWithParams(_mBody->GetCenterOfMass());
}

// Returns a CfgType at a random vertex of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomVertex(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  Vector3d x = polyhedron.GetVertexList()[(int)(DRand()*polyhedron.GetVertexList().size())];
  return GetCfgWithParams(x);
}

// Returns a point inside the triangle determined by the vectors
template <typename MPTraits>
Vector3d
ObstacleBasedSampler<MPTraits>::
ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r) {
  Vector3d u = _q - _p; // From _p to _q
  Vector3d v = _r - _p; // From _p to _r
  double s = DRand();
  double t = DRand();
  // Keep point inside the triangle
  while(s + t > 1) {
    s = DRand();
    t = DRand();
  }
  return (_p + u*s + v*t);
}

// Chooses a random point on a random triangle (weighted by area) in the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomWeightedTriangle(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  // A random fraction of the area
  double targetArea = polyhedron.GetSurfaceArea() * DRand();
  double sum = 0.0;
  int index;

  // Choose index as the triangle that first makes sum > targetArea
  for(index = -1; sum <= targetArea; index++)
    sum += polyhedron.GetPolygonList()[index + 1].GetArea();
  // Choose the triangle of the MultiBody with that index
  const GMSPolygon& poly = polyhedron.GetPolygonList()[index];
  // Choose a random point in that triangle
  const Vector3d& p = poly.GetPoint(0);
  const Vector3d& q = poly.GetPoint(1);
  const Vector3d& r = poly.GetPoint(2);
  Vector3d x = ChoosePointOnTriangle(p, q, r);

  return GetCfgWithParams(x);
}

// Chooses a random point in a random triangle in the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomTriangle(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();

  // Choose a random triangle
  const GMSPolygon& poly = polyhedron.GetPolygonList()[(int)(DRand()*polyhedron.GetPolygonList().size())];
  const Vector3d& p = poly.GetPoint(0);
  const Vector3d& q = poly.GetPoint(1);
  const Vector3d& r = poly.GetPoint(2);
  Vector3d x = ChoosePointOnTriangle(p, q, r);

  return GetCfgWithParams(x);
}

// Chooses a random extreme vertex of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseExtremeVertex(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  int xyz = LRand() % 3; // 0: x, 1: y, 2: z
  int minMax = LRand() % 2; // 0: min, 1: max
  int x = 0; // Index of extreme value

  // Find extreme value
  for(size_t i = 1; i < polyhedron.GetVertexList().size(); i++)
    // minMax is an optional negation
    if(((polyhedron.GetVertexList()[i][xyz] < polyhedron.GetVertexList()[x][xyz]) + minMax) % 2)
      x = i;

  return GetCfgWithParams(polyhedron.GetVertexList()[x]);
}

// Checks m_pointSelection and returns an appropriate CfgType
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseASample(CfgType& _cfg) {
  MultiBody* mBody{nullptr};
  if(m_pointSelection != "cspace")
    mBody = this->GetEnvironment()->GetRandomObstacle();

  // cspace is for Configuration space (This is for unifying OBPRM and WOBPRM)
  if(m_pointSelection == "cspace")
    return _cfg;
  else if(m_pointSelection == "cM")
    return ChooseCenterOfMass(mBody);
  else if(m_pointSelection == "rV")
    return ChooseRandomVertex(mBody, false);
  else if(m_pointSelection == "rT")
    return ChooseRandomTriangle(mBody, false);
  else if(m_pointSelection == "rW")
    return ChooseRandomWeightedTriangle(mBody, false);
  else if(m_pointSelection == "eV")
    return ChooseExtremeVertex(mBody, false);
  else if(m_pointSelection == "cM_rV") {
    if(LRand() % 2)
      return ChooseCenterOfMass(mBody);
    else
      return ChooseRandomVertex(mBody, false);
  }
  else if(m_pointSelection == "rV_rT") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody, false);
    else
      return ChooseRandomTriangle(mBody, false);
  }
  else if(m_pointSelection == "rV_rW") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody, false);
    else
      return ChooseRandomWeightedTriangle(mBody, false);
  }
  else if(m_pointSelection == "all") {
    switch(LRand() % 5) {
      case 0:
        return ChooseCenterOfMass(mBody);
      case 1:
        return ChooseRandomVertex(mBody, false);
      case 2:
        return ChooseRandomTriangle(mBody, false);
      case 3:
        return ChooseRandomWeightedTriangle(mBody, false);
      default:
        return ChooseExtremeVertex(mBody, false);
    }
  }
  else {
    throw PMPLException("OBPRM Exception", WHERE, "Select a valid point selection type first. Exiting.");
  }
}

// Returns a CfgType with the coordinates specified in the vector and no rotation
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
GetCfgWithParams(const Vector3d& _v) {
  CfgType tmp(this->GetTask()->GetRobot());
  for(int i = 0; i < 3; i++)
    tmp[i] = _v[i];
  for(int i = 3; i < 6; i++)
    tmp[i] = 0.0;
  return tmp;
}

#endif
#ifndef OBSTACLE_BASED_SAMPLER_H_
#define OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Obstacle-based sampling
///
/// OBPRM samples by pushing random configurations along a random ray until they
/// change validity, keeping the best free configuration.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ObstacleBasedSampler : public SamplerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    ObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
        int _free = 1, int _coll = 0, double _step = 0,
        bool _useBBX = true, string _pointSelection = "cspace");

    ObstacleBasedSampler(XMLNode& _node);

    void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    // Generates and adds shells to their containers
    void GenerateShells(const Boundary* const _boundary,
        CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    ////////////////////////////////////////////////////////////////
    // The following was added after unification with WOBPRM code //
    ////////////////////////////////////////////////////////////////

    // Returns a CfgType at the center of mass of the MultiBody
    CfgType ChooseCenterOfMass(MultiBody* _mBody);

    // Returns a CfgType at a random vertex of the MultiBody
    CfgType ChooseRandomVertex(MultiBody* _mBody, bool _isFreeBody);

    // Returns a point inside the triangle determined by the vectors
    Vector3d ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r);

    // Chooses a random point on a random triangle (weighted by area) in the
    // MultiBody
    CfgType ChooseRandomWeightedTriangle(MultiBody* _mBody,
        bool _isFreeBody);

    // Chooses a random point in a random triangle in the MultiBody
    CfgType ChooseRandomTriangle(MultiBody* _mBody, bool _isFreeBody);

    // Chooses a random extreme vertex of the MultiBody
    CfgType ChooseExtremeVertex(MultiBody* _mBody, bool _isFreeBody);

    // Checks m_pointSelection and returns an appropriate CfgType
    virtual CfgType ChooseASample(CfgType& _cfg);

  private:

    // Returns a CfgType with the coordinates specified in the vector and no
    // rotation
    CfgType GetCfgWithParams(const Vector3d& _v);

    string m_vcLabel, m_dmLabel; // Validity checker method, distance metric method
    int m_nShellsFree, m_nShellsColl; // Number of free and collision shells
    double m_stepSize; // Step size along the random ray
    bool m_useBBX; // Is the bounding box an obstacle?
    string m_pointSelection; // Needed for the WOBPRM
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ObstacleBasedSampler<MPTraits>::
ObstacleBasedSampler(string _vcLabel, string _dmLabel,
    int _free, int _coll, double _step, bool _useBBX, string _pointSelection) :
    m_vcLabel(_vcLabel), m_dmLabel(_dmLabel), m_nShellsFree(_free),
    m_nShellsColl(_coll), m_stepSize(_step), m_useBBX(_useBBX),
    m_pointSelection(_pointSelection) {
  this->SetName("ObstacleBasedSampler");
}


template <typename MPTraits>
ObstacleBasedSampler<MPTraits>::
ObstacleBasedSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("ObstacleBasedSampler");

  m_useBBX = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity test method");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance metric method");
  m_pointSelection = _node.Read("pointSelection", false,
      "cspace", "Point selection strategy");
  m_nShellsColl = _node.Read("nShellsColl", true, 3, 0, 10,
      "Number of collision shells");
  m_nShellsFree = _node.Read("nShellsFree", true, 3, 0, 10,
      "Number of free shells");
  m_stepSize = _node.Read("stepSize", true, 0.0, 0.0, 10.0,
      "Step size used in increment of cfg position towards or away from"
      "obstacles");

  // Check if the read point_selection is valid
  if(m_pointSelection != "cspace" && m_pointSelection != "cM" &&
      m_pointSelection != "rV" && m_pointSelection != "rT" &&
      m_pointSelection != "rW" && m_pointSelection != "eV" &&
      m_pointSelection != "rV_rT" && m_pointSelection != "rV_rW" &&
      m_pointSelection != "all")
    throw ParseException(_node.Where(),
        "Invalid point selection type '" + m_pointSelection + "'."
        "Choices are: 'cspace', 'cM', 'rV', 'rT', 'rW', 'eV', 'rV_rT', 'rV_rW',"
        " and 'all'.");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tnShellsFree = " << m_nShellsFree << endl;
  _os << "\tnShellsColl = " << m_nShellsColl << endl;
  _os << "\tstepSize = " << m_stepSize << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
  _os << "\tuseBBX = " << m_useBBX << endl;
  _os << "\tpointSelectionStrategy = " << m_pointSelection << endl;
}


template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
GenerateShells(const Boundary* const _boundary,
    CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee = this->GetNameAndLabel() + "::GenerateShells()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  if(this->m_debug)
    cout << "nShellsColl = " << m_nShellsColl << endl;

  // Add free shells
  for(int i = 0; i < m_nShellsFree; i++) {
    // If the shell is valid
    if(_cFree.InBounds(_boundary) and vc->IsValid(_cFree, callee))
      _result.push_back(_cFree);
    // Get next shell
    _cFree += _incr;
  }

  // Reverse direction of _incr
  _incr = -_incr;

  // Add collision shells
  for(int i = 0; i < m_nShellsColl; i++) {
    // If the shell is valid
    if(_cColl.InBounds(_boundary) and !vc->IsValid(_cColl, callee))
      _collision.push_back(_cColl);
    // Get next shell
    _cColl += _incr;
  }
}


template <typename MPTraits>
bool
ObstacleBasedSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  Environment* env = this->GetEnvironment();
  // If the step size is unreasonable, set it to the minimum
  if(m_stepSize <= 0.0)
    m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());

  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto robot = this->GetTask()->GetRobot();

  // Old state
  CfgType c1 = ChooseASample(_cfg);
  bool c1BBox = c1.InBounds(_boundary);
  bool c1Free = vc->IsValid(c1, callee);

  // New state
  CfgType c2 = c1;
  bool c2BBox = c1BBox;
  bool c2Free = c1Free;

  // Create a random ray
  CfgType r(robot);
  r.GetRandomRay(m_stepSize, dm);
  if(r == CfgType(robot)) {
    if(this->m_debug)
      cerr << "Random ray in OBPRM Sampler is 0-valued!\
        Continuing with loop." << endl;
    return false;
  }

  // Loop until the new state is outside the bounds or the validity changes
  while(c2BBox && (c1Free == c2Free)) {
    // Copy new data to old state
    c1 = c2;
    c1BBox = c2BBox;
    c1Free = c2Free;
    // Update new state
    c2 += r;
    c2BBox = c2.InBounds(_boundary);
    c2Free = vc->IsValid(c2, callee);
  }

  // Construct Shells and generate _collision and _result
  if (!m_useBBX) { // If we treat the bounding box as an obstacle
    if(c2BBox) { // If the new state (c2) is in the bounding box
      if(c1Free) { // If the old state (c1) is  in the free C space
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        _collision.push_back(c2);
      }
      else { // The new state (c2) is in the free C space
        // Process configurations
        GenerateShells(_boundary, c2, c1, r, _result, _collision);
        _collision.push_back(c1);
      }
      return true;
    }
  } else {
    if (c1BBox) {
      if (c1Free) { // If the old state (c1) is in the free C space
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        _collision.push_back(c2);
        return true;
      }
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////
// The following was added after unification with WOBPRM code //
// ////////////////////////////////////////////////////////////////

// Returns a CfgType at the center of mass of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseCenterOfMass(MultiBody* _mBody) {
  return GetCfgWithParams(_mBody->GetCenterOfMass());
}

// Returns a CfgType at a random vertex of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomVertex(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  Vector3d x = polyhedron.GetVertexList()[(int)(DRand()*polyhedron.GetVertexList().size())];
  return GetCfgWithParams(x);
}

// Returns a point inside the triangle determined by the vectors
template <typename MPTraits>
Vector3d
ObstacleBasedSampler<MPTraits>::
ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r) {
  Vector3d u = _q - _p; // From _p to _q
  Vector3d v = _r - _p; // From _p to _r
  double s = DRand();
  double t = DRand();
  // Keep point inside the triangle
  while(s + t > 1) {
    s = DRand();
    t = DRand();
  }
  return (_p + u*s + v*t);
}

// Chooses a random point on a random triangle (weighted by area) in the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomWeightedTriangle(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  // A random fraction of the area
  double targetArea = polyhedron.GetSurfaceArea() * DRand();
  double sum = 0.0;
  int index;

  // Choose index as the triangle that first makes sum > targetArea
  for(index = -1; sum <= targetArea; index++)
    sum += polyhedron.GetPolygonList()[index + 1].GetArea();
  // Choose the triangle of the MultiBody with that index
  const GMSPolygon& poly = polyhedron.GetPolygonList()[index];
  // Choose a random point in that triangle
  const Vector3d& p = poly.GetPoint(0);
  const Vector3d& q = poly.GetPoint(1);
  const Vector3d& r = poly.GetPoint(2);
  Vector3d x = ChoosePointOnTriangle(p, q, r);

  return GetCfgWithParams(x);
}

// Chooses a random point in a random triangle in the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomTriangle(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();

  // Choose a random triangle
  const GMSPolygon& poly = polyhedron.GetPolygonList()[(int)(DRand()*polyhedron.GetPolygonList().size())];
  const Vector3d& p = poly.GetPoint(0);
  const Vector3d& q = poly.GetPoint(1);
  const Vector3d& r = poly.GetPoint(2);
  Vector3d x = ChoosePointOnTriangle(p, q, r);

  return GetCfgWithParams(x);
}

// Chooses a random extreme vertex of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseExtremeVertex(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  int xyz = LRand() % 3; // 0: x, 1: y, 2: z
  int minMax = LRand() % 2; // 0: min, 1: max
  int x = 0; // Index of extreme value

  // Find extreme value
  for(size_t i = 1; i < polyhedron.GetVertexList().size(); i++)
    // minMax is an optional negation
    if(((polyhedron.GetVertexList()[i][xyz] < polyhedron.GetVertexList()[x][xyz]) + minMax) % 2)
      x = i;

  return GetCfgWithParams(polyhedron.GetVertexList()[x]);
}

// Checks m_pointSelection and returns an appropriate CfgType
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseASample(CfgType& _cfg) {
  MultiBody* mBody{nullptr};
  if(m_pointSelection != "cspace")
    mBody = this->GetEnvironment()->GetRandomObstacle();

  // cspace is for Configuration space (This is for unifying OBPRM and WOBPRM)
  if(m_pointSelection == "cspace")
    return _cfg;
  else if(m_pointSelection == "cM")
    return ChooseCenterOfMass(mBody);
  else if(m_pointSelection == "rV")
    return ChooseRandomVertex(mBody, false);
  else if(m_pointSelection == "rT")
    return ChooseRandomTriangle(mBody, false);
  else if(m_pointSelection == "rW")
    return ChooseRandomWeightedTriangle(mBody, false);
  else if(m_pointSelection == "eV")
    return ChooseExtremeVertex(mBody, false);
  else if(m_pointSelection == "cM_rV") {
    if(LRand() % 2)
      return ChooseCenterOfMass(mBody);
    else
      return ChooseRandomVertex(mBody, false);
  }
  else if(m_pointSelection == "rV_rT") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody, false);
    else
      return ChooseRandomTriangle(mBody, false);
  }
  else if(m_pointSelection == "rV_rW") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody, false);
    else
      return ChooseRandomWeightedTriangle(mBody, false);
  }
  else if(m_pointSelection == "all") {
    switch(LRand() % 5) {
      case 0:
        return ChooseCenterOfMass(mBody);
      case 1:
        return ChooseRandomVertex(mBody, false);
      case 2:
        return ChooseRandomTriangle(mBody, false);
      case 3:
        return ChooseRandomWeightedTriangle(mBody, false);
      default:
        return ChooseExtremeVertex(mBody, false);
    }
  }
  else {
    throw PMPLException("OBPRM Exception", WHERE, "Select a valid point selection type first. Exiting.");
  }
}

// Returns a CfgType with the coordinates specified in the vector and no rotation
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
GetCfgWithParams(const Vector3d& _v) {
  CfgType tmp(this->GetTask()->GetRobot());
  for(int i = 0; i < 3; i++)
    tmp[i] = _v[i];
  for(int i = 3; i < 6; i++)
    tmp[i] = 0.0;
  return tmp;
}

#endif
#ifndef OBSTACLE_BASED_SAMPLER_H_
#define OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Obstacle-based sampling
///
/// OBPRM samples by pushing random configurations along a random ray until they
/// change validity, keeping the best free configuration.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ObstacleBasedSampler : public SamplerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    ObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
        int _free = 1, int _coll = 0, double _step = 0,
        bool _useBBX = true, string _pointSelection = "cspace");

    ObstacleBasedSampler(XMLNode& _node);

    void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    // Generates and adds shells to their containers
    void GenerateShells(const Boundary* const _boundary,
        CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    ////////////////////////////////////////////////////////////////
    // The following was added after unification with WOBPRM code //
    ////////////////////////////////////////////////////////////////

    // Returns a CfgType at the center of mass of the MultiBody
    CfgType ChooseCenterOfMass(MultiBody* _mBody);

    // Returns a CfgType at a random vertex of the MultiBody
    CfgType ChooseRandomVertex(MultiBody* _mBody, bool _isFreeBody);

    // Returns a point inside the triangle determined by the vectors
    Vector3d ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r);

    // Chooses a random point on a random triangle (weighted by area) in the
    // MultiBody
    CfgType ChooseRandomWeightedTriangle(MultiBody* _mBody,
        bool _isFreeBody);

    // Chooses a random point in a random triangle in the MultiBody
    CfgType ChooseRandomTriangle(MultiBody* _mBody, bool _isFreeBody);

    // Chooses a random extreme vertex of the MultiBody
    CfgType ChooseExtremeVertex(MultiBody* _mBody, bool _isFreeBody);

    // Checks m_pointSelection and returns an appropriate CfgType
    virtual CfgType ChooseASample(CfgType& _cfg);

  private:

    // Returns a CfgType with the coordinates specified in the vector and no
    // rotation
    CfgType GetCfgWithParams(const Vector3d& _v);

    string m_vcLabel, m_dmLabel; // Validity checker method, distance metric method
    int m_nShellsFree, m_nShellsColl; // Number of free and collision shells
    double m_stepSize; // Step size along the random ray
    bool m_useBBX; // Is the bounding box an obstacle?
    string m_pointSelection; // Needed for the WOBPRM
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ObstacleBasedSampler<MPTraits>::
ObstacleBasedSampler(string _vcLabel, string _dmLabel,
    int _free, int _coll, double _step, bool _useBBX, string _pointSelection) :
    m_vcLabel(_vcLabel), m_dmLabel(_dmLabel), m_nShellsFree(_free),
    m_nShellsColl(_coll), m_stepSize(_step), m_useBBX(_useBBX),
    m_pointSelection(_pointSelection) {
  this->SetName("ObstacleBasedSampler");
}


template <typename MPTraits>
ObstacleBasedSampler<MPTraits>::
ObstacleBasedSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("ObstacleBasedSampler");

  m_useBBX = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity test method");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance metric method");
  m_pointSelection = _node.Read("pointSelection", false,
      "cspace", "Point selection strategy");
  m_nShellsColl = _node.Read("nShellsColl", true, 3, 0, 10,
      "Number of collision shells");
  m_nShellsFree = _node.Read("nShellsFree", true, 3, 0, 10,
      "Number of free shells");
  m_stepSize = _node.Read("stepSize", true, 0.0, 0.0, 10.0,
      "Step size used in increment of cfg position towards or away from"
      "obstacles");

  // Check if the read point_selection is valid
  if(m_pointSelection != "cspace" && m_pointSelection != "cM" &&
      m_pointSelection != "rV" && m_pointSelection != "rT" &&
      m_pointSelection != "rW" && m_pointSelection != "eV" &&
      m_pointSelection != "rV_rT" && m_pointSelection != "rV_rW" &&
      m_pointSelection != "all")
    throw ParseException(_node.Where(),
        "Invalid point selection type '" + m_pointSelection + "'."
        "Choices are: 'cspace', 'cM', 'rV', 'rT', 'rW', 'eV', 'rV_rT', 'rV_rW',"
        " and 'all'.");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tnShellsFree = " << m_nShellsFree << endl;
  _os << "\tnShellsColl = " << m_nShellsColl << endl;
  _os << "\tstepSize = " << m_stepSize << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
  _os << "\tuseBBX = " << m_useBBX << endl;
  _os << "\tpointSelectionStrategy = " << m_pointSelection << endl;
}


template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
GenerateShells(const Boundary* const _boundary,
    CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee = this->GetNameAndLabel() + "::GenerateShells()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  if(this->m_debug)
    cout << "nShellsColl = " << m_nShellsColl << endl;

  // Add free shells
  for(int i = 0; i < m_nShellsFree; i++) {
    // If the shell is valid
    if(_cFree.InBounds(_boundary) and vc->IsValid(_cFree, callee))
      _result.push_back(_cFree);
    // Get next shell
    _cFree += _incr;
  }

  // Reverse direction of _incr
  _incr = -_incr;

  // Add collision shells
  for(int i = 0; i < m_nShellsColl; i++) {
    // If the shell is valid
    if(_cColl.InBounds(_boundary) and !vc->IsValid(_cColl, callee))
      _collision.push_back(_cColl);
    // Get next shell
    _cColl += _incr;
  }
}


template <typename MPTraits>
bool
ObstacleBasedSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  Environment* env = this->GetEnvironment();
  // If the step size is unreasonable, set it to the minimum
  if(m_stepSize <= 0.0)
    m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());

  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto robot = this->GetTask()->GetRobot();

  // Old state
  CfgType c1 = ChooseASample(_cfg);
  bool c1BBox = c1.InBounds(_boundary);
  bool c1Free = vc->IsValid(c1, callee);

  // New state
  CfgType c2 = c1;
  bool c2BBox = c1BBox;
  bool c2Free = c1Free;

  // Create a random ray
  CfgType r(robot);
  r.GetRandomRay(m_stepSize, dm);
  if(r == CfgType(robot)) {
    if(this->m_debug)
      cerr << "Random ray in OBPRM Sampler is 0-valued!\
        Continuing with loop." << endl;
    return false;
  }

  // Loop until the new state is outside the bounds or the validity changes
  while(c2BBox && (c1Free == c2Free)) {
    // Copy new data to old state
    c1 = c2;
    c1BBox = c2BBox;
    c1Free = c2Free;
    // Update new state
    c2 += r;
    c2BBox = c2.InBounds(_boundary);
    c2Free = vc->IsValid(c2, callee);
  }

  // Construct Shells and generate _collision and _result
  if (!m_useBBX) { // If we treat the bounding box as an obstacle
    if(c2BBox) { // If the new state (c2) is in the bounding box
      if(c1Free) { // If the old state (c1) is  in the free C space
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        _collision.push_back(c2);
      }
      else { // The new state (c2) is in the free C space
        // Process configurations
        GenerateShells(_boundary, c2, c1, r, _result, _collision);
        _collision.push_back(c1);
      }
      return true;
    }
  } else {
    if (c1BBox) {
      if (c1Free) { // If the old state (c1) is in the free C space
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        _collision.push_back(c2);
        return true;
      }
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////
// The following was added after unification with WOBPRM code //
// ////////////////////////////////////////////////////////////////

// Returns a CfgType at the center of mass of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseCenterOfMass(MultiBody* _mBody) {
  return GetCfgWithParams(_mBody->GetCenterOfMass());
}

// Returns a CfgType at a random vertex of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomVertex(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  Vector3d x = polyhedron.GetVertexList()[(int)(DRand()*polyhedron.GetVertexList().size())];
  return GetCfgWithParams(x);
}

// Returns a point inside the triangle determined by the vectors
template <typename MPTraits>
Vector3d
ObstacleBasedSampler<MPTraits>::
ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r) {
  Vector3d u = _q - _p; // From _p to _q
  Vector3d v = _r - _p; // From _p to _r
  double s = DRand();
  double t = DRand();
  // Keep point inside the triangle
  while(s + t > 1) {
    s = DRand();
    t = DRand();
  }
  return (_p + u*s + v*t);
}

// Chooses a random point on a random triangle (weighted by area) in the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomWeightedTriangle(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  // A random fraction of the area
  double targetArea = polyhedron.GetSurfaceArea() * DRand();
  double sum = 0.0;
  int index;

  // Choose index as the triangle that first makes sum > targetArea
  for(index = -1; sum <= targetArea; index++)
    sum += polyhedron.GetPolygonList()[index + 1].GetArea();
  // Choose the triangle of the MultiBody with that index
  const GMSPolygon& poly = polyhedron.GetPolygonList()[index];
  // Choose a random point in that triangle
  const Vector3d& p = poly.GetPoint(0);
  const Vector3d& q = poly.GetPoint(1);
  const Vector3d& r = poly.GetPoint(2);
  Vector3d x = ChoosePointOnTriangle(p, q, r);

  return GetCfgWithParams(x);
}

// Chooses a random point in a random triangle in the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomTriangle(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();

  // Choose a random triangle
  const GMSPolygon& poly = polyhedron.GetPolygonList()[(int)(DRand()*polyhedron.GetPolygonList().size())];
  const Vector3d& p = poly.GetPoint(0);
  const Vector3d& q = poly.GetPoint(1);
  const Vector3d& r = poly.GetPoint(2);
  Vector3d x = ChoosePointOnTriangle(p, q, r);

  return GetCfgWithParams(x);
}

// Chooses a random extreme vertex of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseExtremeVertex(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  int xyz = LRand() % 3; // 0: x, 1: y, 2: z
  int minMax = LRand() % 2; // 0: min, 1: max
  int x = 0; // Index of extreme value

  // Find extreme value
  for(size_t i = 1; i < polyhedron.GetVertexList().size(); i++)
    // minMax is an optional negation
    if(((polyhedron.GetVertexList()[i][xyz] < polyhedron.GetVertexList()[x][xyz]) + minMax) % 2)
      x = i;

  return GetCfgWithParams(polyhedron.GetVertexList()[x]);
}

// Checks m_pointSelection and returns an appropriate CfgType
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseASample(CfgType& _cfg) {
  MultiBody* mBody{nullptr};
  if(m_pointSelection != "cspace")
    mBody = this->GetEnvironment()->GetRandomObstacle();

  // cspace is for Configuration space (This is for unifying OBPRM and WOBPRM)
  if(m_pointSelection == "cspace")
    return _cfg;
  else if(m_pointSelection == "cM")
    return ChooseCenterOfMass(mBody);
  else if(m_pointSelection == "rV")
    return ChooseRandomVertex(mBody, false);
  else if(m_pointSelection == "rT")
    return ChooseRandomTriangle(mBody, false);
  else if(m_pointSelection == "rW")
    return ChooseRandomWeightedTriangle(mBody, false);
  else if(m_pointSelection == "eV")
    return ChooseExtremeVertex(mBody, false);
  else if(m_pointSelection == "cM_rV") {
    if(LRand() % 2)
      return ChooseCenterOfMass(mBody);
    else
      return ChooseRandomVertex(mBody, false);
  }
  else if(m_pointSelection == "rV_rT") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody, false);
    else
      return ChooseRandomTriangle(mBody, false);
  }
  else if(m_pointSelection == "rV_rW") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody, false);
    else
      return ChooseRandomWeightedTriangle(mBody, false);
  }
  else if(m_pointSelection == "all") {
    switch(LRand() % 5) {
      case 0:
        return ChooseCenterOfMass(mBody);
      case 1:
        return ChooseRandomVertex(mBody, false);
      case 2:
        return ChooseRandomTriangle(mBody, false);
      case 3:
        return ChooseRandomWeightedTriangle(mBody, false);
      default:
        return ChooseExtremeVertex(mBody, false);
    }
  }
  else {
    throw PMPLException("OBPRM Exception", WHERE, "Select a valid point selection type first. Exiting.");
  }
}

// Returns a CfgType with the coordinates specified in the vector and no rotation
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
GetCfgWithParams(const Vector3d& _v) {
  CfgType tmp(this->GetTask()->GetRobot());
  for(int i = 0; i < 3; i++)
    tmp[i] = _v[i];
  for(int i = 3; i < 6; i++)
    tmp[i] = 0.0;
  return tmp;
}

#endif
#ifndef OBSTACLE_BASED_SAMPLER_H_
#define OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Obstacle-based sampling
///
/// OBPRM samples by pushing random configurations along a random ray until they
/// change validity, keeping the best free configuration.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ObstacleBasedSampler : public SamplerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    ObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
        int _free = 1, int _coll = 0, double _step = 0,
        bool _useBBX = true, string _pointSelection = "cspace");

    ObstacleBasedSampler(XMLNode& _node);

    void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    // Generates and adds shells to their containers
    void GenerateShells(const Boundary* const _boundary,
        CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    ////////////////////////////////////////////////////////////////
    // The following was added after unification with WOBPRM code //
    ////////////////////////////////////////////////////////////////

    // Returns a CfgType at the center of mass of the MultiBody
    CfgType ChooseCenterOfMass(MultiBody* _mBody);

    // Returns a CfgType at a random vertex of the MultiBody
    CfgType ChooseRandomVertex(MultiBody* _mBody, bool _isFreeBody);

    // Returns a point inside the triangle determined by the vectors
    Vector3d ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r);

    // Chooses a random point on a random triangle (weighted by area) in the
    // MultiBody
    CfgType ChooseRandomWeightedTriangle(MultiBody* _mBody,
        bool _isFreeBody);

    // Chooses a random point in a random triangle in the MultiBody
    CfgType ChooseRandomTriangle(MultiBody* _mBody, bool _isFreeBody);

    // Chooses a random extreme vertex of the MultiBody
    CfgType ChooseExtremeVertex(MultiBody* _mBody, bool _isFreeBody);

    // Checks m_pointSelection and returns an appropriate CfgType
    virtual CfgType ChooseASample(CfgType& _cfg);

  private:

    // Returns a CfgType with the coordinates specified in the vector and no
    // rotation
    CfgType GetCfgWithParams(const Vector3d& _v);

    string m_vcLabel, m_dmLabel; // Validity checker method, distance metric method
    int m_nShellsFree, m_nShellsColl; // Number of free and collision shells
    double m_stepSize; // Step size along the random ray
    bool m_useBBX; // Is the bounding box an obstacle?
    string m_pointSelection; // Needed for the WOBPRM
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ObstacleBasedSampler<MPTraits>::
ObstacleBasedSampler(string _vcLabel, string _dmLabel,
    int _free, int _coll, double _step, bool _useBBX, string _pointSelection) :
    m_vcLabel(_vcLabel), m_dmLabel(_dmLabel), m_nShellsFree(_free),
    m_nShellsColl(_coll), m_stepSize(_step), m_useBBX(_useBBX),
    m_pointSelection(_pointSelection) {
  this->SetName("ObstacleBasedSampler");
}


template <typename MPTraits>
ObstacleBasedSampler<MPTraits>::
ObstacleBasedSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("ObstacleBasedSampler");

  m_useBBX = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity test method");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance metric method");
  m_pointSelection = _node.Read("pointSelection", false,
      "cspace", "Point selection strategy");
  m_nShellsColl = _node.Read("nShellsColl", true, 3, 0, 10,
      "Number of collision shells");
  m_nShellsFree = _node.Read("nShellsFree", true, 3, 0, 10,
      "Number of free shells");
  m_stepSize = _node.Read("stepSize", true, 0.0, 0.0, 10.0,
      "Step size used in increment of cfg position towards or away from"
      "obstacles");

  // Check if the read point_selection is valid
  if(m_pointSelection != "cspace" && m_pointSelection != "cM" &&
      m_pointSelection != "rV" && m_pointSelection != "rT" &&
      m_pointSelection != "rW" && m_pointSelection != "eV" &&
      m_pointSelection != "rV_rT" && m_pointSelection != "rV_rW" &&
      m_pointSelection != "all")
    throw ParseException(_node.Where(),
        "Invalid point selection type '" + m_pointSelection + "'."
        "Choices are: 'cspace', 'cM', 'rV', 'rT', 'rW', 'eV', 'rV_rT', 'rV_rW',"
        " and 'all'.");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tnShellsFree = " << m_nShellsFree << endl;
  _os << "\tnShellsColl = " << m_nShellsColl << endl;
  _os << "\tstepSize = " << m_stepSize << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
  _os << "\tuseBBX = " << m_useBBX << endl;
  _os << "\tpointSelectionStrategy = " << m_pointSelection << endl;
}


template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
GenerateShells(const Boundary* const _boundary,
    CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee = this->GetNameAndLabel() + "::GenerateShells()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  if(this->m_debug)
    cout << "nShellsColl = " << m_nShellsColl << endl;

  // Add free shells
  for(int i = 0; i < m_nShellsFree; i++) {
    // If the shell is valid
    if(_cFree.InBounds(_boundary) and vc->IsValid(_cFree, callee))
      _result.push_back(_cFree);
    // Get next shell
    _cFree += _incr;
  }

  // Reverse direction of _incr
  _incr = -_incr;

  // Add collision shells
  for(int i = 0; i < m_nShellsColl; i++) {
    // If the shell is valid
    if(_cColl.InBounds(_boundary) and !vc->IsValid(_cColl, callee))
      _collision.push_back(_cColl);
    // Get next shell
    _cColl += _incr;
  }
}


template <typename MPTraits>
bool
ObstacleBasedSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  Environment* env = this->GetEnvironment();
  // If the step size is unreasonable, set it to the minimum
  if(m_stepSize <= 0.0)
    m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());

  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto robot = this->GetTask()->GetRobot();

  // Old state
  CfgType c1 = ChooseASample(_cfg);
  bool c1BBox = c1.InBounds(_boundary);
  bool c1Free = vc->IsValid(c1, callee);

  // New state
  CfgType c2 = c1;
  bool c2BBox = c1BBox;
  bool c2Free = c1Free;

  // Create a random ray
  CfgType r(robot);
  r.GetRandomRay(m_stepSize, dm);
  if(r == CfgType(robot)) {
    if(this->m_debug)
      cerr << "Random ray in OBPRM Sampler is 0-valued!\
        Continuing with loop." << endl;
    return false;
  }

  // Loop until the new state is outside the bounds or the validity changes
  while(c2BBox && (c1Free == c2Free)) {
    // Copy new data to old state
    c1 = c2;
    c1BBox = c2BBox;
    c1Free = c2Free;
    // Update new state
    c2 += r;
    c2BBox = c2.InBounds(_boundary);
    c2Free = vc->IsValid(c2, callee);
  }

  // Construct Shells and generate _collision and _result
  if (!m_useBBX) { // If we treat the bounding box as an obstacle
    if(c2BBox) { // If the new state (c2) is in the bounding box
      if(c1Free) { // If the old state (c1) is  in the free C space
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        _collision.push_back(c2);
      }
      else { // The new state (c2) is in the free C space
        // Process configurations
        GenerateShells(_boundary, c2, c1, r, _result, _collision);
        _collision.push_back(c1);
      }
      return true;
    }
  } else {
    if (c1BBox) {
      if (c1Free) { // If the old state (c1) is in the free C space
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        _collision.push_back(c2);
        return true;
      }
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////
// The following was added after unification with WOBPRM code //
// ////////////////////////////////////////////////////////////////

// Returns a CfgType at the center of mass of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseCenterOfMass(MultiBody* _mBody) {
  return GetCfgWithParams(_mBody->GetCenterOfMass());
}

// Returns a CfgType at a random vertex of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomVertex(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  Vector3d x = polyhedron.GetVertexList()[(int)(DRand()*polyhedron.GetVertexList().size())];
  return GetCfgWithParams(x);
}

// Returns a point inside the triangle determined by the vectors
template <typename MPTraits>
Vector3d
ObstacleBasedSampler<MPTraits>::
ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r) {
  Vector3d u = _q - _p; // From _p to _q
  Vector3d v = _r - _p; // From _p to _r
  double s = DRand();
  double t = DRand();
  // Keep point inside the triangle
  while(s + t > 1) {
    s = DRand();
    t = DRand();
  }
  return (_p + u*s + v*t);
}

// Chooses a random point on a random triangle (weighted by area) in the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomWeightedTriangle(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  // A random fraction of the area
  double targetArea = polyhedron.GetSurfaceArea() * DRand();
  double sum = 0.0;
  int index;

  // Choose index as the triangle that first makes sum > targetArea
  for(index = -1; sum <= targetArea; index++)
    sum += polyhedron.GetPolygonList()[index + 1].GetArea();
  // Choose the triangle of the MultiBody with that index
  const GMSPolygon& poly = polyhedron.GetPolygonList()[index];
  // Choose a random point in that triangle
  const Vector3d& p = poly.GetPoint(0);
  const Vector3d& q = poly.GetPoint(1);
  const Vector3d& r = poly.GetPoint(2);
  Vector3d x = ChoosePointOnTriangle(p, q, r);

  return GetCfgWithParams(x);
}

// Chooses a random point in a random triangle in the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomTriangle(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();

  // Choose a random triangle
  const GMSPolygon& poly = polyhedron.GetPolygonList()[(int)(DRand()*polyhedron.GetPolygonList().size())];
  const Vector3d& p = poly.GetPoint(0);
  const Vector3d& q = poly.GetPoint(1);
  const Vector3d& r = poly.GetPoint(2);
  Vector3d x = ChoosePointOnTriangle(p, q, r);

  return GetCfgWithParams(x);
}

// Chooses a random extreme vertex of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseExtremeVertex(MultiBody* _mBody, bool _isFreeBody) {
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  int xyz = LRand() % 3; // 0: x, 1: y, 2: z
  int minMax = LRand() % 2; // 0: min, 1: max
  int x = 0; // Index of extreme value

  // Find extreme value
  for(size_t i = 1; i < polyhedron.GetVertexList().size(); i++)
    // minMax is an optional negation
    if(((polyhedron.GetVertexList()[i][xyz] < polyhedron.GetVertexList()[x][xyz]) + minMax) % 2)
      x = i;

  return GetCfgWithParams(polyhedron.GetVertexList()[x]);
}

// Checks m_pointSelection and returns an appropriate CfgType
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseASample(CfgType& _cfg) {
  MultiBody* mBody{nullptr};
  if(m_pointSelection != "cspace")
    mBody = this->GetEnvironment()->GetRandomObstacle();

  // cspace is for Configuration space (This is for unifying OBPRM and WOBPRM)
  if(m_pointSelection == "cspace")
    return _cfg;
  else if(m_pointSelection == "cM")
    return ChooseCenterOfMass(mBody);
  else if(m_pointSelection == "rV")
    return ChooseRandomVertex(mBody, false);
  else if(m_pointSelection == "rT")
    return ChooseRandomTriangle(mBody, false);
  else if(m_pointSelection == "rW")
    return ChooseRandomWeightedTriangle(mBody, false);
  else if(m_pointSelection == "eV")
    return ChooseExtremeVertex(mBody, false);
  else if(m_pointSelection == "cM_rV") {
    if(LRand() % 2)
      return ChooseCenterOfMass(mBody);
    else
      return ChooseRandomVertex(mBody, false);
  }
  else if(m_pointSelection == "rV_rT") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody, false);
    else
      return ChooseRandomTriangle(mBody, false);
  }
  else if(m_pointSelection == "rV_rW") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody, false);
    else
      return ChooseRandomWeightedTriangle(mBody, false);
  }
  else if(m_pointSelection == "all") {
    switch(LRand() % 5) {
      case 0:
        return ChooseCenterOfMass(mBody);
      case 1:
        return ChooseRandomVertex(mBody, false);
      case 2:
        return ChooseRandomTriangle(mBody, false);
      case 3:
        return ChooseRandomWeightedTriangle(mBody, false);
      default:
        return ChooseExtremeVertex(mBody, false);
    }
  }
  else {
    throw PMPLException("OBPRM Exception", WHERE, "Select a valid point selection type first. Exiting.");
  }
}

// Returns a CfgType with the coordinates specified in the vector and no rotation
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
GetCfgWithParams(const Vector3d& _v) {
  CfgType tmp(this->GetTask()->GetRobot());
  for(int i = 0; i < 3; i++)
    tmp[i] = _v[i];
  for(int i = 3; i < 6; i++)
    tmp[i] = 0.0;
  return tmp;
}

#endif
