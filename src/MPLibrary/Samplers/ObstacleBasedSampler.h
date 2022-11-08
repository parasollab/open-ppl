#ifndef OBSTACLE_BASED_SAMPLER_H_
#define OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// OBPRM samples by pushing random configurations along a random ray until they
/// change validity, keeping the best free configuration.
///
/// @todo Implement GetCenterOfMass function in the ChooseCenterOfMass class
///
/// @ingroup Samplers
/// @brief Obstacle-based sampling
////////////////////////////////////////////////////////////////////////////////

template <class MPTraits>
class ObstacleBasedSampler : virtual public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;
    typedef std::map<Robot*, const Boundary*> BoundaryMap;

    ///@}
    ///@name Construction
    ///@{

    ObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
        int _free = 1, int _coll = 0, double _step = 0,
        bool _useBBX = true, string _pointSelection = "cspace");

    ObstacleBasedSampler(XMLNode& _node);

    virtual ~ObstacleBasedSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}

  protected:

    // ///@name XML Parser
    // ///@{

    // void ParseXML(XMLNode& _node);

    // ///@}
    ///@name Helpers
    ///@{
    
    /// Main Sampler
    /// Sampler for a single robot in a single boundary
    /// @param _cfg Configuration Type
    /// @param _boundary Boundary of the environment
    /// @param _result A vector that stores Free Configuration Space
    /// @param _collision A vector that stores Collision Configuration Space
    /// @return Success or Failure of sampling
    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);
    
    /// Sampler for multiple robots in a single boundary
    /// @param _cfg Configuration Type
    /// @param _boundary Boundary of the environment
    /// @param _result A vector that stores Free Group Configuration Space
    /// @param _collision A vector that stores Collision Group Configuration Space
    /// @return Success or Failure of sampling
    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        vector<GroupCfgType>& _result, vector<GroupCfgType>& _invalid);

    /// Sampler for multiple robots in each individual boundary
    /// @param _cfg Configuration Type
    /// @param _boundaryMap BoundaryMap of the environment
    /// @param _result A vector that stores Free Group Configuration Space
    /// @param _collision A vector that stores Collision Group Configuration Space
    /// @return Success or Failure of sampling
    virtual bool Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
        vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision);

    /// Generates and adds shells to their containers 
    /// for a single robot in a single boundary case
    /// @param _boundary Boundary of the environment
    /// @param _cFree The configuration in the Free Configuration Space
    /// @param _cColl The configuration in the Collision Configuration Space
    /// @param _incr The amount of increments in the specified direction.
    /// @param _result A vector that stores Free Configuration Space
    /// @param _collision A vector that stores Collision Configuration Space
    void GenerateShells(const Boundary* const _boundary,
        CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    /// Generates and adds shells to their containers
    /// for multiple robots in a single boundary case
    /// @param _boundary Boundary of the environment
    /// @param _cFree The configuration in the Free Configuration Space
    /// @param _cColl The configuration in the Collision Configuration Space
    /// @param _incr The amount of increments in the specified direction.
    /// @param _result A vector that stores Free Group Configuration Space
    /// @param _collision A vector that stores Collision Group Configuration Space
    void GenerateShells(const Boundary* const _boundary,
        GroupCfgType& _cFree, GroupCfgType& _cColl, GroupCfgType& _incr,
        vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision);
    
    /// Generates and adds shells to their containers
    /// for multiple robots in each individual boundary case
    /// @param _boundary Boundary of the environment
    /// @param _cFree The configuration in the Free Configuration Space
    /// @param _cColl The configuration in the Collision Configuration Space
    /// @param _incr The amount of increments in the specified direction.
    /// @param _result A vector that stores Free Group Configuration Space
    /// @param _collision A vector that stores Collision Group Configuration Space
    // void GenerateShells(const Environment* const _environment,
    void GenerateShells(const BoundaryMap& _boundaryMap,
        GroupCfgType& _cFree, GroupCfgType& _cColl, GroupCfgType& _incr,
        vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision);

    /// Returns a CfgType at the center of mass of the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration corresponding to the center of mass of the Multibody
    CfgType ChooseCenterOfMass(MultiBody* _mBody);

    // Returns a CfgType at a random vertex of the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration chosen from the Random Vertex method
    CfgType ChooseRandomVertex(MultiBody* _mBody);

    /// Returns a point inside the triangle determined by the vectors
    /// @param _p The first 3d point of a triangle
    /// @param _q The second 3d point of a triangle
    /// @param _r The Third 3d point of a triangle
    /// @return Configuration chosen from the Point On Triangle method
    Vector3d ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r);

    /// Chooses a random point on a random triangle (weighted by area) in the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration chosen from the Random Weighted Triangle method
    CfgType ChooseRandomWeightedTriangle(MultiBody* _mBody);

    /// Chooses a random point in a random triangle in the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration chosen from the Random Triangle method
    CfgType ChooseRandomTriangle(MultiBody* _mBody);

    /// Chooses a random extreme vertex of the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration chosen from the Extreme Vertex method
    CfgType ChooseExtremeVertex(MultiBody* _mBody);

    /// Checks m_pointSelection and returns an appropriate CfgType
    /// @param _cfg Configuration of the sample
    /// @return Configuration of the sample
    virtual CfgType ChooseASample(CfgType& _cfg);

    /// Checks m_pointSelection and returns an appropriate GroupCfgType
    /// @param _cfg Group configuration of the sample
    /// @return Group configuration chosen from the Extreme Vertex method
    virtual GroupCfgType ChooseASample(GroupCfgType& _cfg);

    /// Call the various sample method other than cspace method
    /// @return Configuration obtained according to the m_pointSelection
    CfgType ChooseASampleOtherMethods();

    /// Returns a CfgType with the coordinates specified in the vector and no rotation
    /// @param _v Multibody of the obstacle
    /// @return Configuration Type of the sample
    CfgType GetCfgWithParams(const Vector3d& _v);

    /// Checks if the groupCfg is in the boundary map
    /// @param _groupCfg The group configuration needs to be checked
    /// @param _boundaryMap The map that pairs each robot and corresponding boundary
    /// @return true or false
    bool GroupInBounds(GroupCfgType& _groupCfg, const BoundaryMap& _boundaryMap);

    /// Checks if the groupCfg is in the boundary map
    /// @param _groupCfg The group configuration needs to be checked
    /// @param _boundary The single boundary of every robots
    /// @return true or false
    bool GroupInBounds(GroupCfgType& _groupCfg, const Boundary* const _boundary);
    ///@}

  private:

    ///@name Internal States
    ///@{
    
    string m_vcLabel, m_dmLabel; ///< Validity checker method, distance metric method
    int m_nShellsFree, m_nShellsColl; ///< Number of free and collision shells
    double m_stepSize; ///< Step size along the random ray
    bool m_useBBX; ///< Is the bounding box an obstacle?
    string m_pointSelection; ///< Needed for the WOBPRM

    ///@}
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
      m_pointSelection != "wT" && m_pointSelection != "eV" &&
      m_pointSelection != "rV_rT" && m_pointSelection != "rV_wT" &&
      m_pointSelection != "cM_rV" &&
      m_pointSelection != "all")
    throw ParseException(_node.Where(),
        "Invalid point selection type '" + m_pointSelection + "'."
        "Choices are: 'cspace', 'cM', 'rV', 'rT', 'wT', 'eV', 'rV_rT', 'rV_wT', 'cM_rV'"
        " and 'all'.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

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

/*-------------------------- Helpers --------------------------*/

// Sampling Rules
template <typename MPTraits>
bool
ObstacleBasedSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  // Call the environment
  Environment* env = this->GetEnvironment();
  
  // If the step size is unreasonable, set it to the minimum
  if(m_stepSize <= 0.0)
    m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());

  // Define a validity checker, a distance metric, and a task
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto robot = this->GetTask()->GetRobot();

  // Old state that is generated by point selection strategies
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
  // If we treat the bounding box as an obstacle
  if (!m_useBBX) { 
    // If the new state (c2) is in the bounding box
    if (c2BBox) { 
      // If the old state (c1) is  in the free C space
      if (c1Free) { 
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        // Add the new state (c2) to the collision vecter
        _collision.push_back(c2);
      }
      // If the new state (c2) is in the free C space
      else { 
        // Process configurations
        GenerateShells(_boundary, c2, c1, r, _result, _collision);
        // Add the old state (c1) to the collision vecter
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
        // Add the new state (c2) to the collision vecter
        _collision.push_back(c2);
        return true;
      }
    }
  }
  return false;
}


template <typename MPTraits>
bool
ObstacleBasedSampler<MPTraits>::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision) {

  // Call the environment
  Environment* env = this->GetEnvironment();

  // If the step size is unreasonable, set it to the minimum
  if(m_stepSize <= 0.0)
    m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());

  // Define a validity checker, a distance metric, and a groupRoadmap
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto groupRoadmap = this->GetGroupRoadmap();

  // Old state that is generated by point selection strategies
  GroupCfgType c1 = ChooseASample(_cfg);
  bool c1BBox = c1.InBounds(_boundary);
  bool c1Free = vc->IsValid(c1, callee);

  // New state
  GroupCfgType c2 = c1;
  bool c2BBox = c1BBox;
  bool c2Free = c1Free;

  // Create a random ray
  GroupCfgType r(groupRoadmap);
  // r.GetRandomRay(m_stepSize, dm);
  for(size_t j = 0; j < r.GetNumRobots(); ++j) {
    r.GetRobotCfg(j).GetRandomRay(m_stepSize, dm);
      // for(size_t i = 0; i < r.GetRobotCfg(j).DOF(); ++i)
          // r.GetRobotCfg(j)[i] = GRand();
  }

  // dm->ScaleCfg(m_stepSize, *this);


  // Loop until the new state is outside the bounds or the validity changes
  while(c2BBox && (c1Free == c2Free)) {
    // Copy new data to old state
    c1 = c2;
    c1BBox = c2BBox;
    c1Free = c2Free;
    // Update new state
    c2 += r;
    c2BBox = GroupInBounds(c2, _boundary);
    c2Free = vc->IsValid(c2, callee);
  }

  // Construct Shells and generate _collision and _result
  // If we treat the bounding box as an obstacle
  if (!m_useBBX) { 
    // If the new state (c2) is in the bounding box
    if (c2BBox) { 
      // If the old state (c1) is  in the free C space
      if (c1Free) { 
        // Reverse direction of r
        r -= r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        // Add the new state (c2) to the collision vecter
        _collision.push_back(c2);
      }
      // If the new state (c2) is in the free C space
      else { 
        // Process configurations
        GenerateShells(_boundary, c2, c1, r, _result, _collision);
        // Add the old state (c1) to the collision vecter
        _collision.push_back(c1);
      }
      return true;
    }
  } else {
    if (c1BBox) {
      if (c1Free) { // If the old state (c1) is in the free C space
        // Reverse direction of r
        r -= r;
        // Process configurations
        GenerateShells(_boundary, c1, c2, r, _result, _collision);
        // Add the new state (c2) to the collision vecter
        _collision.push_back(c2);
        return true;
      }
    }
  }
  return false;
}


template <typename MPTraits>
bool
ObstacleBasedSampler<MPTraits>::
Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
    vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision) {
  
  // Call the environment
  Environment* env = this->GetEnvironment();

  // If the step size is unreasonable, set it to the minimum
  if(m_stepSize <= 0.0)
    m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());

  // Define a validity checker, a distance metric, and a groupRoadmap
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto groupRoadmap = this->GetGroupRoadmap();
  
  // Old state that is generated by point selection strategies
  GroupCfgType c1 = ChooseASample(_cfg);
  bool c1BBox = c1.InBounds(env);
  bool c1Free = vc->IsValid(c1, callee);

  // New state
  GroupCfgType c2 = c1;
  bool c2BBox = c1BBox;
  bool c2Free = c1Free;

  // Create a random ray
  GroupCfgType r(groupRoadmap);
  // r.GetRandomRay(m_stepSize, dm);
  for(size_t j = 0; j < r.GetNumRobots(); ++j) {
    r.GetRobotCfg(j).GetRandomRay(m_stepSize, dm);
      // for(size_t i = 0; i < r.GetRobotCfg(j).DOF(); ++i)
          // r.GetRobotCfg(j)[i] = GRand();
  }

  // Loop until the new state is outside the bounds or the validity changes
  while(c2BBox && (c1Free == c2Free)) {
    // Copy new data to old state
    c1 = c2;
    c1BBox = c2BBox;
    c1Free = c2Free;
    // Update new state
    c2 += r;
    c2BBox = GroupInBounds(c2, _boundaryMap);
    c2Free = vc->IsValid(c2, callee);
  }

  // Construct Shells and generate _collision and _result
  // If we treat the bounding box as an obstacle
  if (!m_useBBX) { 
    // If the new state (c2) is in the bounding box
    if (c2BBox) { 
      // If the old state (c1) is  in the free C space
      if (c1Free) { 
        // Reverse direction of r
        r -= r;
        // Process configurations
        GenerateShells(_boundaryMap, c1, c2, r, _result, _collision);
        // Add the new state (c2) to the collision vecter
        _collision.push_back(c2);
      }
      // If the new state (c2) is in the free C space
      else { 
        // Process configurations
        GenerateShells(_boundaryMap, c2, c1, r, _result, _collision);
        // Add the old state (c1) to the collision vecter
        _collision.push_back(c1);
      }
      return true;
    }
  } else {
    if (c1BBox) {
      if (c1Free) { // If the old state (c1) is in the free C space
        // Reverse direction of r
        r -= r;
        // Process configurations
        GenerateShells(_boundaryMap, c1, c2, r, _result, _collision);
        // Add the new state (c2) to the collision vecter
        _collision.push_back(c2);
        return true;
      }
    }
  }
  return false;
}

template <typename MPTraits>
bool
ObstacleBasedSampler<MPTraits>::
GroupInBounds(GroupCfgType& _groupCfg, const BoundaryMap& _boundaryMap) {
  auto groupTask = this->GetGroupTask();
  for (auto task : *groupTask) {
    auto robot = task.GetRobot();
    CfgType cfg = _groupCfg.GetRobotCfg(robot);
    auto boundary = _boundaryMap.at(robot);
    if (!cfg.InBounds(boundary))
      return false;
  }
  return true;
}


template <typename MPTraits>
bool
ObstacleBasedSampler<MPTraits>::
GroupInBounds(GroupCfgType& _groupCfg, const Boundary* const _boundary) {
  auto groupTask = this->GetGroupTask();
  for (auto task : *groupTask) {
    auto robot = task.GetRobot();
    CfgType cfg = _groupCfg.GetRobotCfg(robot);
    if (!cfg.InBounds(_boundary))
      return false;
  }
  return true;
}

// Shell Generator
template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
GenerateShells(const Boundary* const _boundary,
    CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee = this->GetNameAndLabel() + "::GenerateShells()";

  // Call a validity checker
  auto vc = this->GetValidityChecker(m_vcLabel);
  if(this->m_debug)
    cout << "nShellsColl = " << m_nShellsColl << endl;

  // Add "free" shells
  for(int i = 0; i < m_nShellsFree; i++) {
    // If the shell is valid (If it is in the free C space)
    if(_cFree.InBounds(_boundary) and vc->IsValid(_cFree, callee))
      _result.push_back(_cFree);  
    // Get next shell
    _cFree += _incr;
  }

  // Reverse direction of _incr
  _incr = -_incr;

  // Add "collision" shells
  for(int i = 0; i < m_nShellsColl; i++) {
    // If the shell is valid (If it is in the collision C space)
    if(_cColl.InBounds(_boundary) and !vc->IsValid(_cColl, callee))
      _collision.push_back(_cColl);
    // Get next shell
    _cColl += _incr;
  }
}

template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
GenerateShells(const Boundary* const _boundary,
    GroupCfgType& _cFree, GroupCfgType& _cColl, GroupCfgType& _incr,
    vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision) {

  string callee = this->GetNameAndLabel() + "::GenerateShells()";

  // Call a validity checker
  auto vc = this->GetValidityChecker(m_vcLabel);
  if(this->m_debug)
    cout << "nShellsColl = " << m_nShellsColl << endl;

  // Add "free" shells
  for(int i = 0; i < m_nShellsFree; i++) {
    // If the shell is valid (If it is in the free C space)
    if(GroupInBounds(_cFree, _boundary) and vc->IsValid(_cFree, callee))
      _result.push_back(_cFree);  
    // Get next shell
    _cFree += _incr;
  }

  // Reverse direction of _incr
  _incr -= _incr;

  // Add "collision" shells
  for(int i = 0; i < m_nShellsColl; i++) {
    // If the shell is valid (If it is in the collision C space)
    if(GroupInBounds(_cColl, _boundary) and !vc->IsValid(_cColl, callee))
      _collision.push_back(_cColl);
    // Get next shell
    _cColl += _incr;
  }
}

template <typename MPTraits>
void
ObstacleBasedSampler<MPTraits>::
GenerateShells(const BoundaryMap& _boundaryMap,
    GroupCfgType& _cFree, GroupCfgType& _cColl, GroupCfgType& _incr,
    vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision) {

  string callee = this->GetNameAndLabel() + "::GenerateShells()";

  // Call a validity checker
  auto vc = this->GetValidityChecker(m_vcLabel);
  if(this->m_debug)
    cout << "nShellsColl = " << m_nShellsColl << endl;

  // Add "free" shells
  for(int i = 0; i < m_nShellsFree; i++) {
    // If the shell is valid (If it is in the free C space)
    if(GroupInBounds(_cFree, _boundaryMap) and vc->IsValid(_cFree, callee))
    // if(_cFree.InBounds(_environment) and vc->IsValid(_cFree, callee))
      _result.push_back(_cFree);  
    // Get next shell
    _cFree += _incr;
  }

  // Reverse direction of _incr
  _incr -= _incr;

  // Add "collision" shells
  for(int i = 0; i < m_nShellsColl; i++) {
    // If the shell is valid (If it is in the collision C space)
    if(GroupInBounds(_cColl, _boundaryMap) and vc->IsValid(_cColl, callee))
    // if(_cColl.InBounds(_environment) and !vc->IsValid(_cColl, callee))
      _collision.push_back(_cColl);
    // Get next shell
    _cColl += _incr;
  }
}


// Returns a CfgType of the center of mass
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseCenterOfMass(MultiBody* _mBody) {
  // return GetCfgWithParams(_mBody->poly());
  size_t body = LRand() % _mBody->GetNumBodies();
  const GMSPolyhedron& polyhedron = _mBody->GetBody(body)->GetWorldPolyhedron();
  Vector3d x = polyhedron.GetCentroid();
  return GetCfgWithParams(x);
}

// Returns a CfgType at a random vertex of the MultiBody
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseRandomVertex(MultiBody* _mBody) {
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
ChooseRandomWeightedTriangle(MultiBody* _mBody) {
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
ChooseRandomTriangle(MultiBody* _mBody) {
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
ChooseExtremeVertex(MultiBody* _mBody) {
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

// Returns the CfgType according to the sampling type
template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseASample(CfgType& _cfg) {
  if(m_pointSelection == "cspace")
    return _cfg;
  else
    return ChooseASampleOtherMethods();
}

template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::GroupCfgType
ObstacleBasedSampler<MPTraits>::
ChooseASample(GroupCfgType& _cfg) {
  if(m_pointSelection == "cspace") {
    return _cfg;
  }
  else {
    GroupCfgType groupCfg;
    for (auto robot : _cfg.GetRobots()) {
      groupCfg.SetRobotCfg(robot, ChooseASampleOtherMethods());
    }
    return groupCfg;
  }
}

template <typename MPTraits>
typename ObstacleBasedSampler<MPTraits>::CfgType
ObstacleBasedSampler<MPTraits>::
ChooseASampleOtherMethods() {
  MultiBody* mBody{nullptr};
  if(m_pointSelection != "cspace")
    mBody = this->GetEnvironment()->GetRandomObstacle();

  if(m_pointSelection == "cM")
    return ChooseCenterOfMass(mBody);
  else if(m_pointSelection == "rV")
    return ChooseRandomVertex(mBody);
  else if(m_pointSelection == "rT")
    return ChooseRandomTriangle(mBody);
  else if(m_pointSelection == "wT")
    return ChooseRandomWeightedTriangle(mBody);
  else if(m_pointSelection == "eV")
    return ChooseExtremeVertex(mBody);
  else if(m_pointSelection == "cM_rV") {
    if(LRand() % 2)
      return ChooseCenterOfMass(mBody);
    else
      return ChooseRandomVertex(mBody);
  }
  else if(m_pointSelection == "rV_rT") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody);
    else
      return ChooseRandomTriangle(mBody);
  }
  else if(m_pointSelection == "rV_wT") {
    if(LRand() % 2)
      return ChooseRandomVertex(mBody);
    else
      return ChooseRandomWeightedTriangle(mBody);
  }
  else if(m_pointSelection == "all") {
    switch(LRand() % 5) {
      case 0:
        return ChooseCenterOfMass(mBody);
      case 1:
        return ChooseRandomVertex(mBody);
      case 2:
        return ChooseRandomTriangle(mBody);
      case 3:
        return ChooseRandomWeightedTriangle(mBody);
      default:
        return ChooseExtremeVertex(mBody);
    }
  }
  else {
    throw PMPLException("OBPRM Exception", WHERE, "Select a valid point selection type first. Exiting.");
  }
}

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
