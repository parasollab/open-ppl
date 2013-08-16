#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include "MPProblem/Boundary.h"
#include "MPProblem/Robot.h"
#include "MPProblem/Geometry/MultiBody.h"
#include "Utilities/MPUtils.h"
#include "Graph.h"

class Environment {
  public:

    Environment();
    Environment(XMLNodeReader& _node);
    virtual ~Environment();

    ///////////////////////////////////////////////////
    //I/O
    ///////////////////////////////////////////////////

    //Return the file from which the environment came.
    const string& GetEnvFileName() const {return m_filename;}

    void Read(string _filename);
    void PrintOptions(ostream& _os);
    void Write(ostream & _os);

    //////////////////////////////////////////////////////////
    //Resolution
    //////////////////////////////////////////////////////////

    //Return the resolution for translation discretization.
    double GetPositionRes() const { return m_positionRes; }
    void SetPositionRes(double pRes) {m_positionRes=pRes;}

    //Return the resolution for rotation discretization.
    double GetOrientationRes() const { return m_orientationRes; }
    void SetOrientationRes(const double rRes) {m_orientationRes=rRes;}

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    //return the resolution for reachable distance discretization.
    double GetRDRes() const {return m_rdRes;}
#endif

    //ComputeResolution, if _posRes is <0, auto compute
    //the resolutions based on min_max body spans.
    void ComputeResolution(double _positionResFactor = 0.05);

    ///////////////////////////////////////////////////////////
    //Boundary
    ///////////////////////////////////////////////////////////

    shared_ptr<Boundary> GetBoundary() const {return m_boundary;}
    void SetBoundary(shared_ptr<Boundary> _b) {m_boundary = _b;}

    //test whether input configuration satisfies joint constraints  (i.e., is
    //inside of C-Space) and lies inside of the workspace boundary (i.e., the
    //robot at that configuration is inside of the workspace).
    bool InBounds(const Cfg& _cfg) {return InBounds(_cfg, m_boundary);}
    bool InBounds(const Cfg& _cfg, shared_ptr<Boundary> _b);

    //access the possible range of values for the _i th DOF
    pair<double, double> GetRange(size_t _i) {return GetRange(_i, m_boundary);}
    pair<double, double> GetRange(size_t _i, shared_ptr<Boundary> _b);

    //reset the boundary to the minimum bounding box surrounding the obstacles
    //increased by a margin of _d + robotRadius
    void ResetBoundary(double _d, size_t _robotIndex);

    ///////////////////////////////////////////////////////////
    //MultiBodies
    ///////////////////////////////////////////////////////////

    size_t GetActiveBodyCount() const {return m_activeBodies.size();}
    size_t GetObstacleCount() const {return m_obstacleBodies.size();}
    size_t GetUsableMultiBodyCount() const {return m_usableMultiBodies.size();}
    size_t GetNavigableSurfacesCount() const {return m_navigableSurfaces.size();}

    //Returns a pointer to MultiBody according to this given index.
    //If this index is out of the boundary of list, NULL will be returned.
    shared_ptr<MultiBody> GetMultiBody(size_t _index) const;

    //Return a pointer to MultiBody in m_navigableSurfaces given index.
    //If this index is out of the boundary of list, NULL will be returned.
    shared_ptr<MultiBody> GetNavigableSurface(size_t _index) const;

    shared_ptr<MultiBody> GetRandomObstacle() const;
    size_t GetRandomNavigableSurfaceIndex();


    //AddObstacle
    //_modelFileName: path to .obj file that specifies geometry of the obstacle
    //_where: a 6 dof vector specifying position and orientation of the geometry:
    //      (x, y, z, rotation about X, rotation about Y, rotation about Z)
    //return value: obstacle's index in m_obstacleBodies on success, -1 otherwise
    int AddObstacle(string _modelFileName, const Transformation& _where, const vector<cd_predefined>& _cdTypes);

    //RemoveObstacleAt
    //Removes multibody stored at position given in m_obstacleBodies
    void RemoveObstacleAt(size_t position);

    void BuildCDstructure(cd_predefined cdtype);

  protected:

    void ReadBoundary(istream& _is);

    //BuildRobotStructure, builds a robot graph which determines DOFs for a given robot
    //In an environment with multiple active bodies, for now this function will assume they all have the same DOFs
    //until PMPL is changed later to support multiple roadmaps for heterogeneous systems. That is, this function assumes
    //that if there is a multiagent sim going on, the agents are homogenous
    void BuildRobotStructure();

    //determine if _cfg is inside of the C-space defined by this workspace
    //boundary, and the joint limits of the robot
    bool InCSpace(const Cfg& _cfg, shared_ptr<Boundary> _b);

    //determine if a robot placed at _cfg lies entirely inside the workspace
    //boundary _b
    bool InWSpace(const Cfg& _cfg, shared_ptr<Boundary> _b);

    string m_filename; //which file did this environment come from
    bool m_saveDofs; //should we save the dof information to a file

    double m_positionRes; //positional resolution of movement
    double m_orientationRes; //rotational resolution of movement
    double m_rdRes; //resolution for movement in RD space

    //Robot Graph (for single Active Multibody instance. Used for automatically
    //determining C-space
    typedef stapl::sequential::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES, size_t> RobotGraph;
    RobotGraph m_robotGraph;
    vector<Robot> m_robots; //computed set of robots in C-space for a single active body

    shared_ptr<Boundary> m_boundary; //boundary of the workspace

    vector<shared_ptr<MultiBody> > m_activeBodies; //robots
    vector<shared_ptr<MultiBody> > m_obstacleBodies; //all other multibodies
    vector<shared_ptr<MultiBody> > m_usableMultiBodies; //obstacles and robot (for collision detection)
    vector<shared_ptr<MultiBody> > m_navigableSurfaces; //surfaces
};

inline
shared_ptr<MultiBody>
Environment::GetMultiBody(size_t _index) const {
  if(_index < m_usableMultiBodies.size())
    return m_usableMultiBodies[_index];
  else {
    cerr << "Error:Cannot access MultiBody with index " << _index
      << ". Possible indices are [0, " << m_usableMultiBodies.size() << ")." << endl;
    exit(1);
  }
}

inline
shared_ptr<MultiBody> Environment::GetNavigableSurface(size_t _index) const {
  if(_index < m_navigableSurfaces.size())
    return m_navigableSurfaces[_index];
  else {
    cerr << "Error:Cannot access NavigableSurface with index " << _index
      << ". Possible indices are [0, " << m_navigableSurfaces.size() << ")." << endl;
    exit(1);
  }
}

#endif
