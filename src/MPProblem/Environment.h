#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include "Cfg/CfgMultiRobot.h"
#include "MPProblem/Boundary.h"
//#include "MPProblem/Geometry/MultiBody.h"
#include "Utilities/MPUtils.h"

class ActiveMultiBody;
class MultiBody;
class StaticMultiBody;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Workspace for the motion planning problem.
///
/// The Environment is essentially the workspace of the motion planning problem.
/// We define a workspace as a set of MultiBody which are essentially either
/// robot or obstacle geometries and a Boundary to define the
/// sampling/exploration region for the planner.
////////////////////////////////////////////////////////////////////////////////
class Environment {
  public:

    Environment();
    Environment(XMLNode& _node);
    virtual ~Environment();

    ///////////////////////////////////////////////////
    //I/O
    ///////////////////////////////////////////////////

    //Return the file from which the environment came.
    const string& GetEnvFileName() const {return m_filename;}

    void Read(string _filename);
    void Print(ostream& _os) const;
    void Write(ostream& _os);

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

    //ComputeResolution, if _posRes is <0, auto&  compute
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
    // TODO this is a work around for CfgMultiRobot class InBounds check
    bool InBounds(const CfgMultiRobot& _cfg) {return InBounds(_cfg, m_boundary);}
    bool InBounds(const CfgMultiRobot& _cfg, shared_ptr<Boundary> _b);

    //reset the boundary to the minimum bounding box surrounding the obstacles
    //increased by a margin of _d + robotRadius
    void ResetBoundary(double _d, size_t _robotIndex);

    //expand the boundary by a margin of _d + robotRadius
    void ExpandBoundary(double _d, size_t _robotIndex);

    ///////////////////////////////////////////////////////////
    //MultiBodies
    ///////////////////////////////////////////////////////////

    size_t GetActiveBodyCount() const {return m_activeBodies.size();}
    size_t GetObstacleCount() const {return m_obstacleBodies.size();}
    size_t GetUsableMultiBodyCount() const {return m_usableMultiBodies.size();}
    size_t GetNavigableSurfacesCount() const {return m_navigableSurfaces.size();}

    //Returns a pointer to ActiveBody according to this given index.
    //If this index is out of the boundary of list, NULL will be returned.
    shared_ptr<ActiveMultiBody> GetActiveBody(size_t _index) const;

    //Returns a pointer to StaticBody according to this given index.
    //If this index is out of the boundary of list, NULL will be returned.
    shared_ptr<StaticMultiBody> GetStaticBody(size_t _index) const;

    //Returns a pointer to MultiBody according to this given index.
    //If this index is out of the boundary of list, NULL will be returned.
    shared_ptr<MultiBody> GetMultiBody(size_t _index) const;

    //Return a pointer to MultiBody in m_navigableSurfaces given index.
    //If this index is out of the boundary of list, NULL will be returned.
    shared_ptr<StaticMultiBody> GetNavigableSurface(size_t _index) const;

    shared_ptr<MultiBody> GetRandomObstacle() const;
    size_t GetRandomNavigableSurfaceIndex();


    //AddObstacle
    //_modelFileName: path to .obj file that specifies geometry of the obstacle
    //_where: a 6 dof vector specifying position and orientation of the geometry:
    //      (x, y, z, rotation about X, rotation about Y, rotation about Z)
    //return value: obstacle's index in m_obstacleBodies on success, -1 otherwise
    //int AddObstacle(string _modelFileName, const Transformation& _where, const vector<cd_predefined>& _cdTypes);

    //RemoveObstacleAt
    //Removes multibody stored at position given in m_obstacleBodies
    //void RemoveObstacleAt(size_t position);

    void BuildCDstructure(cd_predefined _cdtype);

  protected:

    void ReadBoundary(istream& _is, CountingStreamBuffer& _cbs);

    //determine if _cfg is inside of the C-space defined by this workspace
    //boundary, and the joint limits of the robot
    bool InCSpace(const Cfg& _cfg, shared_ptr<Boundary> _b);

    //determine if a robot placed at _cfg lies entirely inside the workspace
    //boundary _b
    bool InWSpace(const Cfg& _cfg, shared_ptr<Boundary> _b);

    string m_filename;     ///< Which file did this environment come from
    string m_modelDataDir; ///< Directory where environment file is located
    bool m_saveDofs;       ///< Should we save the dof information to a file

    double m_positionRes;    ///< Positional resolution of movement
    double m_orientationRes; ///< Rotational resolution of movement
    double m_rdRes;          ///< Resolution for movement in RD space

    shared_ptr<Boundary> m_boundary; ///< Boundary of the workspace

    vector<shared_ptr<ActiveMultiBody>> m_activeBodies; ///< Robots
    vector<shared_ptr<StaticMultiBody>>
      m_navigableSurfaces;                              ///< Surfaces
    vector<shared_ptr<StaticMultiBody>>
      m_obstacleBodies;                                 ///< Other multibodies
    vector<shared_ptr<MultiBody>> m_usableMultiBodies;  ///< All multibodies
};


#endif
