#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Transformation.h"
using namespace mathtool;

#include "Boundary.h"
#include "NonHolonomicMultiBody.h"
#include "Cfg/CfgMultiRobot.h"
#include "Cfg/State.h"
#include "Utilities/MPUtils.h"

class ActiveMultiBody;
class CollisionDetectionMethod;
class MultiBody;
class StaticMultiBody;
class SurfaceMultiBody;
class WorkspaceDecomposition;

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

    ///@name Local Types
    ///@{

    typedef function<shared_ptr<WorkspaceDecomposition>(const Environment*)>
        DecompositionFunction;

    ///@}
    ///@name Construction
    ///@{

    Environment() = default;
    Environment(XMLNode& _node);
    virtual ~Environment() = default;

    ///@}
    ///@name I/O
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Filename from which environment came
    const string& GetEnvFileName() const {return m_filename;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Parse environment file
    /// @param _filename Filename
    void Read(string _filename);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Prints environment resolutions and boundary information
    /// @param _os Output stream
    void Print(ostream& _os) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Output environment to .env file format
    /// @param _os Output stream
    void Write(ostream& _os);

    ///@}
    ///@name Resolutions
    ///@{

    double GetPositionRes() const {return m_positionRes;}
    void SetPositionRes(double _res) {m_positionRes = _res;}

    double GetOrientationRes() const {return m_orientationRes;}
    void SetOrientationRes(double _res) {m_orientationRes = _res;}

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    double GetRDRes() const {return m_rdRes;} ///< Reachable distance resolution.
#endif

    double GetTimeRes() const {return m_timeRes;}

    ///@}
    ///@name Boundary Functions
    ///@{

    shared_ptr<Boundary> GetBoundary() const {return m_boundary;}
    void SetBoundary(shared_ptr<Boundary> _b) {m_boundary = _b;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Test if configuration in inside of the workspace and satisfies
    ///        physical robot constraints.
    /// @tparam CfgType Configuration class type
    /// @param _cfg Configuration
    /// @param _b Workspace region boundary
    /// @return True if inside workspace and satisfying contraints
    ///
    /// Test whether input configuration satisfies joint constraints  (i.e., is
    /// inside of C-Space) and lies inside of the workspace boundary (i.e., the
    /// robot at that configuration is inside of the workspace).
    template<class CfgType>
    bool InBounds(const CfgType& _cfg, shared_ptr<Boundary> _b);

    /// @overload
    template<class CfgType>
    bool InBounds(const CfgType& _cfg) {return InBounds(_cfg, m_boundary);}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Resize the boundary to a margin away from the obstacles
    /// @param _d Margin to increase minimum bounding box
    /// @param _robotIndex Robot to base the margin off of
    ///
    /// Reset the boundary to the minimum bounding box surrounding the obstacles
    /// increased by a margin of _d + robotRadius.
    void ResetBoundary(double _d, size_t _robotIndex);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Expand the boundary by a margin of _d + robotRadius
    /// @param _d Margin to increase bounding box
    /// @param _robotIndex Robot to base the margin off of
    void ExpandBoundary(double _d, size_t _robotIndex);

    ///@}
    ///@name Multibody Functions
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of Active MultiBodies
    size_t NumRobots() const {return m_robots.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of Static MultiBodies
    size_t NumObstacles() const {return m_obstacles.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of Surface MultiBodies
    size_t NumSurfaces() const {return m_surfaces.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Requested multibody
    /// @return Pointer to active multibody
    shared_ptr<ActiveMultiBody> GetRobot(const size_t _index) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Requested multibody
    /// @return Pointer to static multibody
    shared_ptr<StaticMultiBody> GetObstacle(size_t _index) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Requested multibody
    /// @return Pointer to surface multibody
    shared_ptr<SurfaceMultiBody> GetSurface(size_t _index) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @return Pointer to random static multibody
    shared_ptr<StaticMultiBody> GetRandomObstacle() const;

    ////////////////////////////////////////////////////////////////////////////
    /// @return Index to random navigable surface. -1 means base surface.
    ssize_t GetRandomSurfaceIndex();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add Obstacle to environment
    /// @param _dir Directory for geometry file
    /// @param _filename Geometry filename
    /// @param _t Transformation of object
    /// @return (index, pointer) pair to newly created obstacle
    pair<size_t, shared_ptr<StaticMultiBody>> AddObstacle(const string& _dir,
        const string& _filename, const Transformation& _t = Transformation());

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Remove obstacle from environment
    /// @param _position Index in m_obstacleBodies to be removed
    void RemoveObstacle(size_t _position);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Remove obstacle from environment
    /// @param _obst Obstacle to be removed
    void RemoveObstacle(shared_ptr<StaticMultiBody> _obst);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute a mapping of the obstacle vertices.
    /// @return A map from obstacle points to obstacle indexes.
    map<Vector3d, vector<size_t>> ComputeObstacleVertexMap() const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build collision detection models for external libraries
    void BuildCDStructure();

    ///@}
    ///@name Decomposition
    ///@{

    const WorkspaceDecomposition* GetDecomposition() const {
      return m_decomposition.get();
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute a decomposition of the workspace.
    /// @param[in] _f The decomposition function to use.
    void Decompose(DecompositionFunction&& _f) {m_decomposition = _f(this);}

    ///@}

  protected:

    ///@name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Initialize the pseudo point robot. This is a robot with index -1
    ///        whos body is a single tiny, open triangle.
    void InitializePointRobot();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read boundary information
    /// @param _is Input stream
    /// @param _cbs Counting stream buffer for accurate error reporting
    void ReadBoundary(istream& _is, CountingStreamBuffer& _cbs);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Write boundary information
    /// @param _os Output stream
    void WriteBoundary(ostream& _os);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Automatically compute resolutions
    ///
    /// ComputeResolution, if m_posRes is < 0 then auto compute the resolutions
    /// based on minimum of max body spans multiplied by @c m_positionResFactor.
    /// Reachable distance resolution is computed based upon input res
    /// multiplied by the number of joints.
    void ComputeResolution();

    ////////////////////////////////////////////////////////////////////////////
    /// @tparam CfgType Configuration class type
    /// @brief Determine if @p _cfg is within physical robot contraints
    /// @param _cfg Configuration
    /// @param _b Workspace region of environment
    /// @return True if @p _cfg is inside physical robot constraints
    template<class CfgType>
    bool InCSpace(const CfgType& _cfg, shared_ptr<Boundary> _b);

    ////////////////////////////////////////////////////////////////////////////
    /// @tparam CfgType Configuration class type
    /// @brief Determine if @p _cfg is within workspace boundary
    /// @param _cfg Configuration
    /// @param _b Workspace region of environment
    /// @return True if @p _cfg is inside workspace boundary
    bool InWSpace(const Cfg& _cfg, shared_ptr<Boundary> _b);

    ///@}
    ///@name File Info
    ///@{

    string m_filename;      ///< Which file did this environment come from?
    string m_modelDataDir;  ///< Directory where environment file is located.
    bool m_saveDofs{false}; ///< Should we save the dof information to a file?

    ///@}
    ///@name Resolution Info
    ///@{

    double m_positionRes{.05};       ///< Positional resolution of movement.
    double m_positionResFactor;      ///< Factor of body span to use as auto-
                                     ///< computed positional resolution.
    double m_orientationRes{.05};    ///< Rotational resolution of movement.
    double m_rdRes{.05};             ///< Resolution for movement in RD space.
    double m_timeRes{.05};           ///< Resolution for time.

    ///@}
    ///@name Models
    ///@{

    shared_ptr<Boundary> m_boundary; ///< Workspace boundary.

    shared_ptr<ActiveMultiBody> m_pointRobot;        ///< A point robot.
    vector<shared_ptr<ActiveMultiBody>> m_robots;    ///< Robots.
    vector<shared_ptr<StaticMultiBody>> m_obstacles; ///< Other multibodies.
    vector<shared_ptr<SurfaceMultiBody>> m_surfaces; ///< Surfaces.

    ///@}
    ///@name Decomposition
    ///@{

    /// A workspace decomposition model describes a partitioning of free space.
    shared_ptr<WorkspaceDecomposition> m_decomposition;

    ///@}
};

/*----------------------------- Boundary Functions ---------------------------*/

template<class CfgType>
bool
Environment::
InBounds(const CfgType& _cfg, shared_ptr<Boundary> _b) {
  return InCSpace(_cfg, _b) && InWSpace(_cfg, _b);
}


template<>
bool
Environment::
InBounds<CfgMultiRobot>(const CfgMultiRobot& _cfg, shared_ptr<Boundary> _b);

/*--------------------------------- Helpers ----------------------------------*/

template<class CfgType>
bool
Environment::
InCSpace(const CfgType& _cfg, shared_ptr<Boundary> _b) {
  size_t activeBodyIndex = _cfg.GetRobotIndex();
  return GetRobot(activeBodyIndex)->InCSpace(_cfg.GetData(), _b);
}


#ifdef PMPState
template<>
bool
Environment::
InCSpace<State>(const State& _cfg, shared_ptr<Boundary> _b);
#endif

/*----------------------------------------------------------------------------*/

#endif
