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

#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Bodies/NonHolonomicMultiBody.h"
#include "MPProblem/ConfigurationSpace/State.h"
#include "Utilities/MPUtils.h"

class ActiveMultiBody;
class CollisionDetectionMethod;
class MultiBody;
class StaticMultiBody;
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
    /// @param _robot Robot to base the margin off of
    ///
    /// Reset the boundary to the minimum bounding box surrounding the obstacles
    /// increased by a margin of _d + robotRadius.
    void ResetBoundary(double _d, ActiveMultiBody* _robot);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Expand the boundary by a margin of _d + robotRadius
    /// @param _d Margin to increase bounding box
    /// @param _robot Robot to base the margin off of
    void ExpandBoundary(double _d, ActiveMultiBody* _robot);

    ///@}
    ///@name Obstacle Functions
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of Static MultiBodies
    size_t NumObstacles() const {return m_obstacles.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Requested multibody
    /// @return Pointer to static multibody
    StaticMultiBody* GetObstacle(size_t _index) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @return Pointer to random static multibody
    StaticMultiBody* GetRandomObstacle() const;

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

    shared_ptr<const WorkspaceDecomposition> GetDecomposition() const {
      return const_pointer_cast<const WorkspaceDecomposition>(m_decomposition);
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
    void ComputeResolution(vector<shared_ptr<ActiveMultiBody>> _robots);

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
    double m_timeRes{.05};           ///< Resolution for time.

    ///@}
    ///@name Models
    ///@{

    shared_ptr<Boundary> m_boundary; ///< Workspace boundary.

    vector<shared_ptr<StaticMultiBody>> m_obstacles; ///< Other multibodies.

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
  return _cfg.GetRobot()->InCSpace(_cfg.GetData(), _b) && _b->InBoundary(_cfg);
}

/*----------------------------------------------------------------------------*/

#endif
