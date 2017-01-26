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
#include "Utilities/MPUtils.h"

class ActiveMultiBody;
class CollisionDetectionMethod;
class MultiBody;
class Robot;
class StaticMultiBody;
class WorkspaceDecomposition;


////////////////////////////////////////////////////////////////////////////////
/// Workspace for the motion planning problem.
///
/// @details The Environment is essentially the workspace of a motion planning
///          problem, and includes a boundary and zero or more obstacles.
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

    virtual ~Environment();

    ///@}
    ///@name I/O
    ///@{

    /// @return Filename from which environment came
    const string& GetEnvFileName() const {return m_filename;}

    /// Parse environment file.
    /// @param _filename The name of the file to read.
    void Read(string _filename);

    /// Print environment resolutions and boundary information.
    /// @param _os The output stream to print to.
    void Print(ostream& _os) const;

    /// Output environment to .env file format.
    /// @param _os The output stream to write to.
    void Write(ostream& _os);

    ///@}
    ///@name Resolutions
    ///@{

    /// Automatically compute resolutions.
    ///
    /// ComputeResolution, if m_posRes is < 0 then auto compute the resolutions
    /// based on minimum of max body spans multiplied by @c m_positionResFactor.
    /// Reachable distance resolution is computed based upon input res
    /// multiplied by the number of joints.
    void ComputeResolution(const vector<Robot*>& _robots);

    double GetPositionRes() const {return m_positionRes;}
    void SetPositionRes(double _res) {m_positionRes = _res;}

    double GetOrientationRes() const {return m_orientationRes;}
    void SetOrientationRes(double _res) {m_orientationRes = _res;}

    double GetTimeRes() const {return m_timeRes;}

    ///@}
    ///@name Boundary Functions
    ///@{

    const Boundary* GetBoundary() const {return m_boundary;}
    void SetBoundary(Boundary* const _b) {m_boundary = _b;}

    /// Test if a configuration lies inside of the workspace and satisfies
    /// physical robot constraints.
    /// @param _cfg The configuration to test.
    /// @param _b The enclosing boundary of the workspace region.
    /// @return True if the configuration is inside the boundary and satisfies
    ///         contraints.
    ///
    /// Test whether input configuration satisfies joint constraints  (i.e., is
    /// inside of C-Space) and lies inside of the workspace boundary (i.e., the
    /// robot at that configuration is inside of the workspace).
    template<class CfgType>
    bool InBounds(const CfgType& _cfg, const Boundary* const _b);

    /// @overload
    template<class CfgType>
    bool InBounds(const CfgType& _cfg) {return InBounds(_cfg, m_boundary);}

    /// Resize the boundary to the minimum bounding box surrounding the obstacles
    /// increased by a margin of _d + robotRadius.
    /// @param _d Margin to increase minimum bounding box
    /// @param _robot Robot to base the margin off of
    /// @TODO This is a temporary hack to be removed after cloning functions are
    ///       implemented in the boundary class hierarchy. Rather than changing
    ///       the environment boundary, methods wishing to use a modified
    ///       version of it should clone it and adjust the clone.
    void ResetBoundary(double _d, ActiveMultiBody* _robot);

    /// Forward to the boundary reset.
    /// @TODO This is a temporary hack to be removed after cloning functions are
    ///       implemented in the boundary class hierarchy. Rather than changing
    ///       the environment boundary, methods wishing to use a modified
    ///       version of it should clone it and adjust the clone.
    void ResetBoundary(const vector<pair<double, double>>& _bbx,
        const double _margin);

    /// Expand the boundary by a margin of _d + robotRadius
    /// @param _d Margin to increase bounding box
    /// @param _robot Robot to base the margin off of
    /// @TODO This is a temporary hack to be removed after cloning functions are
    ///       implemented in the boundary class hierarchy. Rather than changing
    ///       the environment boundary, methods wishing to use a modified
    ///       version of it should clone it and adjust the clone.
    void ExpandBoundary(double _d, ActiveMultiBody* _robot);

    ///@}
    ///@name Obstacle Functions
    ///@{

    /// Number of Static MultiBodies
    size_t NumObstacles() const {return m_obstacles.size();}

    /// @param _index Requested multibody
    /// @return Pointer to static multibody
    StaticMultiBody* GetObstacle(size_t _index) const;

    /// @return Pointer to random static multibody
    StaticMultiBody* GetRandomObstacle() const;

    /// Add an obstacle to the environment.
    /// @param _dir Directory for geometry file
    /// @param _filename Geometry filename
    /// @param _t Transformation of object
    /// @return (index, pointer) pair to newly created obstacle
    pair<size_t, shared_ptr<StaticMultiBody>> AddObstacle(const string& _dir,
        const string& _filename, const Transformation& _t = Transformation());

    /// Remove an obstacle from the environment.
    /// @param _position Index in m_obstacleBodies to be removed
    void RemoveObstacle(size_t _position);

    /// Remove obstacle from environment
    /// @param _obst Obstacle to be removed
    void RemoveObstacle(shared_ptr<StaticMultiBody> _obst);

    /// Compute a mapping of the obstacle vertices.
    /// @return A map from obstacle points to obstacle indexes.
    map<Vector3d, vector<size_t>> ComputeObstacleVertexMap() const;

    /// Build collision detection models for external libraries
    void BuildCDStructure();

    ///@}
    ///@name Decomposition
    ///@{

    const WorkspaceDecomposition* GetDecomposition() const {
      return m_decomposition.get();
    }

    /// Compute a decomposition of the workspace.
    /// @param _f The decomposition function to use.
    void Decompose(DecompositionFunction&& _f) {m_decomposition = _f(this);}

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Read boundary information.
    /// @param _is Input stream
    /// @param _cbs Counting stream buffer for accurate error reporting
    void ReadBoundary(istream& _is, CountingStreamBuffer& _cbs);

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

    Boundary* m_boundary{nullptr};                   ///< Workspace boundary.
    vector<shared_ptr<StaticMultiBody>> m_obstacles; ///< Obstacle multibodies.

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
InBounds(const CfgType& _cfg, const Boundary* const _b) {
  return _cfg.GetMultiBody()->InCSpace(_cfg.GetData(), _b)
      && _b->InBoundary(_cfg);
}

/*----------------------------------------------------------------------------*/

#endif
