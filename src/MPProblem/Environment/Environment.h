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

class CollisionDetectionMethod;
class MultiBody;
class Robot;
class WorkspaceDecomposition;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Workspace for the motion planning problem, including a boundary and zero or
/// more obstacles.
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

    /// Parse XML options from the Problem XML node.
    void ReadXMLOptions(XMLNode& _node);

    /// Parse XML environment file.
    void ReadXML(XMLNode& _node);

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

    /// Resize the boundary to the minimum bounding box surrounding the obstacles
    /// increased by a margin of _d + multibody radius.
    /// @param _d Margin to increase minimum bounding box
    /// @param _multibody MultiBody to base the margin off of
    /// @TODO This is a temporary hack to be removed after cloning functions are
    ///       implemented in the boundary class hierarchy. Rather than changing
    ///       the environment boundary, methods wishing to use a modified
    ///       version of it should clone it and adjust the clone.
    void ResetBoundary(double _d, const MultiBody* const _multibody);

    /// Forward to the boundary reset.
    /// @TODO This is a temporary hack to be removed after cloning functions are
    ///       implemented in the boundary class hierarchy. Rather than changing
    ///       the environment boundary, methods wishing to use a modified
    ///       version of it should clone it and adjust the clone.
    void ResetBoundary(const vector<pair<double, double>>& _bbx,
        const double _margin);

    /// Expand the boundary by a margin of _d + multibody radius
    /// @param _d Margin to increase bounding box
    /// @param _multibody MultiBody to base the margin off of
    /// @TODO This is a temporary hack to be removed after cloning functions are
    ///       implemented in the boundary class hierarchy. Rather than changing
    ///       the environment boundary, methods wishing to use a modified
    ///       version of it should clone it and adjust the clone.
    void ExpandBoundary(double _d, const MultiBody* const _multibody);

    ///@}
    ///@name Obstacle Functions
    ///@{

    /// Number of MultiBodies
    size_t NumObstacles() const {return m_obstacles.size();}

    /// @param _index Requested multibody
    /// @return Pointer to static multibody
    MultiBody* GetObstacle(size_t _index) const;

    /// @return Pointer to random static multibody
    MultiBody* GetRandomObstacle() const;

    /// Add an obstacle to the environment.
    /// @param _dir Directory for geometry file
    /// @param _filename Geometry filename
    /// @param _t Transformation of object
    /// @return (index, pointer) pair to newly created obstacle
    pair<size_t, shared_ptr<MultiBody>> AddObstacle(const string& _dir,
        const string& _filename, const Transformation& _t = Transformation());

    /// Remove an obstacle from the environment.
    /// @param _position Index in m_obstacleBodies to be removed
    void RemoveObstacle(size_t _position);

    /// Remove obstacle from environment
    /// @param _obst Obstacle to be removed
    void RemoveObstacle(shared_ptr<MultiBody> _obst);

    /// Compute a mapping of the obstacle vertices.
    /// @return A map from obstacle points to obstacle indexes.
    map<Vector3d, vector<size_t>> ComputeObstacleVertexMap() const;

    ///@}
    ///@name Decomposition
    ///@{

    /// Get the decomposition model if this workspace. If it has not been
    /// created yet, a null pointer will be returned.
    WorkspaceDecomposition* GetDecomposition();

    /// Get the decomposition model if this workspace. If it has not been
    /// created yet, a null pointer will be returned.
    const WorkspaceDecomposition* GetDecomposition() const;

    /// Compute a decomposition of the workspace.
    /// @param _f The decomposition function to use.
    void Decompose(DecompositionFunction&& _f);

    ///@}
    ///@name Physical Properties
    ///@{

    /// Get the friction coefficient
    double GetFrictionCoefficient() const noexcept;

    /// Get the gravity 3-vector
    const Vector3d& GetGravity() const noexcept;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Read boundary information.
    /// @param _is Input stream
    /// @param _cbs Counting stream buffer for accurate error reporting
    /// @TODO Move this into the boundary classes where it belongs.
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

    /// The position resolution is set below 0 to trigger a formal computation
    /// of it, if it's not provided in either the XML or env file.
    double m_positionRes{-1.};       ///< Positional resolution of movement.
    double m_positionResFactor{.01}; ///< Factor of body span to use as auto-
                                     ///< computed positional resolution.
    double m_orientationRes{.05};    ///< Rotational resolution of movement.
    double m_timeRes{.05};           ///< Resolution for time.

    ///@}
    ///@name Models
    ///@{

    Boundary* m_boundary{nullptr};             ///< Workspace boundary.
    vector<shared_ptr<MultiBody>> m_obstacles; ///< Obstacle multibodies.

    ///@}
    ///@name Decomposition
    ///@{

    /// A workspace decomposition model describes a partitioning of free space.
    shared_ptr<WorkspaceDecomposition> m_decomposition;

    ///@}
    ///@name Physical Properties
    ///@{

    double m_frictionCoefficient{0}; ///< The uniform friction coefficient.
    Vector3d m_gravity;              ///< The gravity direction and magnitude.

    ///@}

};

#endif
