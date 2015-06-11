#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include "Cfg/CfgMultiRobot.h"
#include "MPProblem/Boundary.h"
#include "Utilities/MPUtils.h"

class ActiveMultiBody;
class CollisionDetectionMethod;
class MultiBody;
class StaticMultiBody;
class SurfaceMultiBody;

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

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{
    Environment();
    ////////////////////////////////////////////////////////////////////////////
    /// @param _node XMLNode describing environment parameters
    Environment(XMLNode& _node);

    virtual ~Environment();
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name I/O
    /// @{
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
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Resolutions
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @return Position resolution
    double GetPositionRes() const { return m_positionRes; }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _res Position resoltuion
    void SetPositionRes(double _res) { m_positionRes = _res; }

    ////////////////////////////////////////////////////////////////////////////
    /// @return Orientation resolution
    double GetOrientationRes() const { return m_orientationRes; }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _res Orientation resolution
    void SetOrientationRes(double _res) { m_orientationRes = _res; }

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    ////////////////////////////////////////////////////////////////////////////
    /// @return Reachable distance resolution
    double GetRDRes() const {return m_rdRes;}
#endif
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Boundary
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @return Boundary pointer
    shared_ptr<Boundary> GetBoundary() const {return m_boundary;}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _b Boundary pointer
    void SetBoundary(shared_ptr<Boundary> _b) {m_boundary = _b;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Test if configuration in inside of the workspace and satisfies
    ///        physical robot constraints.
    /// @param _cfg Configuration
    /// @param _b Workspace region boundary
    /// @return True if inside workspace and satisfying contraints
    ///
    /// Test whether input configuration satisfies joint constraints  (i.e., is
    /// inside of C-Space) and lies inside of the workspace boundary (i.e., the
    /// robot at that configuration is inside of the workspace).
    bool InBounds(const Cfg& _cfg, shared_ptr<Boundary> _b);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Test if configuration in inside of the workspace and satisfies
    ///        physical robot constraints.
    ///
    /// @overload
    /// No boundary is specified.
    bool InBounds(const Cfg& _cfg) {return InBounds(_cfg, m_boundary);}
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Test if configuration in inside of the workspace and satisfies
    ///        physical robot constraints.
    ///
    /// @overload
    /// CfgMultiRobot overload.
    /// @todo this is a work around for CfgMultiRobot class InBounds check
    bool InBounds(const CfgMultiRobot& _cfg, shared_ptr<Boundary> _b);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Test if configuration in inside of the workspace and satisfies
    ///        physical robot constraints.
    ///
    /// @overload
    /// CfgMultiRobot overload.
    /// @todo this is a work around for CfgMultiRobot class InBounds check
    bool InBounds(const CfgMultiRobot& _cfg) {return InBounds(_cfg, m_boundary);}

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
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name MultiBodies
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of Active MultiBodies
    size_t GetActiveBodyCount() const {return m_activeBodies.size();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of Static MultiBodies
    size_t GetObstacleCount() const {return m_obstacleBodies.size();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of total MultiBodies
    size_t GetUsableMultiBodyCount() const {return m_usableMultiBodies.size();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of Surface MultiBodies
    size_t GetNavigableSurfacesCount() const {return m_navigableSurfaces.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Requested multibody
    /// @return Pointer to active multibody
    shared_ptr<ActiveMultiBody> GetActiveBody(size_t _index) const;
    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Requested multibody
    /// @return Pointer to static multibody
    shared_ptr<StaticMultiBody> GetStaticBody(size_t _index) const;
    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Requested multibody
    /// @return Pointer to multibody
    shared_ptr<MultiBody> GetMultiBody(size_t _index) const;
    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Requested multibody
    /// @return Pointer to surface multibody
    shared_ptr<SurfaceMultiBody> GetNavigableSurface(size_t _index) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @return Pointer to random static multibody
    shared_ptr<MultiBody> GetRandomObstacle() const;
    ////////////////////////////////////////////////////////////////////////////
    /// @return Index to random navigable surface. -1 means base surface.
    ssize_t GetRandomNavigableSurfaceIndex();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add Obstacle to environment
    /// @param _modelFileName path to .obj file that specifies obstacle geometry
    /// @param _where a 6 dof vector specifying position and orientation
    /// @param _cdTypes OLD
    /// @return Obstacle's index in m_obstacleBodies on success, -1 otherwise
    //int AddObstacle(string _modelFileName, const Transformation& _where,
    //    const vector<cd_predefined>& _cdTypes);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Remove obstacle from environment
    /// @param position Index in m_obstacleBodies to be removed
    //void RemoveObstacleAt(size_t position);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build collision detection models for external libraries
    /// @param _cdMethod Requested external library to build for
    void BuildCDStructure(CollisionDetectionMethod* _cdMethod);
    /// @}
    ////////////////////////////////////////////////////////////////////////////

  protected:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read boundary information
    /// @param _is Input stream
    /// @param _cbs Counting stream buffer for accurate error reporting
    void ReadBoundary(istream& _is, CountingStreamBuffer& _cbs);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Automatically compute resolutions
    ///
    /// ComputeResolution, if m_posRes is < 0 then auto compute the resolutions
    /// based on minimum of max body spans multiplied by @c m_positionResFactor.
    /// Reachable distance resolution is computed based upon input res
    /// multiplied by the number of joints.
    void ComputeResolution();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determine if @p _cfg is within physical robot contraints
    /// @param _cfg Configuration
    /// @param _b Workspace region of environment
    /// @return True if @p _cfg is inside physical robot constraints
    bool InCSpace(const Cfg& _cfg, shared_ptr<Boundary> _b);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determine if @p _cfg is within workspace boundary
    /// @param _cfg Configuration
    /// @param _b Workspace region of environment
    /// @return True if @p _cfg is inside workspace boundary
    bool InWSpace(const Cfg& _cfg, shared_ptr<Boundary> _b);

    string m_filename;     ///< Which file did this environment come from
    string m_modelDataDir; ///< Directory where environment file is located
    bool m_saveDofs;       ///< Should we save the dof information to a file

    double m_positionRes;       ///< Positional resolution of movement
    double m_positionResFactor; ///< Factor of body span to use as auto computed
                                ///< Positional resolution
    double m_orientationRes;    ///< Rotational resolution of movement
    double m_rdRes;             ///< Resolution for movement in RD space

    shared_ptr<Boundary> m_boundary; ///< Boundary of the workspace

    vector<shared_ptr<ActiveMultiBody>> m_activeBodies; ///< Robots
    vector<shared_ptr<SurfaceMultiBody>>
      m_navigableSurfaces;                              ///< Surfaces
    vector<shared_ptr<StaticMultiBody>>
      m_obstacleBodies;                                 ///< Other multibodies
    vector<shared_ptr<MultiBody>> m_usableMultiBodies;  ///< All multibodies
};


#endif
