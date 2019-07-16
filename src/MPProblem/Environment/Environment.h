#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include "Geometry/Boundaries/Boundary.h"
#include "Utilities/MPUtils.h"

#include "Transformation.h"
#include "glutils/color.h"

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

class CollisionDetectionMethod;
class MultiBody;

class Robot;
class WorkspaceDecomposition;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Workspace representation of terrain within the world.
////////////////////////////////////////////////////////////////////////////////
class Terrain {
  public:

    ///@name Local Types
    ///@{

    enum Axis {X, Y, Z};

		///@}

    ///@name Construction
    ///@{

    /// Default constructor
    Terrain();

    /// Constructor for xml parsing
    Terrain(XMLNode& _node);

    /// Copy constructor
    Terrain(const Terrain& _terrain);

    bool IsNeighbor(const Terrain& _terrain);

    double GetPerimeter();
    ///@}
    ///@name Accessors
    ///@{

    const glutils::color& Color() const noexcept;

    const Boundary* GetBoundary() const noexcept;

		const std::vector<std::unique_ptr<Boundary>>& GetBoundaries() const noexcept;

		bool InTerrain(const Point3d _p) const noexcept;

		bool InTerrain(const Cfg _cfg) const noexcept;

		bool IsVirtual() const noexcept;

    bool IsWired() const noexcept;
    ///@}
  private:
		///@name Helpers
		///@{

    bool IsTouching(Boundary* _bound1, Boundary* _bound2, Axis& _type);

    double Overlap(Boundary* _b1, Boundary* _b2);

		///@}
		///@name Internal State
		///@{

    glutils::color m_color{glutils::color::green}; ///< the color of the boundary when rendering
    std::unique_ptr<Boundary> m_boundary; ///< represents where the terrain is located.

		std::vector<std::unique_ptr<Boundary>> m_boundaries; ///< represent the space occupied by the terrain

		bool m_virtual{false}; ///< indicates a boundary limit for robots of the same capability
													 ///< but does not invalidate robots of other capabiilties inside it.

    /// A rendering property, if true then the boundary is
    /// rendered as a solid mesh; otherwise, it is rendered in wire frame.
    bool m_wire{true};

		///@}
};


////////////////////////////////////////////////////////////////////////////////
/// Workspace for the motion planning problem, including a boundary and zero or
/// more obstacles.
////////////////////////////////////////////////////////////////////////////////
class Environment {

  public:

    ///@name Local Types
    ///@{

    typedef std::unordered_map<std::string, std::vector<Terrain>>
        TerrainMap;

    ///@}
    ///@name Construction
    ///@{

    Environment();

    explicit Environment(XMLNode& _node);

    Environment(const Environment& _other); ///< Copy.
    Environment(Environment&& _other);      ///< Move.

    virtual ~Environment();

    ///@}
    ///@name Assignment
    ///@{

    Environment& operator=(const Environment& _other); ///< Copy.
    Environment& operator=(Environment&& _other);      ///< Move.

    ///@}
    ///@name I/O
    ///@{

    /// Get the environment file name.
    const std::string& GetEnvFileName() const noexcept;

    /// Parse XML options from the Problem XML node.
    void ReadXMLOptions(XMLNode& _node);

    /// Parse XML environment file.
    void ReadXML(XMLNode& _node);

    /// Parse an old-style environment file.
    /// @param _filename The name of the file to read.
    /// @note This is provided only for supporting old environments. Do not make
    ///       any modifications here - if you need to add features, you must
    ///       upgrade your env file to the newer XML format.
    void Read(std::string _filename);

    /// Print environment resolutions and boundary information.
    /// @param _os The output stream to print to.
    void Print(std::ostream& _os) const;

    /// Output environment to .env file format.
    /// @param _os The output stream to write to.
    void Write(std::ostream& _os);

    ///@}
    ///@name Resolutions
    ///@{

    /// Automatically compute resolutions.
    ///
    /// ComputeResolution, if m_posRes is < 0 then auto compute the resolutions
    /// based on minimum of max body spans multiplied by @c m_positionResFactor.
    /// Reachable distance resolution is computed based upon input res
    /// multiplied by the number of joints.
    void ComputeResolution(const std::vector<std::unique_ptr<Robot>>& _robots);

    double GetPositionRes() const noexcept;
    void SetPositionRes(double _res) noexcept;

    double GetOrientationRes() const noexcept;
    void SetOrientationRes(double _res) noexcept;

    double GetTimeRes() const noexcept;

    ///@}
    ///@name Boundary Functions
    ///@{

    const Boundary* GetBoundary() const noexcept;

    void SetBoundary(std::unique_ptr<Boundary>&& _b) noexcept;

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
    void ResetBoundary(const std::vector<std::pair<double, double>>& _bbx,
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
    size_t NumObstacles() const noexcept;

    /// @param _index Requested multibody
    /// @return Pointer to static multibody
    MultiBody* GetObstacle(size_t _index) const;

    /// @return Pointer to random static multibody
    MultiBody* GetRandomObstacle() const;

    /// Add an obstacle to the environment.
    /// @param _dir Directory for geometry file
    /// @param _filename Geometry filename
    /// @param _t Transformation of object
    /// @return The index of the newly created obstacle.
    size_t AddObstacle(const std::string& _dir, const std::string& _filename,
        const mathtool::Transformation& _t = mathtool::Transformation());

    /// Remove an obstacle from the environment.
    /// @param _position Index in m_obstacleBodies to be removed
    void RemoveObstacle(const size_t _position);

    /// Remove obstacle from environment
    /// @param _obst Obstacle to be removed
    void RemoveObstacle(MultiBody* const _obst);

    /// Compute a mapping of the obstacle vertices.
    /// @return A map from obstacle points to obstacle indexes.
    std::map<mathtool::Vector3d, std::vector<size_t>> ComputeObstacleVertexMap()
        const;

    /// Check if the boundary is also modeled as an obstacle.
    bool UsingBoundaryObstacle() const noexcept;

    ///@}
    ///@name Physical Properties
    ///@{

    /// Get the friction coefficient
    double GetFrictionCoefficient() const noexcept;

    /// Get the gravity 3-vector
    const mathtool::Vector3d& GetGravity() const noexcept;

    ///@}
    ///@name Terrain Functions
    ///@{

    const TerrainMap& GetTerrains() const noexcept;

    ///@}
    ///@name Simulation Functions
    ///@{

    /// Get the initial transformation for the camera.
    const mathtool::Transformation& GetInitialCameraTransformation() const
        noexcept;

    ///IROS Hacks

    bool IsolateTerrain(Cfg start, Cfg goal);

		bool SameTerrain(Cfg _start, Cfg _goal);

    void RestoreBoundary();

    void SaveBoundary();

    std::unique_ptr<Boundary> m_originalBoundary;
    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Initialize a boundary object of the appropriate type.
    void InitializeBoundary(std::string _type, const std::string _where);

    /// Create an obstacle for the boundary.
    void CreateBoundaryObstacle();

    ///@}
    ///@name File Info
    ///@{

    std::string m_filename;     ///< Which file did this environment come from?
    std::string m_modelDataDir; ///< Directory where environment file is located.

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

    std::unique_ptr<Boundary> m_boundary;               ///< Workspace boundary.
    std::vector<std::unique_ptr<MultiBody>> m_obstacles;///< Obstacle multibodies.
    bool m_boundaryObstacle{false}; ///< Use the boundary as an obstacle?

    ///@}
    ///@name Physical Properties
    ///@{

    double m_frictionCoefficient{0}; ///< The uniform friction coefficient.
    mathtool::Vector3d m_gravity;    ///< The gravity direction and magnitude.

    ///@}
    ///@name Terrains
    ///@{

    TerrainMap m_terrains; ///< Environment terrains.

    ///@}
    ///@name Simulation Properties
    ///@{

    mathtool::Transformation m_initialCameraTransform; ///< Camera starts here.

    ///@}

};

#endif
