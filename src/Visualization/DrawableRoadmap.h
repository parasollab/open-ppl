#ifndef DRAWABLE_ROADMAP_H_
#define DRAWABLE_ROADMAP_H_

#include <vector>
#include <chrono>
#include <thread>
#include "glutils/drawable.h"
#include "DrawableCfg.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "ConfigurationSpace/Weight.h"

#include <mutex>

class DrawableMultiBody;

////////////////////////////////////////////////////////////////////////////////
/// Renderable representation of a Roadmap. Allows for visualization of vertex
/// and edge insertion.
////////////////////////////////////////////////////////////////////////////////
class DrawableRoadmap : public glutils::drawable  {
  public:

    ///@name Local Types
    ///@{

    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> GraphType;
    typedef typename GraphType::VI VI;
    typedef typename GraphType::EI EI;

    ///@}

    ///@name Constructor
    ///@{

    /// Installs hooks onto _graph and populates existing
    /// vertices and edges.
    ///@param _graph The graph this DrawableRoadmap represents.
    ///@param _color The color to draw the cfgs and edges.
    ///@param _name A name refering to this DrawableRoadmap
    DrawableRoadmap(GraphType* _graph, const glutils::color& _color,
        const std::string& _name = "");

    ~DrawableRoadmap();

    ///@}
    ///@name Modifiers
    ///@{

    /// Adds vertices to be rendered
    ///@param _vi Vertex iterator to the vertex just added.
    void AddVertex(VI _vi);

    /// Adds edges to be rendered
    ///@param _ei edge iterator to the edge just added.
    void AddEdge(EI _ei);


    /// Removes vertices from graph
    ///@param _vi Vertex iterator to the vertex just added.
    void RemoveVertex(VI _vi);

    /// Removes an edges from graph
    ///@param _ei edge iterator to the edge just added.
    void RemoveEdge(EI _ei);

    ///@}
    ///@name Rendering Property
    ///@{

    void ToggleRobot();

    ///@}

  protected:

    ///@name Drawable Overloads
    ///@{

    virtual void initialize() override;

    /// How to draw the base roadmap
    virtual void draw() override;


    virtual void draw_select() override;

    /// How to draw the roadmap when selected
    virtual void draw_selected() override;

    /// How to draw the roadmap when highlighted
    virtual void draw_highlighted() override;

    ///@}

  private:
    std::vector<DrawableCfg> m_cfgs;                ///< The position of cfgs.
    std::vector<std::vector<glutils::vector3f>> m_edges;  ///< the path of each edge.
    std::vector<DrawableCfg> m_bufferCfgs;                      ///< buffer the cfgs when being added
    std::vector<std::vector<glutils::vector3f>> m_bufferEdges;  ///< buffer the edges when they are being added
    glutils::color m_color;                               ///< The color to be rendered in.
    MultiBody m_multiBody;                 ///< local copy of the graphs multibody.
    std::unique_ptr<DrawableMultiBody> m_dmb;              ///< DrawableMultiBody used by the cfgs.
    std::atomic<bool> m_drawRobot{false};               ///< rendering mode. (defaults to point mode)
    mutable std::mutex m_CfgGuard;         ///< Lock for updating cfgs.
    mutable std::mutex m_EdgeGuard;        ///< Lock for updating edges.
    GraphType* m_graph;                    ///< a pointer to the RoadmapGraph.

    std::string m_name;                    ///< A name for this graph.
};

#endif
