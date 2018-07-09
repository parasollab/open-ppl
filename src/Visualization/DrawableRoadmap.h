#ifndef DRAWABLE_ROADMAP_H_
#define DRAWABLE_ROADMAP_H_

#include <vector>
#include <chrono>
#include <thread>
#include "glutils/drawable.h"

////////////////////////////////////////////////////////////////////////////////
/// Renderable representation of a Roadmap. Allows for visualization of vertex
/// and edge insertion.
////////////////////////////////////////////////////////////////////////////////
class DrawableRoadmap : public glutils::drawable  {
  public:

    /// @name Constructor
    /// @{

    /// Installs hooks onto _graph and populates existing
    /// vertices and edges.
    /// @tparam GraphType the type of the roadmap graph.
    ///                   Must be RoadmapGraph
    /// @param _graph The graph this DrawableRoadmap represents.
    /// @param _color The color to draw the cfgs and edges.
    template <typename GraphType>
    DrawableRoadmap(GraphType* _graph, const glutils::color& _color);

    ~DrawableRoadmap();

    /// @}
    /// @name Modifiers
    /// @{

    /// Adds vertices to be rendered
    /// @tparam VI Vertex Iterator
    /// @param _vi Vertex iterator to the vertex just added.
    template <typename VI>
    void AddVertex(VI _vi);

    /// Adds edges to be rendered
    /// @tparam EI Edge Iterator
    /// @param _ei edge iterator to the edge just added.
    template <typename EI>
    void AddEdge(EI _vi);

    /// @}

  protected:

    /// @name Drawable Overloads
    /// @{

    /// How to draw the base roadmap
    virtual void draw() override;


    virtual void draw_select() override;

    /// How to draw the roadmap when selected
    virtual void draw_selected() override;

    /// How to draw the roadmap when highlighted
    virtual void draw_highlighted() override;

    /// @}

  private:
    std::vector<glutils::vector3f> m_cfgs;                ///< The position of cfgs.
    std::vector<std::vector<glutils::vector3f>> m_edges;  ///< the path of each edge.
    glutils::color m_color;                               ///< The color to be rendered in.
};

template <typename GraphType>
DrawableRoadmap::
DrawableRoadmap(GraphType* _graph, const glutils::color& _color) :
  m_color(_color) {
  // setting hook for adding vertices to DRM
  _graph->InstallHook(GraphType::HookType::AddVertex, "DRM-AddVertex",
      [this](typename GraphType::VI _vi) {
        this->AddVertex(_vi);
      });

  // setting hook for adding edges to DRM
  _graph->InstallHook(GraphType::HookType::AddEdge, "DRM-AddEdge",
      [this](typename GraphType::EI _vi) {
        this->AddEdge(_vi);
      });

  // Adding the current content of the graph to the DRM.
  for(auto iter = _graph->begin(); iter != _graph->end(); ++iter)
    AddVertex(iter);

  for(auto iter = _graph->edges_begin(); iter != _graph->edges_end(); ++iter)
    AddEdge(iter);
}

template <typename VI>
void
DrawableRoadmap::
AddVertex(VI _vi) {
  // Gets Vertex data.
  auto cfg = _vi->property();
  auto p = cfg.GetPoint();

  // build 3d vector
  glutils::vector3f point = {(float)p[0], (float) p[1], (float) p[2]};

  // add it to vector of cfg points.
  m_cfgs.push_back(point);
}

template <typename EI>
void
DrawableRoadmap::
AddEdge(EI _ei) {

  // Gets Edge data.
  auto edge = _ei->property();
  const auto& intermediates = edge.GetIntermediates();

  // vids of the source and target vids
  auto start_vid = _ei->source();
  auto end_vid = _ei->target();

  m_edges.push_back(std::vector<glutils::vector3f>());

  // add the source point to the edge points
  // Since the cfgs are added inorder of creation, the
  // VIDs can be used as an index into the array.
  m_edges.back().push_back(m_cfgs[start_vid]);

  // for each intermiate add it to the last edge, the edge
  // just created.
  for(const auto& inter : intermediates) {
    auto p = inter.GetPoint();
    glutils::vector3f point = {(float) p[0], (float) p[1], (float) p[2]};
    m_edges.back().push_back(point);
  }

  // add the target cfg to the edge points
  m_edges.back().push_back(m_cfgs[end_vid]);
}

#endif
