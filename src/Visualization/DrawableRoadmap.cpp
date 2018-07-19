#include "DrawableRoadmap.h"
#include "DrawableMultiBody.h"
#include "Geometry/Bodies/MultiBody.h"
#include "sandbox/gui/main_window.h"
#include "Simulator/Conversions.h"
#include "Simulator/Simulation.h"

#include <iostream>

// counter for the number of instances of DrawableRoadmapGraph
static size_t roadmapGraphCounter = 0;

DrawableRoadmap::
DrawableRoadmap(GraphType* _graph, const glutils::color& _color, const std::string& _name) :
  m_color(_color),
  m_multiBody(*_graph->GetRobot()->GetMultiBody()),
  m_graph(_graph) {

  // if a name is given the use that name, otherwise generate one
  if(_name.empty())
    m_name = "DRM" + std::to_string(roadmapGraphCounter++);
  else
    m_name = _name;

  std::cout << "Name: " << m_name << std::endl;
  // setting hook for adding vertices to DRM
  m_graph->InstallHook(GraphType::HookType::AddVertex, m_name,
      [this](VI _vi) {
        this->AddVertex(_vi);
      });

  // setting hook for adding edges to DRM
  m_graph->InstallHook(GraphType::HookType::AddEdge, m_name,
      [this](EI _ei) {
        this->AddEdge(_ei);
      });

  // Adding the current content of the graph to the DRM.
  for(auto iter = m_graph->begin(); iter != m_graph->end(); ++iter)
    AddVertex(iter);

  for(auto iter = m_graph->begin(); iter != m_graph->end(); ++iter)
    for(auto edges = iter->begin(); edges != iter->end(); ++edges)
        AddEdge(edges);

  // @TODO: Check if the graph is selected and being rendered before toggling
  //        the render mode.
  auto fn = [this](QKeyEvent* const _e) {
      if(_e->key() == Qt::Key_P) {
        this->ToggleRobot();
      }
    };

  theOneWindow->add_key_mapping(m_name, std::move(fn));
}

// The map hooks have already been cleared by the time the destructor is called
// by the Simulator.
DrawableRoadmap::
~DrawableRoadmap() {
  if(m_graph->IsHook(GraphType::HookType::AddVertex, m_name))
    m_graph->RemoveHook(GraphType::HookType::AddVertex, m_name);
  if(m_graph->IsHook(GraphType::HookType::AddEdge, m_name))
    m_graph->RemoveHook(GraphType::HookType::AddEdge, m_name);
  if(theOneWindow)
    theOneWindow->delete_key_mapping(m_name);
}

void
DrawableRoadmap::
AddVertex(VI _vi) {
  // Gets Vertex data.
  auto cfg = _vi->property();
  std::lock_guard<std::mutex> lock(m_CfgGuard);
  m_bufferCfgs.push_back(DrawableCfg(cfg.GetData(), m_dmb.get()));
}

void
DrawableRoadmap::
AddEdge(EI _ei) {

  // Gets Edge data.
  auto edge = _ei->property();
  const auto& intermediates = edge.GetIntermediates();

  // vids of the source and target vids
  auto startVid = _ei->source();
  auto endVid = _ei->target();

  std::vector<glutils::vector3f> edgeList;

  // get the cfgs from the graph
  auto startCfg = m_graph->GetVertex(startVid);
  auto endCfg   = m_graph->GetVertex(endVid);

  edgeList.push_back(ToGLUtils(startCfg.GetPoint()));

  // for each intermiate add it to the last edge, the edge
  // just created.
  for(const auto& inter : intermediates) {
    auto p = inter.GetPoint();
    edgeList.push_back(ToGLUtils(p));
  }

  edgeList.push_back(ToGLUtils(endCfg.GetPoint()));

  // add the target cfg to the edge points
  std::lock_guard<std::mutex> lock(m_EdgeGuard);
  m_bufferEdges.emplace_back(std::move(edgeList));
}

void
DrawableRoadmap::
RemoveVertex(VI _vi) {
  throw NotImplementedException(WHERE);
}

void DrawableRoadmap::
RemoveEdge(EI _ei) {
  throw NotImplementedException(WHERE);
}

void
DrawableRoadmap::
initialize() {
  glutils::drawable::initialize();

  for(size_t i = 0; i < m_multiBody.GetNumBodies(); ++i)
    m_multiBody.GetBody(i)->SetColor(m_color);

  m_dmb = std::unique_ptr<DrawableMultiBody>(new DrawableMultiBody(&m_multiBody));
  m_dmb->initialize();
}


void
DrawableRoadmap::
draw() {
  // sets the color
  glColor4fv(m_color);

  /// copy all of the recently added elements into the render buffer
  {
    std::lock_guard<std::mutex> lock(m_CfgGuard);
    copy(m_bufferCfgs.begin(), m_bufferCfgs.end(), std::back_inserter(m_cfgs));
    m_bufferCfgs.clear();
  }
  {
    std::lock_guard<std::mutex> lock(m_EdgeGuard);
    copy(m_bufferEdges.begin(), m_bufferEdges.end(), std::back_inserter(m_edges));
    m_bufferEdges.clear();
  }



  // if draw robot then draw a rendering of the robot at the cfg,
  // otherwise, draw the cfg as a point.
  if(m_drawRobot)
    for(auto& cfg : m_cfgs)
      cfg.render();
  else {
    glPointSize(8.0);
    glDisable(GL_LIGHTING);
    // sets the size of each point (radius in pixels?)

    // says to draw a point primative
    glBegin(GL_POINTS);
    for(auto& v : m_cfgs)
      glVertex3fv(v.GetPoint().begin());
    glEnd();
    glEnable(GL_LIGHTING);
  }

  glDisable(GL_LIGHTING);

  // sets the width of the line
  glLineWidth(1.0);
  for(const auto& edge : m_edges) {
    // after the first vertex, for each new vertice
    // added a new line will be drawn.
    // Lets say n vertices were drawn, n-1 vertices will
    // rendered. Start from 0 to 1, 1 to 2, 2 to 3 and so on.
    glBegin(GL_LINE_STRIP);
    for(const auto& inter : edge)
      glVertex3fv(inter.begin());
    glEnd();
  }
  glEnable(GL_LIGHTING);
}
void DrawableRoadmap:: draw_select() {}
void
DrawableRoadmap::
draw_selected() {}

void
DrawableRoadmap::
draw_highlighted() {}

void
DrawableRoadmap::
ToggleRobot() {
  m_drawRobot = !m_drawRobot;
}
