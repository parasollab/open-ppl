#ifndef DRAWABLE_POLYHEDRON_H_
#define DRAWABLE_POLYHEDRON_H_

#include "glutils/drawable_display_list.h"
#include "Geometry/GMSPolyhedron.h"

class DrawablePolyhedron : public glutils::drawable_display_list {
  public:


    ///@name Constructor
    ///@{

    DrawablePolyhedron(const GMSPolyhedron& _polyhedron, const glutils::color& _color, bool _wire);

    ///@}
    ///@name Modifier
    ///@{

    void ToggleRenderMode();

    ///@}
  protected:

    ///@name drawable_display_list overrides
    ///@{

    virtual void build() override;
    virtual void build_select() override;
    virtual void build_selected() override;

    ///@}
  protected:

    ///@name Helpers
    ///@{

    /// Builds a map showing the lines of adjacent polygons and whether they
    /// need to be drawn.
    /// @return The adjacency map of the current polyhedron.
    std::set<std::pair<size_t, size_t>> BuildAdjacencyMap();

    ///@}
    ///@name Local State
    ///@{

    const GMSPolyhedron& m_polyhedron; ///< Copy of the polyhedron
    glutils::color m_color;     ///< Color fo the polyhedron
    bool m_wire;                ///< Render mode (true: wireframe, false: solid mesh)

    ///@}
};

#endif
