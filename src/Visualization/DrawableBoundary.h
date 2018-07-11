#ifndef DRAWABLE_BOUNDARY_H_
#define DRAWABLE_BOUNDARY_H_

class Boundary;

#include <glutils/drawable_call_list.h>

////////////////////////////////////////////////////////////////////////////////
/// Drawable representation of a pmpl boundary
////////////////////////////////////////////////////////////////////////////////
class DrawableBoundary : public glutils::drawable_call_list {
  public:
    ///@name Constructor
    ///@{

    /// Builds a new drawable boundary for drawing.
    /// @param _boundary Boundary to be drawn
    /// @param _color the color to render the boundary.
    /// @param _solid whether to draw the boundary in wire frame
    ///               or rasterized triangles.
    DrawableBoundary(const Boundary* _boundary, const glutils::color& _color,
        bool _solid = false);

    ///@}
    ///@name Drawable Call List overrides
    ///@{

    virtual void build() override;
    virtual void build_select() override;

    ///@}
  private:
    ///@name Internal State
    ///@{

    const Boundary* m_boundary; ///< Boundary being drawn.
    glutils::color m_color;     ///< Color to render in
    bool m_solid;               ///< solid flag

    ///@}
};


#endif
