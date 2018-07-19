#ifndef DRAWABLE_BOUNDARY_H_
#define DRAWABLE_BOUNDARY_H_


#include "DrawablePolyhedron.h"

class Boundary;


////////////////////////////////////////////////////////////////////////////////
/// Drawable representation of a pmpl boundary
////////////////////////////////////////////////////////////////////////////////
class DrawableBoundary : public DrawablePolyhedron {
  public:
    ///@name Constructor
    ///@{

    /// Builds a new drawable boundary for drawing.
    ///@param _boundary Boundary to be drawn
    ///@param _color the color to render the boundary.
    ///@param _wire whether to draw the boundary in wire frame
    ///              or rasterized triangles.

    DrawableBoundary(const Boundary* _boundary, const glutils::color& _color,
        bool _wire = false);

    ///@}

  protected:
    ///@name DrawalbePolyhedron Overrides
    ///@{

    void build() override;

    ///@}
    ///@name Local State
    ///@{
  
    GMSPolyhedron m_polyhedron;

    ///@}
};


#endif
