#ifndef GRID_OVERLAY_H_
#define GRID_OVERLAY_H_

#include "Cfg/Cfg.h"
#include "Environment/Boundary.h"
#include "Vector.h"

///////////////////////////////////////////////////////////////////////////////
/// A grid overlay of a given boundary.
///////////////////////////////////////////////////////////////////////////////
class GridOverlay {

  public:

    ///@name Local Types
    ///@{

    typedef size_t VID;

    ///@}
    ///@name Construction
    ///@{

    GridOverlay(shared_ptr<Boundary> _b, double _length);

    ///@}
    ///@name Cell Finders
    ///@{

    size_t LocateCell(const Cfg& _v) const;
    size_t LocateCell(const Point3d& _p) const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_length;                 ///< Length of a cell.
    shared_ptr<Boundary> m_boundary; ///< The boundary of the grid.

    ///@}
};

#endif
