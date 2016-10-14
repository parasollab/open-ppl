#ifndef GRIDOVERLAY_H_
#define GRIDOVERLAY_H_
#include "Cfg/Cfg.h"
#include "Environment/Boundary.h"
#include "Vector.h"

///////////////////////////////////////////////////////////////////////////////
/// A grid overlay of a given boundary.
///
///
/// Currently it only works for bounding boxes
///////////////////////////////////////////////////////////////////////////////



class GridOverlay {
  public:

    ///@name Types 
    ///@{
      typedef size_t VID; 
    ///@}


    ///@name Constructor
    ///@{
    GridOverlay(shared_ptr<Boundary> _b, int _length);
    ///@}
    
    
    size_t LocateCell(const Cfg& _v) const;
    size_t IndexOfCoor(const Vector3d& _coor) const;

  private:
    int m_length; ///< Length of a cell
    shared_ptr<Boundary> m_boundary; ///< the boundary of the grid
};

#endif
