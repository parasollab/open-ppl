#ifndef _MOVIE_BYU_SAVER_H_
#define _MOVIE_BYU_SAVER_H_

#include <iostream>
#include <fstream>
#include <string>

#include "ModelGraph.h" //for counting edge size
using namespace modelgraph;
#include "IPolygonal.h"

////////////////////////////////////////////////////////////////////////////////
/// \ingroup DeadCode
////////////////////////////////////////////////////////////////////////////////
class MoiveBYUSaver {

  typedef IPolygonal::TriVector TriVector;
  typedef IPolygonal::PtVector PtVector;

  public:

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Output a BYU file of a model.
    /// \param[in] _filename The filename to use.
    /// \param[in] _pts The model's vertices.
    /// \param[in] _tris The model's triangles.
    bool SaveBYU(const std::string& _filename, const PtVector& _pts,
        const TriVector& _tris) {
      std::ofstream fout(_filename);
      if(!fout.good()) {
        cerr << "Can't Open file : " << _filename << endl;
        return false;
      }
      SaveBYU(fout, _pts, _tris);
      return true;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Output a BYU format of a model.
    /// \param[in] _pts The model's vertices.
    /// \param[in] _tris The model's triangles.
    void SaveBYU(std::ostream& _os, const PtVector& _pts,
        const TriVector& _tris) {
      _os << 1 << " " << _pts.size() << " " << _tris.size()
          << " " << _tris.size() * 3 << std::endl
          << 1 << " " << _tris.size() << std::endl;
      for(const auto& p : _pts)
        _os << p[0] << " " << p[1] << " " << p[2] << "\n";
      for(const auto& t : _tris)
        _os << (1 + t[0]) << " " << (1 + t[1]) << " " << -(1 + t[2]) << "\n";
    }

};

#endif
