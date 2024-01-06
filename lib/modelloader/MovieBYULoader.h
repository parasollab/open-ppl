#ifndef MOVIE_BYU_LOADER_H_
#define MOVIE_BYU_LOADER_H_

#include <fstream>
#include <utility>
#include <vector>

#include "IModel.h"

////////////////////////////////////////////////////////////////////////////////
/// \brief A loader for .BYU files. Requires the polygons to be triangles.
////////////////////////////////////////////////////////////////////////////////
class CMovieBYULoader : public IModel {

  public:

    ///\name IModel Overriedes
    ///@{

    virtual bool ParseFile(bool _silent = false) override;

    ///@}

  private:

    ///\name Parsing Helpers
    ///@{

    bool ParseHeader(std::ifstream& _in);   ///< Parse the header section.
    bool ParseVertices(std::ifstream& _in); ///< Parse the vertex section.
    bool ParsePolygons(std::ifstream& _in); ///< Parse the polygon section.

    ///@}
    ///\name Internal State
    ///@{

    int m_partsSize{0};   ///< The number of parts in the model.
    int m_vertexSize{0};  ///< The number of vertices in the model.
    int m_polygonSize{0}; ///< The number of polygons in the model.
    int m_edgeSize{0};    ///< The number of edges in the model.

    std::vector<std::pair<int,int>> m_parts; ///< Start/end polygon indexes for
                                             ///< each part.

    ///@}
};

#endif
