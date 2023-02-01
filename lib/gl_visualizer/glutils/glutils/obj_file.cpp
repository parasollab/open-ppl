#include "glutils/obj_file.h"
#include "glutils/triangulated_model.h"

#include <fstream>

#include "nonstd/exception.h"
#include "nonstd/string.h"


namespace glutils {

  obj_file&
  obj_file::
  operator>>(triangulated_model& _t)
  {
    std::ifstream file(m_filename);
    if(!file.good())
      throw nonstd::exception(WHERE) << "Could not open the file '"
                                     << m_filename << "'.";

    char c1, c2;
    std::string buffer;
    while(file >> c1)
    {
      file.get(c2);
      if(c2 != ' ')
        continue;
      switch(c1)
      {
        case 'v': // It's a vertex, Jim.
        {
          vector3f v;
          file >> v;
          _t.add_point(v, true);
          break;
        }
        case 'f': // It's a facet, Jim.
        {
          getline(file, buffer);
          auto facets = nonstd::tokenize(buffer);
          size_t a = std::stoi(nonstd::tokenize(facets[0], "/").front());
          size_t b = std::stoi(nonstd::tokenize(facets[1], "/").front());
          size_t c = std::stoi(nonstd::tokenize(facets[2], "/").front());
          _t.add_facet(a - 1, b - 1, c - 1);
          break;
        }
        default: // It's a comment or unsupported string, Jim.
          getline(file, buffer); // Eat line.
      }
    }

    return *this;
  }


  obj_file&
  obj_file::
  operator<<(const triangulated_model& _t)
  {
    std::ofstream file(m_filename);
    if(!file.is_open())
      throw nonstd::exception(WHERE) << "Could not open the file '"
                                     << m_filename << "'.";

    if(!m_message.str().empty())
      file << m_message.str() << std::endl;

    auto corners = _t.find_aabb_corners();
    auto& min = corners.first,
        & max = corners.second;

    file << "# Vertices: " << _t.num_points()
         << "\n# Facets: " << _t.num_facets()
         << "\n# Centroid: " << _t.find_centroid()
         << "\n# AABB ranges: [" << min[0] << ":" << max[0] << " "
                                 << min[1] << ":" << max[1] << " "
                                 << min[2] << ":" << max[2] << "]"
         << "\n# AABB center: " << _t.find_aabb_center()
         << std::endl;

    file << std::fixed;
    for(auto p = _t.points_begin(); p != _t.points_end(); ++p)
      file << "v " << (*p)[0] << " " << (*p)[1] << " " << (*p)[2] << std::endl;
    for(auto f = _t.facets_begin(); f != _t.facets_end(); ++f)
      file << "f " << (*f)[0] + 1 << " " << (*f)[1] + 1
           << " "  << (*f)[2] + 1 << std::endl;

    return *this;
  }


  std::ostringstream&
  obj_file::
  msg()
  {
    return m_message;
  }

}
