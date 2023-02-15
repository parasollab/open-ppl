#include "glutils/triangulated_model.h"

#include <algorithm>
#include <list>
#include <map>

#include "nonstd/exception.h"


namespace glutils {

  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Triangle Facet ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  /*----------------------------- Construction -------------------------------*/

  triangle_facet::
  triangle_facet(
      const index _i1,
      const index _i2,
      const index _i3,
      const point_list& _pl
  )
  : m_indexes{_i1, _i2, _i3},
    m_points(_pl)
  {
    // Rotate the vertex list so that the lowest index is always first. This
    // helps check for equality efficiently.
    auto iter = std::min_element(m_indexes.begin(), m_indexes.end());
    std::rotate(m_indexes.begin(), iter, m_indexes.end());
    compute_normal();
  }

  /*----------------------------- Accessors ----------------------------------*/

  triangle_facet::index_iterator
  triangle_facet::
  begin() const noexcept
  {
    return m_indexes.begin();
  }


  triangle_facet::index_iterator
  triangle_facet::
  end() const noexcept
  {
    return m_indexes.end();
  }


  triangle_facet::index
  triangle_facet::
  operator[](
      const size_t _i
  ) const noexcept
  {
    return m_indexes[_i];
  }


  const triangle_facet::point&
  triangle_facet::
  get_point(
      const size_t _i
  ) const noexcept
  {
    return m_points[_i];
  }


  const vector3f&
  triangle_facet::
  get_normal() const noexcept
  {
    return m_normal;
  }

  /*----------------------------- Modifiers ----------------------------------*/

  triangle_facet&
  triangle_facet::
  reverse() noexcept
  {
    std::reverse(m_indexes.begin() + 1, m_indexes.end());
    m_normal *= -1;

    return *this;
  }

  /*------------------------------ Equality ----------------------------------*/

  bool
  triangle_facet::
  operator==(
      const triangle_facet& _t
  ) const noexcept
  {
    // If both facets have the same point list, we can just check the indexes.
    if(&m_points == &_t.m_points)
    {
      for(size_t i = 0; i < 3; ++i)
        if(m_indexes[i] != _t.m_indexes[i])
          return false;
    }
    // Otherwise, we need to check the actual points.
    else
    {
      for(size_t i = 0; i < 3; ++i)
        if(get_point(i) != _t.get_point(i))
          return false;
    }
    return true;
  }


  bool
  triangle_facet::
  operator!=(
      const triangle_facet& _t
  ) const noexcept
  {
    return !(*this == _t);
  }

  /*------------------------------ Ordering ----------------------------------*/

  bool
  triangle_facet::
  operator<(
      const triangle_facet& _t
  ) const noexcept
  {
    for(size_t i = 0; i < 3; ++i)
    {
      if(m_indexes[0] < _t.m_indexes[0])
        return true;
      else if(_t.m_indexes[0] < m_indexes[0])
        return false;
    }
    return false;
  }

  /*------------------------------ Helpers -----------------------------------*/

  void
  triangle_facet::
  compute_normal() noexcept
  {
    m_normal = (get_point(1) - get_point(0)) % (get_point(2) - get_point(0));
    m_normal.normalize();
  }

  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~ Triangulated Model ~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  /*--------------------------- Creation Interface ---------------------------*/

  size_t
  triangulated_model::
  add_point(
      const point& _p,
      const bool _duplicates
  )
  {
    // If not duplicating points, first check for an existing copy of _p.
    if(!_duplicates)
    {
      auto iter = std::find(m_points.begin(), m_points.end(), _p);
      if(iter != m_points.end())
        return std::distance(m_points.begin(), iter);
    }

    // We are either not checking duplicates or _p isn't in the point set. Add
    // it and return the new index.
    m_points.push_back(_p);
    return m_points.size() - 1;
  }


  size_t
  triangulated_model::
  add_facet(
      const size_t _i1,
      const size_t _i2,
      const size_t _i3
  )
  {
    m_facets.emplace_back(_i1, _i2, _i3, m_points);
    return m_facets.size() - 1;
  }


  void
  triangulated_model::
  add_model(
      const triangulated_model& _t
  )
  {
    // Add points from _t to this, keeping track of the mapping from old index to
    // new index.
    std::vector<size_t> indexMap(_t.m_points.size(), 0);

    for(size_t i = 0; i < _t.m_points.size(); ++i)
      indexMap[i] = add_point(_t.get_point(i));

    for(auto iter = _t.facets_begin(); iter != _t.facets_end(); ++iter)
    {
      const auto& facet = *iter;
      add_facet(indexMap[facet[0]], indexMap[facet[1]], indexMap[facet[2]]);
    }
  }

  /*------------------------------- Accessors --------------------------------*/

  const triangulated_model::point&
  triangulated_model::
  get_point(
      const size_t _i
  ) const noexcept
  {
    return m_points[_i];
  }


  const triangulated_model::facet&
  triangulated_model::
  get_facet(
      const size_t _i
  ) const noexcept
  {
    return m_facets[_i];
  }


  triangulated_model::const_point_iterator
  triangulated_model::
  points_begin() const noexcept
  {
    return m_points.begin();
  }


  triangulated_model::const_point_iterator
  triangulated_model::
  points_end() const noexcept
  {
    return m_points.end();
  }


  triangulated_model::const_facet_iterator
  triangulated_model::
  facets_begin() const noexcept
  {
    return m_facets.begin();
  }


  triangulated_model::const_facet_iterator
  triangulated_model::
  facets_end() const noexcept
  {
    return m_facets.end();
  }


  triangulated_model::point_iterator
  triangulated_model::
  points_begin() noexcept
  {
    return m_points.begin();
  }


  triangulated_model::point_iterator
  triangulated_model::
  points_end() noexcept
  {
    return m_points.end();
  }


  triangulated_model::facet_iterator
  triangulated_model::
  facets_begin() noexcept
  {
    return m_facets.begin();
  }


  triangulated_model::facet_iterator
  triangulated_model::
  facets_end() noexcept
  {
    return m_facets.end();
  }

  /*------------------------------- Queries ----------------------------------*/

  size_t
  triangulated_model::
  num_points() const noexcept
  {
    return m_points.size();
  }


  size_t
  triangulated_model::
  num_facets() const noexcept
  {
    return m_facets.size();
  }


  vector3f
  triangulated_model::
  find_centroid() const noexcept
  {
    vector3f center;
    for(const auto& p : m_points)
      center += p;
    return center /= m_points.size();
  }


  vector3f
  triangulated_model::
  find_aabb_center() const noexcept
  {
    std::pair<vector3f, vector3f> corners = find_aabb_corners();
    return (corners.first + corners.second) / 2;
  }


  std::pair<vector3f, vector3f>
  triangulated_model::
  find_aabb_corners() const noexcept
  {
    constexpr GLfloat lowest  = std::numeric_limits<GLfloat>::lowest(),
                      highest = std::numeric_limits<GLfloat>::max();

    // Define the min and max corners.
    std::pair<vector3f, vector3f> corners{{highest, highest, highest},
                                          {lowest, lowest, lowest}};
    auto& min = corners.first,
        & max = corners.second;

    for(const auto& p : m_points)
    {
      for(size_t i = 0; i < 3; ++i)
      {
        min[i] = std::min(min[i], p[i]);
        max[i] = std::max(max[i], p[i]);
      }
    }

    return corners;
  }

  /*------------------------------ Modifiers ---------------------------------*/

  triangulated_model&
  triangulated_model::
  translate(
      const vector3f& _v
  ) noexcept
  {
    for(auto& p : m_points)
      p += _v;

    return *this;
  }


  triangulated_model&
  triangulated_model::
  rotate(
      const vector3f& _v
  ) noexcept
  {
    for(auto& p : m_points)
      p.rotate(_v);

    return *this;
  }


  triangulated_model&
  triangulated_model::
  clean() noexcept
  {
    // Remove vertices that don't refer to anything.

    // Create a map from each vertex index to the facets that contain it.
    std::map<size_t, std::list<facet*>> index_map;

    // Add each index to the map with an empty facet list.
    for(size_t i = 0; i < num_points(); ++i)
      index_map[i];

    // Iterate through facets and add them to the map.
    for(auto iter = m_facets.begin(); iter != m_facets.end(); ++iter)
      for(const auto i : *iter)
        index_map[i].push_back(&*iter);

    // Remove unused vertices, starting from the back to avoid having to
    // recompute the indexes.
    /// @TODO Fix this silly O(n^2) algorithm and replace with O(n) solution by
    ///       re-writing a new point list and swapping.
    for(auto iter = index_map.rbegin(); iter != index_map.rend(); ++iter)
    {
      const bool unused = iter->second.empty();
      if(unused)
        m_points.erase(m_points.begin() + iter->first);
    }

    // When we removed the unused vertices, we shifted the indexes of the
    // remaining points. We will need to adjust the indexes in all of the facets
    // appropriately. First, figure out how much each index needs to change.
    std::map<size_t, size_t> change_map;
    size_t offset = 0;
    for(const auto& pair : index_map)
    {
      const bool unused = pair.second.empty();
      offset += unused;
      if(!unused)
        change_map[pair.first] = offset;
    }

    // Now iterate through the facets and adjust the indexes using the change
    // map.
    for(auto iter = m_facets.begin(); iter != m_facets.end(); ++iter)
      for(auto& index : iter->m_indexes)
        index -= change_map.at(index);

    return *this;
  }


  triangulated_model&
  triangulated_model::
  reverse() noexcept
  {
    for(auto& facet : m_facets)
      facet.reverse();
    return *this;
  }

  /*------------------------------ Equality ----------------------------------*/

  bool
  triangulated_model::
  operator==(
      const triangulated_model& _t
  ) const noexcept
  {
    // First check that the number of points are the same.
    if(num_points() != _t.num_points())
      return false;

    // If the number of points are the same, test facets.
    // ARG facets can be in a different order @#$%
    // ARG referenced points can be in a different order @#$%
    return m_facets == _t.m_facets;
  }


  bool
  triangulated_model::
  operator!=(
      const triangulated_model& _t
  ) const noexcept
  {
    return !(*this == _t);
  }

  /*---------------------------- Common Shapes -------------------------------*/

  triangulated_model
  triangulated_model::
  make_box(
      const GLfloat _length_x,
      const GLfloat _length_y,
      const GLfloat _length_z
  )
  {
    // Require sensible values for the input parameters.
    if(_length_x <= 0)
      throw nonstd::exception(WHERE) << "Positive X length required (received "
                                     << _length_x << ").";
    if(_length_y <= 0)
      throw nonstd::exception(WHERE) << "Positive Y length required (received "
                                     << _length_y << ").";
    if(_length_z <= 0)
      throw nonstd::exception(WHERE) << "Positive Z length required (received "
                                     << _length_z << ").";

    // Get half-lengths.
    const GLfloat h_length_x = _length_x / 2,
                  h_length_y = _length_y / 2,
                  h_length_z = _length_z / 2;

    // Make vertices.
    vector3f pts[8] = {{-h_length_x,  h_length_y,  h_length_z},
                       {-h_length_x, -h_length_y,  h_length_z},
                       { h_length_x, -h_length_y,  h_length_z},
                       { h_length_x,  h_length_y,  h_length_z},
                       {-h_length_x,  h_length_y, -h_length_z},
                       {-h_length_x, -h_length_y, -h_length_z},
                       { h_length_x, -h_length_y, -h_length_z},
                       { h_length_x,  h_length_y, -h_length_z}};

    // Add points to the model and get their indexes.
    triangulated_model t;
    size_t id[8];
    for(size_t i = 0; i < 8; ++i)
      id[i] = t.add_point(pts[i]);

    // Make facets.
    size_t facets[12][3] = {{id[0], id[1], id[2]},
                            {id[0], id[2], id[3]},
                            {id[3], id[2], id[6]},
                            {id[3], id[6], id[7]},
                            {id[7], id[6], id[5]},
                            {id[7], id[5], id[4]},
                            {id[4], id[5], id[1]},
                            {id[4], id[1], id[0]},
                            {id[0], id[3], id[7]},
                            {id[0], id[7], id[4]},
                            {id[1], id[5], id[6]},
                            {id[1], id[6], id[2]}};

    // Add facets.
    for(size_t i = 0; i < 12; ++i)
      t.add_facet(facets[i][0], facets[i][1], facets[i][2]);

    return t;
  }


  triangulated_model
  triangulated_model::
  make_sphere(
      const GLfloat _radius,
      const size_t _segments
  )
  {
    // Require sensible values for the input parameters.
    if(_radius <= 0)
      throw nonstd::exception(WHERE) << "Positive radius required (received "
                                     << _radius << ").";
    if(_segments < 3)
      throw nonstd::exception(WHERE) << "Minimum of three segments are required "
                                     << "(received " << _segments << ").";

    triangulated_model t;

    const GLfloat zIncr = glutils::PI / _segments; // Angle increment for z.
    const GLfloat oIncr = 2 * zIncr;               // Angle increment for x,y.

    GLfloat x, y, z, r;

    // Draw +zHat cap.
    {
      // Create the +zHat pole.
      x = 0;
      y = 0;
      z = _radius;
      const size_t capIndex = t.add_point({x, y, z});

      // Create the ring of points beneath the pole.
      std::vector<size_t> indexes;;

      z = _radius * std::cos(zIncr);
      r = _radius * std::sin(zIncr);
      for(size_t i = 0; i < _segments; ++i)
      {
        x = r * std::cos(oIncr * i);
        y = r * std::sin(oIncr * i);
        indexes.push_back(t.add_point({x, y, z}));
      }

      // Create the facets connecting the pole to the point ring.
      for(size_t i = 0; i < _segments; ++i)
        t.add_facet(capIndex, indexes[i], indexes[(i + 1) % _segments]);
    }

    // Draw main surface.
    {
      GLfloat z2, r2;

      // Create a ring of segments following the previous.
      std::vector<size_t> topIndexes, bottomIndexes;

      for(size_t j = 1; j < _segments - 1; ++j)
      {
        // The top and bottom point rings in this segment ring each require a
        // different z and planar radius.
        z  = _radius * std::cos(zIncr * j);
        r  = _radius * std::sin(zIncr * j);
        z2 = _radius * std::cos(zIncr * (j + 1));
        r2 = _radius * std::sin(zIncr * (j + 1));

        // Generate the points for this segment ring.
        topIndexes.clear();
        bottomIndexes.clear();
        for(size_t i = 0; i < _segments; ++i)
        {
          x = std::cos(oIncr * i);
          y = std::sin(oIncr * i);
          topIndexes.push_back(   t.add_point({x * r , y * r ,  z}));
          bottomIndexes.push_back(t.add_point({x * r2, y * r2, z2}));
        }

        // Create facets to complete this segment ring.
        for(size_t i = 0; i < _segments; ++i)
        {
          const size_t i2 = (i + 1) % _segments;
          t.add_facet(topIndexes[i] , bottomIndexes[i], topIndexes[i2]);
          t.add_facet(topIndexes[i2], bottomIndexes[i], bottomIndexes[i2]);
        }
      }
    }

    // Draw -zHat cap.
    {
      // Create the +zHat pole.
      x = 0;
      y = 0;
      z = -_radius;
      const size_t capIndex = t.add_point({x, y, z});

      // Create the ring of points above the pole.
      std::vector<size_t> indexes;

      z = _radius * std::cos(zIncr * (_segments - 1));
      r = _radius * std::sin(zIncr * (_segments - 1));
      for(size_t i = 0; i < _segments; ++i)
      {
        x = r * std::cos(oIncr * i);
        y = r * std::sin(oIncr * i);
        indexes.push_back(t.add_point({x, y, z}));
      }

      // Create the facets connecting the pole to the point ring.
      for(size_t i = 0; i < _segments; ++i)
        t.add_facet(capIndex, indexes[(i + 1) % _segments], indexes[i]);
    }

    return t;
  }


  triangulated_model
  triangulated_model::
  make_cone(
      const GLfloat _radius,
      const GLfloat _height,
      const size_t _segments
  )
  {
    // Require sensible values for the input parameters.
    if(_radius <= 0)
      throw nonstd::exception(WHERE) << "Positive radius required (received "
                                     << _radius << ").";
    if(_height <= 0)
      throw nonstd::exception(WHERE) << "Positive height is required (received "
                                     << _height << ").";
    if(_segments < 3)
      throw nonstd::exception(WHERE) << "Minimum of three segments are required "
                                     << "(received " << _segments << ").";

    triangulated_model t;

    const GLfloat incr = 2. * glutils::PI / _segments;
    GLfloat x, y, z;

    // Create the point ring.
    z = 0;
    std::vector<size_t> ringIndexes;
    for(size_t i = 0; i < _segments; ++i)
    {
      x = _radius * std::cos(incr * i);
      y = _radius * std::sin(incr * i);
      ringIndexes.push_back(t.add_point({x, y, z}));
    }

    // Create the circular base on the x-y plane.
    {
      // Create the cap.
      x = 0;
      y = 0;
      z = 0;
      const size_t capIndex = t.add_point({x, y, z});

      // Create facets connecting the cap to the point ring.
      for(size_t i = 0; i < _segments; ++i)
        t.add_facet(capIndex, ringIndexes[i], ringIndexes[(i + 1) % _segments]);
    }

    // Create the conic cap on top of the circle.
    {
      // Create the cap.
      x = 0;
      y = 0;
      z = -_height;
      const size_t capIndex = t.add_point({x, y, z});

      // Create facets connecting the cap to the point ring.
      for(size_t i = 0; i < _segments; ++i)
        t.add_facet(capIndex, ringIndexes[(i + 1) % _segments], ringIndexes[i]);
    }

    // Translate the model so that it is centered on its bounding box.
    t.translate({0, 0, _height/2});
    return t;
  }


  triangulated_model
  triangulated_model::
  make_cylinder(
      const GLfloat _radius,
      const GLfloat _length,
      const size_t _segments
  )
  {
    // Require sensible values for the input parameters.
    if(_length <= 0)
      throw nonstd::exception(WHERE) << "Positive length is required (received "
                                     << _length << ").";
    if(_segments < 3)
      throw nonstd::exception(WHERE) << "Minimum of three segments are required "
                                     << "(received " << _segments << ").";
    if(_radius <= 0)
      throw nonstd::exception(WHERE) << "Positive radius required (received "
                                     << _radius << ").";

    triangulated_model t;

    const GLfloat incr       = 2. * glutils::PI / _segments,
                  halfLength = _length / 2.;
    GLfloat x, y, z;

    // Draw top cap.
    x = 0;
    y = 0;
    z = halfLength;
    const size_t topCapIndex = t.add_point({x, y, z});

    // Create the top point ring.
    std::vector<size_t> topIndexes;
    for(size_t i = 0; i < _segments; ++i)
    {
      x = _radius * std::cos(incr * i);
      y = _radius * std::sin(incr * i);
      topIndexes.push_back(t.add_point({x, y, z}));
    }

    // Create facets connecting the cap to the point ring.
    for(size_t i = 0; i < _segments; ++i)
      t.add_facet(topCapIndex, topIndexes[i], topIndexes[(i + 1) % _segments]);

    // Draw bottom cap.
    x = 0;
    y = 0;
    z = -halfLength;
    const size_t bottomCapIndex = t.add_point({x, y, z});

    // Create the bottom point ring.
    std::vector<size_t> bottomIndexes;
    for(size_t i = 0; i < _segments; ++i)
    {
      x = _radius * std::cos(incr * i);
      y = _radius * std::sin(incr * i);
      bottomIndexes.push_back(t.add_point({x, y, z}));
    }

    // Create facets connecting the cap to the point ring.
    for(size_t i = 0; i < _segments; ++i)
      t.add_facet(bottomCapIndex, bottomIndexes[(i + 1) % _segments],
                  bottomIndexes[i]);

    // Create facets connecting the top and bottom rings.
    for(size_t i = 0; i < _segments; ++i)
    {
      const size_t i2 = (i + 1) % _segments;
      t.add_facet(topIndexes[i] , bottomIndexes[i], topIndexes[i2]);
      t.add_facet(topIndexes[i2], bottomIndexes[i], bottomIndexes[i2]);
    }

    return t;
  }


  triangulated_model
  triangulated_model::
  make_pipe(
      const GLfloat _outerRadius,
      const GLfloat _innerRadius,
      const GLfloat _length,
      const size_t _segments
  )
  {
    // Require sensible values for the input parameters.
    if(_length <= 0)
      throw nonstd::exception(WHERE) << "Positive length is required (received "
                                     << _length << ").";
    if(_segments < 3)
      throw nonstd::exception(WHERE) << "Minimum of three segments are required "
                                     << "(received " << _segments << ").";
    if(_innerRadius <= 0)
      throw nonstd::exception(WHERE) << "Positive inner radius required "
                                     << "(received " << _innerRadius << ").";
    if(_outerRadius <= 0)
      throw nonstd::exception(WHERE) << "Positive outer radius required "
                                     << "(received " << _outerRadius << ").";
    if(_innerRadius >= _outerRadius)
      throw nonstd::exception(WHERE) << "Outer radius (" << _outerRadius << ") "
                                     << "must exceed inner radius ("
                                     << _innerRadius << ").";

    triangulated_model t;

    const GLfloat incr       = 2. * glutils::PI / _segments,
                  halfLength = _length / 2.;
    GLfloat x, y, z;

    // Create the point rings.
    z = halfLength;
    std::vector<size_t> topOuter, topInner, bottomOuter, bottomInner;
    for(size_t i = 0; i < _segments; ++i)
    {
      x = std::cos(incr * i);
      y = std::sin(incr * i);

      topOuter.push_back(t.add_point({x * _outerRadius, y * _outerRadius, z}));
      topInner.push_back(t.add_point({x * _innerRadius, y * _innerRadius, z}));

      bottomOuter.push_back(t.add_point({x * _outerRadius, y * _outerRadius, -z}));
      bottomInner.push_back(t.add_point({x * _innerRadius, y * _innerRadius, -z}));
    }

    // Create facets.
    for(size_t i = 0; i < _segments; ++i)
    {
      const size_t i2 = (i + 1) % _segments;

      // Connect top and bottom rings:

      // Outer facets.
      t.add_facet(topOuter[i] , bottomOuter[i], topOuter[i2]);
      t.add_facet(topOuter[i2], bottomOuter[i], bottomOuter[i2]);

      // Inner facets.
      t.add_facet(topInner[i] , topInner[i2]   , bottomInner[i]);
      t.add_facet(topInner[i2], bottomInner[i2], bottomInner[i]);

      // Connect inner and outer rings:

      // Top facets.
      t.add_facet(topOuter[i] , topOuter[i2], topInner[i2]);
      t.add_facet(topInner[i2], topInner[i] , topOuter[i]);

      // Bottom facets.
      t.add_facet(bottomOuter[i] , bottomInner[i2], bottomOuter[i2]);
      t.add_facet(bottomInner[i2], bottomOuter[i] , bottomInner[i]);
    }

    return t;
  }


  triangulated_model
  triangulated_model::
  make_capped_cylinder(
      const GLfloat _radius,
      const GLfloat _firstSphereRadius,
      const GLfloat _secondSphereRadius,
      const GLfloat _length,
      const size_t _segments
  )
  {
    const bool useFirstSphere = _firstSphereRadius != 0.,
               useSecondSphere = _secondSphereRadius != 0.;

    // Require sensible values for the input parameters.
    if(_radius <= 0)
      throw nonstd::exception(WHERE) << "Positive radius required (received "
                                     << _radius << ").";
    if(useFirstSphere and _firstSphereRadius < _radius)
      throw nonstd::exception(WHERE) << "First sphere radius " << _firstSphereRadius
                                     << " must not be less than cylinder radius "
                                     << _radius << ".";
    if(useSecondSphere and _secondSphereRadius < _radius)
      throw nonstd::exception(WHERE) << "Second sphere radius " << _secondSphereRadius
                                     << " must not be less than cylinder radius "
                                     << _radius << ".";
    if(_length <= 0)
      throw nonstd::exception(WHERE) << "Positive length is required (received "
                                     << _length << ").";
    if(_length <= _firstSphereRadius + _secondSphereRadius)
      throw nonstd::exception(WHERE) << "Cylinder length " << _length
                                     << " should be longer than the summed sphere "
                                     << "radii " << _firstSphereRadius << " + "
                                     << _secondSphereRadius << " = "
                                     << _firstSphereRadius + _secondSphereRadius
                                     << ".";
    if(_segments < 3)
      throw nonstd::exception(WHERE) << "Minimum of three segments are required "
                                     << "(received " << _segments << ").";

    triangulated_model t;

    const GLfloat halfLength = _length / 2.,
                  zIncr = glutils::PI / _segments, // Angle increment for z.
                  oIncr = 2 * zIncr;               // Angle increment for x,y.
    GLfloat x, y, z, r;

    // Start drawing first sphere or cap. Save the last ring of points for
    // connection to the bottom later on.
    std::vector<size_t> upperRing;
    // sphere:
    //   make point rings until the radius border would be crossed. then make
    //   top point ring as last ring of the sphere by scaling distance back
    //   until r = _radius.
    if(useFirstSphere) {
      // Create the +zHat pole.
      {
        x = 0;
        y = 0;
        z = _firstSphereRadius + halfLength;
        const size_t capIndex = t.add_point({x, y, z});

        // Create the ring of points beneath the pole.
        std::vector<size_t> indexes;;

        z = _firstSphereRadius * std::cos(zIncr) + halfLength;
        r = _firstSphereRadius * std::sin(zIncr);
        for(size_t i = 0; i < _segments; ++i)
        {
          x = r * std::cos(oIncr * i);
          y = r * std::sin(oIncr * i);
          indexes.push_back(t.add_point({x, y, z}));
        }

        // Create the facets connecting the pole to the point ring.
        for(size_t i = 0; i < _segments; ++i)
          t.add_facet(capIndex, indexes[i], indexes[(i + 1) % _segments]);
      }

      // Draw main surface.
      {
        GLfloat z2, r2;

        // Create a ring of segments following the previous.
        std::vector<size_t> topIndexes, bottomIndexes;

        for(size_t j = 1; j < _segments - 1; ++j)
        {
          // The top and bottom point rings in this segment ring each require a
          // different z and planar radius.
          z  = _firstSphereRadius * std::cos(zIncr * j) + halfLength;
          r  = _firstSphereRadius * std::sin(zIncr * j);
          z2 = _firstSphereRadius * std::cos(zIncr * (j + 1)) + halfLength;
          r2 = _firstSphereRadius * std::sin(zIncr * (j + 1));

          // Test for crossing the _radius boundary on the descending side.
          const bool crossed = r2 <= r
                           and r  >= _radius
                           and r2 <= _radius;
          if(crossed) {
            r2 = _radius;
            const GLfloat angle = std::asin(r2 / _firstSphereRadius);
            z2 = -_firstSphereRadius * std::cos(angle)
               + halfLength;
          }

          // Generate the points for this segment ring.
          topIndexes.clear();
          bottomIndexes.clear();
          for(size_t i = 0; i < _segments; ++i)
          {
            x = std::cos(oIncr * i);
            y = std::sin(oIncr * i);
            topIndexes.push_back(   t.add_point({x * r , y * r ,  z}));
            bottomIndexes.push_back(t.add_point({x * r2, y * r2, z2}));
          }

          // Create facets to complete this segment ring.
          for(size_t i = 0; i < _segments; ++i)
          {
            const size_t i2 = (i + 1) % _segments;
            t.add_facet(topIndexes[i] , bottomIndexes[i], topIndexes[i2]);
            t.add_facet(topIndexes[i2], bottomIndexes[i], bottomIndexes[i2]);
          }

          // If we crossed the border, save the upper points and break out.
          if(crossed) {
            upperRing = std::move(bottomIndexes);
            break;
          }
        }
      }
    }
    // cap:
    //   make top point ring
    //   join to center point
    else {
      // Create the cap point.
      const size_t topCapIndex = t.add_point({0, 0, halfLength});

      // Create the point ring.
      for(size_t i = 0; i < _segments; ++i)
      {
        x = _radius * std::cos(oIncr * i);
        y = _radius * std::sin(oIncr * i);
        upperRing.push_back(t.add_point({x, y, halfLength}));
      }

      // Create facets connecting the cap to the point ring.
      for(size_t i = 0; i < _segments; ++i)
        t.add_facet(topCapIndex, upperRing[i], upperRing[(i + 1) % _segments]);
    }

    // Start drawing second sphere or cap.
    std::vector<size_t> lowerRing;
    // if sphere:
    //   skip point rings until the radius border would be crossed. make bottom
    //   point ring as the first ring of the sphere by scaling distance back
    //   until r = _radius.
    if(useSecondSphere) {
      // Draw main surface.
      {
        GLfloat z2, r2;

        // Create a ring of segments following the previous.
        std::vector<size_t> topIndexes, bottomIndexes;

        bool crossed = false;

        for(size_t j = 1; j < _segments - 1; ++j)
        {
          // The top and bottom point rings in this segment ring each require a
          // different z and planar radius.
          z  = _secondSphereRadius * std::cos(zIncr * j) - halfLength;
          r  = _secondSphereRadius * std::sin(zIncr * j);
          z2 = _secondSphereRadius * std::cos(zIncr * (j + 1)) - halfLength;
          r2 = _secondSphereRadius * std::sin(zIncr * (j + 1));

          // Test for crossing the _radius boundary on the ascending side.
          bool justCrossed = false;
          if(!crossed) {
            justCrossed = r2 >= r
                      and r  <= _radius
                      and r2 >= _radius;
            if(justCrossed) {
              r = _radius;
              const GLfloat angle = std::asin(r / _secondSphereRadius);
              z = _secondSphereRadius * std::cos(angle) - halfLength;
              crossed = true;
            }
            else
              continue; // Skip this ring.
          }

          // Generate the points for this segment ring.
          topIndexes.clear();
          bottomIndexes.clear();
          for(size_t i = 0; i < _segments; ++i)
          {
            x = std::cos(oIncr * i);
            y = std::sin(oIncr * i);
            topIndexes.push_back(   t.add_point({x * r , y * r ,  z}));
            bottomIndexes.push_back(t.add_point({x * r2, y * r2, z2}));
          }

          // Create facets to complete this segment ring.
          for(size_t i = 0; i < _segments; ++i)
          {
            const size_t i2 = (i + 1) % _segments;
            t.add_facet(topIndexes[i] , bottomIndexes[i], topIndexes[i2]);
            t.add_facet(topIndexes[i2], bottomIndexes[i], bottomIndexes[i2]);
          }

          // If we crossed the border, save the upper points and break out.
          if(justCrossed)
            lowerRing = topIndexes;
        }
      }

      // Draw -zHat cap.
      {
        // Create the +zHat pole.
        z = -_secondSphereRadius - halfLength;
        const size_t capIndex = t.add_point({0, 0, z});

        // Create the ring of points above the pole.
        std::vector<size_t> indexes;

        z = _secondSphereRadius * std::cos(zIncr * (_segments - 1)) - halfLength;
        r = _secondSphereRadius * std::sin(zIncr * (_segments - 1));
        for(size_t i = 0; i < _segments; ++i)
        {
          x = r * std::cos(oIncr * i);
          y = r * std::sin(oIncr * i);
          indexes.push_back(t.add_point({x, y, z}));
        }

        // Create the facets connecting the pole to the point ring.
        for(size_t i = 0; i < _segments; ++i)
          t.add_facet(capIndex, indexes[(i + 1) % _segments], indexes[i]);
      }
    }
    // if cap:
    //   make bottom point ring
    //   joint to center point
    else {
      // Draw cap point.
      const size_t bottomCapIndex = t.add_point({0, 0, -halfLength});

      // Create the point ring.
      for(size_t i = 0; i < _segments; ++i)
      {
        x = _radius * std::cos(oIncr * i);
        y = _radius * std::sin(oIncr * i);
        lowerRing.push_back(t.add_point({x, y, -halfLength}));
      }

      // Create facets connecting the cap to the point ring.
      for(size_t i = 0; i < _segments; ++i)
        t.add_facet(bottomCapIndex, lowerRing[(i + 1) % _segments],
                    lowerRing[i]);
    }

    // Create facets connecting the top and bottom rings.
    for(size_t i = 0; i < _segments; ++i)
    {
      const size_t i2 = (i + 1) % _segments;
      t.add_facet(upperRing[i] , lowerRing[i], upperRing[i2]);
      t.add_facet(upperRing[i2], lowerRing[i], lowerRing[i2]);
    }

    return t;
  }

  /*--------------------------------------------------------------------------*/
}
