#ifndef GLUTILS_TRIANGULATED_MODEL_H_
#define GLUTILS_TRIANGULATED_MODEL_H_

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

#include "glutils/gltraits.h"


namespace glutils {

  //////////////////////////////////////////////////////////////////////////////
  /// A triangular, right-handed facet for a polygonal model.
  //////////////////////////////////////////////////////////////////////////////
  class triangle_facet final
  {

    ///@name Local Types
    ///@{

    typedef size_t                     index;
    typedef std::array<index, 3>       index_list;
    typedef index_list::const_iterator index_iterator;

    typedef glutils::vector3f          point;
    typedef std::vector<point>         point_list;

    ///@}
    ///@name Internal State
    ///@{

    index_list m_indexes;       ///< The indexes of m_points in this facet.
    const point_list& m_points; ///< A reference to the owning model's points.

    vector3f m_normal;          ///< The right-handed normal.

    ///@}

    public:

      ///@name Construction
      ///@{

      /// Construct a triangular facet from three indexes and a reference to the
      /// owning model's point list.
      /// @param _i1 The first point index.
      /// @param _i2 The second point index.
      /// @param _i3 The third point index.
      /// @param _pl The owning model's point list.
      triangle_facet(
          const index _i1,
          const index _i2,
          const index _i3,
          const point_list& _pl
      );

      ///@}
      ///@name Accessors
      ///@{

      /// Get a begin iterator to the point indexes.
      index_iterator
      begin() const noexcept;

      /// Get an end iterator to the point indexes.
      index_iterator
      end() const noexcept;

      /// Get the index (in the owning model) of a facet point.
      /// @param _i The index (in this facet) of the point.
      /// @return The index of point _i in the owning model.
      index
      operator[](
          const size_t _i
      ) const noexcept;

      /// Get a facet point.
      /// @param _i The index (in this facet) of the point.
      /// @return The point in the owning model referenced by facet index _i.
      const point&
      get_point(
          const size_t _i
      ) const noexcept;

      /// Get the facet normal.
      /// @return The normal vector.
      const vector3f&
      get_normal() const noexcept;

      ///@}
      ///@name Modifiers
      ///@{

      /// Reverse the facet so that the normal faces the opposite direction.
      /// @return A reference to self.
      triangle_facet&
      reverse() noexcept;

      ///@}
      ///@name Equality
      ///@{
      /// Determine whether two facets represent the same set of points in the
      /// same order.

      bool
      operator==(
          const triangle_facet& _t
      ) const noexcept;

      bool
      operator!=(
          const triangle_facet& _t
      ) const noexcept;

      ///@}
      ///@name Ordering
      ///@{

      /// Defines a weak ordering to allow sorting.
      bool
      operator<(
          const triangle_facet& _t
      ) const noexcept;

      ///@}

      friend class triangulated_model;

    private:

      ///@name Helpers
      ///@{

      /// Compute the facet normal.
      void
      compute_normal() noexcept;

      ///@}

  };

  //////////////////////////////////////////////////////////////////////////////
  /// A geometric model, represented by vertices and triangular faces.
  //////////////////////////////////////////////////////////////////////////////
  class triangulated_model final
  {

    ///@name Internal Types
    ///@{

    typedef glutils::vector3f          point;
    typedef std::vector<point>         point_list;

    typedef glutils::triangle_facet    facet;
    typedef std::vector<facet>         facet_list;

    ///@}
    ///@name Internal State
    ///@{

    point_list m_points;  ///< The vertices of this model.
    facet_list m_facets;  ///< The facets of this model.

    ///@}

    public:

      ///@name Local Types
      ///@{

      typedef point_list::iterator       point_iterator;
      typedef facet_list::iterator       facet_iterator;

      typedef point_list::const_iterator const_point_iterator;
      typedef facet_list::const_iterator const_facet_iterator;

      ///@}
      ///@name Creation Interface
      ///@{

      /// Add a point to the model.
      /// @param _p The point to add.
      /// @param _duplicates Allow duplicate points?
      /// @return The index of the added point.
      size_t
      add_point(
          const point& _p,
          const bool _duplicates = false
      );

      /// Add a facet to the model by referencing the indexes of existing points.
      /// @param _i1 The index of the first point.
      /// @param _i1 The index of the second point.
      /// @param _i1 The index of the third point.
      /// @return The index of the added facet.
      size_t
      add_facet(
          const size_t _i1,
          const size_t _i2,
          const size_t _i3
      );

      /// Copy the points and facets from another triangulated model into this
      /// one.
      /// @param _t The source model.
      void
      add_model(
          const triangulated_model& _t
      );

      ///@}
      ///@name Accessors
      ///@{

      /// Get a point in the model.
      /// @param _i The point index.
      /// @return The _ith point in the model.
      const point&
      get_point(
          const size_t _i
      ) const noexcept;

      /// Get a facet in the model.
      /// @param _i The facet index.
      /// @return The _ith facet in the model.
      const facet&
      get_facet(
          const size_t _i
      ) const noexcept;

      /// Get a begin iterator to the model points.
      const_point_iterator
      points_begin() const noexcept;

      /// Get an end iterator to the model points.
      const_point_iterator
      points_end() const noexcept;

      /// Get a begin iterator to the model facets.
      const_facet_iterator
      facets_begin() const noexcept;

      /// Get an end iterator to the model facets.
      const_facet_iterator
      facets_end() const noexcept;

      /// Get a begin iterator to the model points.
      point_iterator
      points_begin() noexcept;

      /// Get an end iterator to the model points.
      point_iterator
      points_end() noexcept;

      /// Get a begin iterator to the model facets.
      facet_iterator
      facets_begin() noexcept;

      /// Get an end iterator to the model facets.
      facet_iterator
      facets_end() noexcept;

      ///@}
      ///@name Queries
      ///@{

      /// Get the number of vertices in the model.
      size_t
      num_points() const noexcept;

      /// Get the number of facets in the model.
      size_t
      num_facets() const noexcept;

      /// Find the model's centroid by averaging all of its vertices.
      /// @return The centroid of the model's vertices.
      vector3f
      find_centroid() const noexcept;

      /// Find the center of the model's axis-aligned bounding box.
      /// @return The center of the model's axis-aligned bounding box.
      vector3f
      find_aabb_center() const noexcept;

      /// Find the corners of the model's axis-aligned bounding box.
      /// @return The min, max corners of the model's axis-aligned bounding box.
      std::pair<vector3f, vector3f>
      find_aabb_corners() const noexcept;

      ///@}
      ///@name Modifiers
      ///@{

      /// Translate all of the vertices in the model.
      /// @param _v The translation vector to apply.
      /// @return A reference to self.
      triangulated_model&
      translate(
          const vector3f& _v
      ) noexcept;

      /// Rotate the model about its local frame.
      /// @param _v The rotaton vector (Euler vector format).
      /// @return A reference to self.
      triangulated_model&
      rotate(
          const vector3f& _v
      ) noexcept;

      /// Remove duplicate vertices from the model.
      /// @return A reference to self.
      triangulated_model&
      clean() noexcept;

      /// Reverse the facets so that the normals face the opposite direction.
      /// @return A reference to self.
      triangulated_model&
      reverse() noexcept;

      ///@}
      ///@name Equality
      ///@{
      /// Determine whether two models contain the same facets and vertices.

      bool
      operator==(
          const triangulated_model& _t
      ) const noexcept;

      bool
      operator!=(
          const triangulated_model& _t
      ) const noexcept;

      ///@}
      ///@name Common Shapes
      ///@{

      /// Create a box centered at the origin.
      /// @param _length_x The box length in the x direction.
      /// @param _length_y The box length in the y direction.
      /// @param _length_z The box length in the z direction.
      /// @return A triangulated box model.
      static
      triangulated_model
      make_box(
          const GLfloat _length_x = 1,
          const GLfloat _length_y = 1,
          const GLfloat _length_z = 1
      );

      /// Create a sphere centered at the origin with axis along the z direction.
      /// @param _radius The sphere radius.
      /// @param _segments The number of segments to use. The model will have
      ///                  _segments - 2 rings of _segments squares and 2 rings
      ///                  of _segments triangles, for a total of
      ///                  2 * _segments * (_segments - 1) triangles.
      /// @return A triangulated sphere model.
      static
      triangulated_model
      make_sphere(
          const GLfloat _radius = 1,
          const size_t _segments = 16
      );

      /// Create a cone with the base centered at the origin and tip pointed away
      /// from the camera (-z direction).
      /// @param _radius The radius of the base.
      /// @param _height The height of the cone.
      /// @param _segments The number of segments to use for the sides.
      /// @return A triangulated cone model.
      static
      triangulated_model
      make_cone(
          const GLfloat _radius = 1,
          const GLfloat _height = 1,
          const size_t _segments = 16
      );

      /// Create a cylinder centered at the origin and oriented along the z-axis.
      /// @param _radius The cylinder radius.
      /// @param _length The length perpendicular to the radius.
      /// @param _segments The number of segments to use for the side wall.
      /// @return A triangulated cylinder model.
      static
      triangulated_model
      make_cylinder(
          const GLfloat _radius = 1,
          const GLfloat _length = 1,
          const size_t _segments = 16
      );

      /// Create a hollow cylinder centered at the origin and oriented along the
      /// z-axis.
      /// @param _outerRadius The outer radius.
      /// @param _innerRadius The inner radius.
      /// @param _length The length perpendicular to the radius.
      /// @param _segments The number of segments to use for the side wall.
      /// @return A triangulated cylinder model.
      static
      triangulated_model
      make_pipe(
          const GLfloat _outerRadius = 1,
          const GLfloat _innerRadius = .9,
          const GLfloat _length = 1,
          const size_t _segments = 16
      );

      /// Create a cylinder capped by one or two spheres. Centered at the origin
      /// and oriented along the z-axis.
      /// @param _radius The cylinder radius.
      /// @param _firstSphereRadius The radius for the first sphere, which
      ///                           should be greater than or equal to _radius.
      ///                           Alternatively it can be 0 to cap the cylinder.
      /// @param _secondSphereRadius The radius for the second sphere, which
      ///                           should be greater than or equal to _radius.
      ///                           Alternatively it can be 0 to cap the cylinder.
      /// @param _length The length between sphere centerse, perpendicular to
      ///                the radius. Must be greater than the two sphere radii.
      /// @param _segments The number of segments to use for the side wall.
      /// @return A triangulated capped cylinder model.
      static
      triangulated_model
      make_capped_cylinder(
          const GLfloat _radius = 1,
          const GLfloat _firstSphereRadius = 2,
          const GLfloat _secondSphereRadius = 2,
          const GLfloat _length = 8,
          const size_t _segments = 16
      );

      ///@}

  };

}

#endif
