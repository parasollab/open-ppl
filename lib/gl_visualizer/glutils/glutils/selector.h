#ifndef GLUTILS_SELECTOR_H_
#define GLUTILS_SELECTOR_H_

#include <cstddef>
#include <set>
#include <unordered_map>
#include <vector>

#include "glutils/color.h"
#include "glutils/gltraits.h"


namespace glutils {

  class drawable;

  //////////////////////////////////////////////////////////////////////////////
  /// A selection helper for drawables. Both GL selection and color picking are
  /// supported.
  //////////////////////////////////////////////////////////////////////////////
  class selector final
  {

    ///@name Local Types
    ///@{

    /// A mapping from selection id to drawable.
    typedef std::unordered_map<GLuint, drawable*> selection_map;

    /// A mapping from picking color to drawable.
    typedef std::unordered_map<color, drawable*> color_map;

    /// A list of selected ids.
    typedef std::set<drawable*> hit_list;

    ///@}
    ///@name Internal State
    ///@{

    selection_map m_selection_map; ///< The selection map.
    color_map m_color_map;         ///< The color-picking map.

    hit_list m_hits;           ///< The id's that were selected on last check.
    hit_list m_unhits;         ///< The id's that were deselected by last check.

    bool m_debug{false};       ///< Show debugging messages?

    // For GL selection.
    static constexpr size_t m_buffer_size{1024}; ///< The size of the hit buffer.
    GLuint m_buffer[m_buffer_size];              ///< This selector's hit buffer.

    ///@}

    public:

      ///@}
      ///@name Map Management
      ///@{

      /// Add a drawable to the selection map.
      /// @param _d The drawable to add.
      void add_drawable(drawable* _d);

      /// Remove a drawable from the selection map.
      /// @param _d The drawable to remove.
      void remove_drawable(drawable* _d);

      ///@}
      ///@name Selection
      ///@{

      /// Get the models that were selected on last attempt.
      const hit_list& hits() const noexcept;

      /// Get the models that were unselected on last attempt.
      const hit_list& unhits() const noexcept;

      /// Perform GL selection, marking the selected drawables as such. This
      /// must be called from the rendering thread.
      /// @param _x The X window coordinate for the pick box center. This uses
      ///           the GL convention, where X = 0 is the left of the screen.
      /// @param _y The Y window coordinate for the pick box center. This uses
      ///           the GL convention, where Y = 0 is the bottom of the screen.
      /// @param _w The pick box width.
      /// @param _h The pick box height.
      void select(const size_t _x, const size_t _y, const size_t _w = 5,
          const size_t _h = 5);

      /// Perform color picking, marking the selected drawable as such. This
      /// must be called from the rendering thread. This selection technique
      /// only supports point selection.
      /// @param _x The X window coordinate for the pick box center. This uses
      ///           the GL convention, where X = 0 is the left of the screen.
      /// @param _y The Y window coordinate for the pick box center. This uses
      ///           the GL convention, where Y = 0 is the bottom of the screen.
      void color_pick(const size_t _x, const size_t _y);

      ///@}
      ///@name Debug
      ///@{

      void debug(const bool _show); ///< Enable or disable debugging messages.

      ///@}

    private:

      ///@name Helpers
      ///@{

      /// Setup the GL context for selection.
      void setup_gl_context(const size_t _x, const size_t _y, const size_t _w,
          const size_t _h);

      /// Restore the GL context to its original state.
      void restore_gl_context();

      /// Parse the GL selection hit buffer and mark selected items.
      /// @param _num_hits The number of items in the hit buffer.
      /// @param _all Select all hits or just the closest?
      void parse_selection_buffer(const GLint _num_hits, const bool _all);

      /// Update the hit/unhit lists after a selection call.
      /// @param _hits The objects selected on this call.
      void update_hits(hit_list&& _hits);

      ///@}

  };

}

#endif
