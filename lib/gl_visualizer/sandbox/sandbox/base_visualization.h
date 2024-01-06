#ifndef BASE_VISUALIZATION_H_
#define BASE_VISUALIZATION_H_

#include <cstddef>
#include <mutex>
#include <set>
#include <vector>

#include "glutils/drawable.h" // Included for derived classes.

namespace glutils
{
  class selector;
}


////////////////////////////////////////////////////////////////////////////////
/// The base interface for a visualization that runs in the sandbox. It has
/// default storage for drawables, and a default selector/highlighter. The
/// rendering functions can be overriden to add additional instructions, but the
/// base class methods should always be called as well to retain the default
/// functionality.
////////////////////////////////////////////////////////////////////////////////
class base_visualization
{

  protected:

    ///@name Internal State
    ///@{

    std::vector<glutils::drawable*> m_drawables; ///< The drawable objects.

    glutils::selector* m_selector{nullptr};      ///< Selection helper.
    glutils::selector* m_highlighter{nullptr};   ///< Highlight helper.

    std::mutex m_selector_guard;                 ///< Guard for selector.
    std::mutex m_highlighter_guard;              ///< Guard for highlighter.

    ///@}

  public:

    ///@name Construction
    ///@{

    base_visualization();

    virtual ~base_visualization();

    ///@}
    ///@name Visualization Control
    ///@{

    /// Define any instructions to be executed at the start of the visualization.
    virtual void start() {}

    /// Define how to reset the visualization to its initial state.
    virtual void reset() {}

    /// Update the transforms for all drawable objects. This is not done in
    /// render to give more flexibility for multi-threaded visualizations. There
    /// is no internal lock, so derived applications should decide how to handle
    /// contention on the transform data (most likely with an external lock).
    virtual void update();

    ///@}
    ///@name Rendering
    ///@{
    /// Define how to render the simulation. If these functions are overridden,
    /// the base class versions should be called as well to maintain the
    /// functionality of the drawables.

    /// Define any instructions needed to render this visualization.
    virtual void render();

    /// Define any instructions to be executed when a selection takes place.
    /// @param _x The x coordinate of the picking box's center.
    /// @param _y The y coordinate of the picking box's center.
    /// @param _w The picking box width.
    /// @param _h The picking box height.
    virtual void render_select(const size_t _x, const size_t _y, const size_t _w,
        const size_t _h);

    /// Define any instructions to be executed when a hover event takes place.
    /// @param _x The x coordinate of the picking box's center.
    /// @param _y The y coordinate of the picking box's center.
    /// @param _w The picking box width.
    /// @param _h The picking box height.
    virtual void render_hover(const size_t _x, const size_t _y, const size_t _w,
        const size_t _h);

    ///@}
    ///@name Drawables
    ///@{

    /// Add a drawable to the visualization.
    /// @param _d The drawable to add.
    virtual void add_drawable(glutils::drawable* _d);

    /// Remove a drawable from the visualization and delete it.
    /// @param _d The drawable to remove.
    virtual void remove_drawable(glutils::drawable* _d);

    ///@}
    ///@name Selection and Highlighting
    ///@{
    /// Access the currently selected and highlighted drawables. These functions
    /// are thread-safe with the rendering tools.

    /// Get the currently highlighted drawables.
    virtual std::set<glutils::drawable*> highlighted_drawables();

    /// Get the currently selected drawables.
    virtual std::set<glutils::drawable*> selected_drawables();

    ///@}

};

#endif
