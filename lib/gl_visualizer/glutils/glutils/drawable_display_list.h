#ifndef GLUTILS_DRAWABLE_DISPLAY_LIST_H_
#define GLUTILS_DRAWABLE_DISPLAY_LIST_H_

#include <memory>

#include "glutils/drawable.h"
#include "glutils/gltraits.h"

namespace glutils {

  class display_list_set;

  //////////////////////////////////////////////////////////////////////////////
  /// A drawable object that can be represented with a display list.
  //////////////////////////////////////////////////////////////////////////////
  class drawable_display_list :
      public drawable
  {

    ///@name Local Types
    ///@{

    typedef std::shared_ptr<display_list_set> display_list_ptr;

    ///@}
    ///@name Internal State
    ///@{

    display_list_ptr m_lists; ///< Shared pointer to a set of display lists.

    ///@}

    public:

      ///@name Construction
      ///@{

      /// Create a new drawable with a new display list.
      drawable_display_list();

      /// Create a drawable from an existing display list.
      /// @param _cl A shared pointer to an existing display list set, which
      ///            will be used in place of generating a new display list.
      drawable_display_list(display_list_ptr _cl) noexcept;

      virtual ~drawable_display_list();

      virtual void initialize() override;

      virtual void uninitialize() override;

      ///@}

    protected:

      ///@name Call List Specification
      ///@{
      /// These functions should be defined by subclasses to specify the
      /// appropriate rendering instructions.

      /// Instructions for drawing this object at the origin, from the standard
      /// OpenGL perspective.
      virtual void build() = 0;

      /// As build, but purely geometry only (no colors, textures, etc).
      virtual void build_select() = 0;

      /// Additional instructions for drawing the selection decorations on the
      /// object. These will be drawn over the object when it is selected.
      virtual void build_selected() {}

      /// Additional instructions for drawing the highlight decorations on the
      /// object. These will be drawn over the object when it is highlighted,
      /// after any decorations for selection.
      virtual void build_highlighted() {}

    private:

      ///@}
      ///@name drawable Overrides
      ///@{

      virtual void draw() override final;
      virtual void draw_select() override final;
      virtual void draw_selected() override final;
      virtual void draw_highlighted() override final;

      ///@}
  };

}

#endif
