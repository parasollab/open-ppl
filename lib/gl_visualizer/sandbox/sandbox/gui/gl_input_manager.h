#ifndef GL_INPUT_MANAGER_H_
#define GL_INPUT_MANAGER_H_

#include <QtGui>
#include <QWidget>
#include <QObject>


class gl_widget;


////////////////////////////////////////////////////////////////////////////////
/// Manages user input from the keyboard, mouse, or tablet on behalf of the
/// gl_widget. Its purpose is to translate device inputs (like QMouseEvent) into
/// logical inputs (signals/slots like pan_camera).
////////////////////////////////////////////////////////////////////////////////
class gl_input_manager: public QWidget
{

  Q_OBJECT

  protected:

    ///@name Local Types
    ///@{

    struct button_state final
    {

      QPoint click;   ///< The last clicked point.
      size_t drag{0}; ///< Manhattan length of current click-and-drag.

      void reset() noexcept; ///< Reset the state to its default.

    };

    ///@}
    ///@name Internal State
    ///@{

    gl_widget* const m_gl;     ///< The owning gl_widget.

    QPoint m_hover;            ///< The current mouse position.
    button_state m_left;       ///< The left-button state.
    button_state m_middle;     ///< The left-button state.
    button_state m_right;      ///< The left-button state.

    size_t m_deadzone{5};      ///< Ignore drags shorter than this many pixels.
    float m_sensitivity{.1};   ///< Higher values make the mouse more sensitive.

    ///@}

  public:

    ///@name Construction
    ///@{

    gl_input_manager(gl_widget* const _parent);

    virtual ~gl_input_manager();

    ///@}
    ///@name Input Event Handlers
    ///@{
    /// Process user input events on behalf of the gl_widget.

    virtual void mousePressEvent      (QMouseEvent*) override;
    virtual void mouseReleaseEvent    (QMouseEvent*) override;
    virtual void mouseMoveEvent       (QMouseEvent*) override;
    virtual void mouseDoubleClickEvent(QMouseEvent*) override;
    virtual void wheelEvent           (QWheelEvent*) override;
    virtual void keyPressEvent        (QKeyEvent*) override;
    virtual void keyReleaseEvent      (QKeyEvent*) override;
    virtual void tabletEvent          (QTabletEvent*) override;

    ///@}

  signals:

    ///@name Event Signals
    ///@{
    /// Broadcast signals to share processed input data with other widgets.

    void mouse_selection(QPoint);         ///< A point mouse selection.
    void mouse_selection(QPoint, QPoint); ///< An area mouse selection.
    void mouse_hover(QPoint);             ///< A point mouse hover.
    void mouse_hover(QPoint, QPoint);     ///< An area mouse hover.

    void pan_camera(double, double);      ///< Pan the camera.
    void rotate_camera(double, double);   ///< Look around.
    void orbit_camera(double, double);    ///< Orbit the origin.
    void zoom_camera(double);             ///< Zoom in and out.

    ///@}

  protected:

    ///@name Mouse Actors
    ///@{
    /// Respond to specific mouse events.

    /// Respond to a left click/unclick.
    /// @param[in] _press True if clicked, false if released.
    void click_left(QMouseEvent* _e, const bool _press);

    /// Respond to a middle click/unclick.
    /// @param[in] _press True if clicked, false if released.
    void click_middle(QMouseEvent* _e, const bool _press);

    /// Respond to a right click/unclick.
    /// @param[in] _press True if clicked, false if released.
    void click_right(QMouseEvent* _e, const bool _press);

    /// Respond to a left click-and-drag.
    /// @param[in] _e The related mouse event.
    /// @param[in] _delta The change from the previous mouse location.
    void drag_left(QMouseEvent* _e, const QPoint& _delta);

    /// Respond to a middle click-and-drag.
    /// @param[in] _e The related mouse event.
    /// @param[in] _delta The change from the previous mouse location.
    void drag_middle(QMouseEvent* _e, const QPoint& _delta);

    /// Respond to a right click-and-drag.
    /// @param[in] _e The related mouse event.
    /// @param[in] _delta The change from the previous mouse location.
    void drag_right(QMouseEvent* _e, const QPoint& _delta);

    ///@}
};

#endif
