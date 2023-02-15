#ifndef GL_WIDGET_H_
#define GL_WIDGET_H_

#include <QOpenGLWidget>
#include <QPoint>
#include <QMouseEvent>

class base_visualization;
class gl_input_manager;

namespace glutils {
  class camera;
}


////////////////////////////////////////////////////////////////////////////////
/// Provides an OpenGL scene embeded within a QWidget.
///
/// The scene is rendered automatically at 30fps by a local QTimer object. This
/// widget also allows the mouse to be used to control the viewing camera.
////////////////////////////////////////////////////////////////////////////////
class gl_widget : public QOpenGLWidget
{

  Q_OBJECT

  ///@name Rendering Internals
  ///@{

  QTimer*             m_clock{nullptr};         ///< Rendering clock.
  gl_input_manager*   m_input_manager{nullptr}; ///< Manages user input.
  glutils::camera*    m_camera{nullptr};        ///< The current camera.
  base_visualization* m_visualization{nullptr}; ///< The current visualization.

  ///@}

  public:

    ///@name Construction
    ///@{

    gl_widget(QWidget* _parent);

    virtual ~gl_widget();

    ///@}
    ///@name Accessors
    ///@{

    glutils::camera* camera() const;                  ///< Get the camera.
    base_visualization* visualization() const;        ///< Get the visualization.
    void visualization(base_visualization* const _s); ///< Set the visualization.

    ///@}

  public slots:

    ///@name Simulator Controls
    ///@{

    void start();
    void reset();

    ///@}
    ///@name Camera Controls
    ///@{

    /// Translate the camera without changing the orientation.
    /// @param _x The X translation.
    /// @param _y The Y translation.
    void pan_camera(const double _x, const double _y);

    /// Zoom the camera in the screen-in direction (effectively a -Z
    /// translation).
    /// @param _z The ammount to zoom.
    void zoom_camera(const double _z);

    /// Rotate the camera about its own X and Y axis.
    /// @param _x The X rotation.
    /// @param _y The Y rotation.
    void rotate_camera(const double _x, const double _y);

    /// Orbit the camera about the origin.
    /// @param _x The X rotation.
    /// @param _y The Y rotation.
    void orbit_camera(const double _x, const double _y);

    ///@}
    ///@name Selection
    ///@{

    /// Perform selection rendering at the given point.
    /// @param _x The x-center for the pick box.
    /// @param _y The y-center for the pick box.
    /// @param _w The width of the pick box.
    /// @param _h The height of the pick box.
    void select(const size_t _x, const size_t _y, const size_t _w,
        const size_t _h);

    /// Perform highlight rendering at the given point.
    /// @param _x The x-center for the pick box.
    /// @param _y The y-center for the pick box.
    /// @param _w The width of the pick box.
    /// @param _h The height of the pick box.
    void hover(const size_t _x, const size_t _y, const size_t _w,
        const size_t _h);

    void select(QPoint);         ///< Slot for accepting single-point selection.
    void select(QPoint, QPoint); ///< Slot for accepting click-and-drag selection.

    void hover(QPoint);          ///< Slot for accepting single-point hover.
    void hover(QPoint, QPoint);  ///< Slot for accepting click-and-drag hover.

    ///@}

  signals:

    ///@name Signals
    ///@{

    /// Display a message on the status bar for some amount of time.
    /// @param[in] _m The message to display.
    /// @param[in] _ms The time in milli-seconds to show the message, or 0 for
    ///                indefinite.
    void status_message(const QString& _m, int _ms);

    ///@}

  private:

    ///@name GL Functions
    ///@{

    virtual void initializeGL() override; ///< Initialize the OpenGL scene.
    virtual void paintGL() override;      ///< Draw the OpenGL scene.

    /// Redefine the OpenGL view when the window is resized.
    /// @param[in] _w The new scene width.
    /// @param[in] _h The new scene height.
    virtual void resizeGL(int _w, int _h) override;

    ///@}
    ///@name Input Forwarding
    ///@{
    /// Outsource user input handling to the standard input manager.

    virtual void mousePressEvent(QMouseEvent* _e) override;
    virtual void mouseReleaseEvent(QMouseEvent* _e) override;
    virtual void mouseMoveEvent(QMouseEvent* _e) override;
    virtual void mouseDoubleClickEvent(QMouseEvent* _e) override;
    virtual void wheelEvent(QWheelEvent* _e) override;
    virtual void keyPressEvent(QKeyEvent* _e) override;
    virtual void keyReleaseEvent(QKeyEvent* _e) override;
    virtual void tabletEvent(QTabletEvent* _e) override;

    ///@}

};

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/

#endif
