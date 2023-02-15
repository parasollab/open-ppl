#ifndef MAIN_WINDOW_H_
#define MAIN_WINDOW_H_

#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <QtGui>
#include <QMainWindow>
#include <QStackedWidget>

class base_visualization;
class gl_widget;


////////////////////////////////////////////////////////////////////////////////
/// The main application window. Owns and manages all the application's widgets.
///
/// Included features:
/// - Status bar
/// - Multiple windows (switch with F1-F12)
/// - Tabbed dialog dock that can display one or more dialogs
////////////////////////////////////////////////////////////////////////////////
class main_window : public QMainWindow
{

  Q_OBJECT

  public:

    ///@name Local Types
    ///@{

    /// A function which maps key events to arbitrary behaviors.
    typedef std::function<void(QKeyEvent* const)> key_map;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    // Layout components
    QStackedWidget* m_stack;       ///< Handles window switching.
    QToolBar*       m_tool_bar;    ///< The tool bar at the top of the window.
    QStatusBar*     m_status_bar;  ///< Status bar at the bottom of the window.
    QDockWidget*    m_dialog_dock; ///< Displays docked dialogs.

    // Built-in windows
    gl_widget*      m_gl_widget;   ///< Manages and displays the OpenGL scene.

    // Key maps
    std::map<std::string, key_map> m_key_maps; ///< A set of key mappings.
    std::mutex m_key_maps_guard;               ///< Lock for changing key maps.

    ///@}

  public:

    ///@name Construction
    ///@{

    main_window();

    main_window(const main_window&) = delete;

    virtual ~main_window();

    ///@}
    ///@name Accessors
    ///@{

    /// Most applications will use just one main window. For those cases, this
    /// accessor provides a single point of access. If not already set, it will
    /// point to the next instance created.
    static main_window*& get() noexcept;

    gl_widget* gl() const;

    ///@}
    ///@name Visualization Interface
    ///@{

    /// Set the visualization.
    /// @param[in] _v The visualization to use.
    void visualization(base_visualization* const _v);

    /// Get the visualization.
    /// @return The current visualization.
    base_visualization* visualization();

    ///@}
    ///@name Key Mappings
    ///@{
    /// Additional key mappings can be added to the main window. Each will be
    /// called on every key press.

    /// Add a key mapping.
    /// @param _label The label for the new mapping.
    void add_key_mapping(const std::string& _label, key_map&& _map);

    /// Remove a key mapping.
    /// @param _label The label of the mapping to remove.
    void delete_key_mapping(const std::string& _label);

    ///@}

  signals:

    ///@name Action Signals
    ///@{

    void start(); ///< Signal the simulation to start.
    void reset(); ///< Signal the simulation to reset.

    ///@}

  public slots:

    ///@name Alerts
    ///@{
    /// Trigger a modal pop-up. The user must acknowledge the alert before
    /// continuing.

    void show_alert(std::string _s);

    ///@}
    ///@name Window Controls
    ///@{
    /// Control the available windows and switching between them.

    /// Display a widget from the window stack.
    /// @param[in] _w The widget to display.
    void show_window(QWidget* const _w);

    /// Display a widget from the window stack by index.
    /// @param[in] _i The widget index.
    void show_window(const int _i);

    /// Add a widget to the window stack. Ownership will be transfered to the
    /// stack.
    /// @param[in] _w The widget to add.
    void add_window(QWidget* const _w);

    /// Delete a widget from the window stack and de-allocate its memory.
    /// @param[in] _w The widget to delete.
    void delete_window(QWidget* const _w);

    ///@}
    ///@name Dialog Dock Controls
    ///@{
    /// Control display of non-modal dialogs in the tabbed dialog dock.

    /// Show a dialog in the dialog dock. The dialog will delete itself after
    /// completion.
    /// @param[in] _dialog The dialog to display in the dock.
    void show_dialog(QDialog* const _dialog);

    /// Close a dialog in the dock. This must be connected to the QDialog which
    /// will be closed.
    void close_dialog();

    /// Show the dialog dock.
    void show_dialog_dock();

    /// Hide the dialog dock.
    void hide_dialog_dock();

    /// Close all dialogs and clear the dock.
    void reset_dialog_dock();

  protected:

    /// Get the dialog tabs.
    QTabWidget* dialog_tabs();

    ///@}
    ///@name Setup
    ///@{
    /// These functions initialize the layout of the main window. They can be
    /// individually overridden to customize the application.

    /// Create the window layout.
    virtual void initialize_layout();

    /// Create the window stack.
    virtual void initialize_window_stack();

    /// Create the tool bar.
    virtual void initialize_tool_bar();

    /// Create the status bar.
    virtual void initialize_status_bar();

    /// Create the dialog dock.
    virtual void initialize_dialog_dock();

    /// Create the gl widget.
    virtual void initialize_gl_widget();

    /// Create the default key map.
    virtual void initialize_key_maps();

    /// Create connections between the various widgets. Must be called after the
    /// other components have been initialized.
    virtual void connect_components();

    ///@}
    ///@name Input Handling
    ///@{

    virtual void keyPressEvent(QKeyEvent* _e) override;

    ///@}

};

#endif
