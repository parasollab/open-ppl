#include "main_window.h"

#include "gl_widget.h"
#include <QMessageBox>
#include <QDockWidget>
#include <QToolBar>
#include <QStatusBar>

#include <iostream>


/*-------------------------------- Construction ------------------------------*/

main_window::
main_window()
  : QMainWindow(nullptr)
{
  initialize_layout();
  initialize_key_maps();
  connect_components();

  if(!main_window::get())
    main_window::get() = this;
}


main_window::
~main_window()
{
  if(main_window::get() == this)
    main_window::get() = nullptr;
}

/*------------------------------- Accessors ----------------------------------*/

static main_window* main_window_singleton = nullptr;


main_window*&
main_window::
get() noexcept
{
  return main_window_singleton;
}


gl_widget*
main_window::
gl() const
{
  return m_gl_widget;
}

/*--------------------------- Visualization Interface ------------------------*/

void
main_window::
visualization(base_visualization* const _v)
{
  m_gl_widget->visualization(_v);
}


base_visualization*
main_window::
visualization()
{
  return m_gl_widget->visualization();
}

/*------------------------------- Key Mappings -------------------------------*/

void
main_window::
add_key_mapping(const std::string& _label, key_map&& _map)
{
  std::lock_guard<std::mutex> lock(m_key_maps_guard);

  if(m_key_maps.count(_label))
    std::cerr << "main_window:: Warning, key map '" << _label
              << "' already exists and will not be overwritten."
              << std::endl;
  else
    m_key_maps[_label] = std::move(_map);
}


void
main_window::
delete_key_mapping(const std::string& _label)
{
  std::lock_guard<std::mutex> lock(m_key_maps_guard);

  auto iter = m_key_maps.find(_label);
  if(iter != m_key_maps.end())
    m_key_maps.erase(iter);
}

/*---------------------------------- Alerts ----------------------------------*/

void
main_window::
show_alert(std::string _s)
{
  QMessageBox m(this);
  m.setText(QString(_s.c_str()));
  m.exec();
}

/*---------------------------- Window Controls -------------------------------*/

void
main_window::
show_window(QWidget* const _w)
{
  int i = m_stack->indexOf(_w);
  show_window(i);
}


void
main_window::
show_window(const int _i)
{
  if(_i < 0 or _i >= m_stack->count())
  {
    std::cerr << "main_window::show_window error: requested display of non-"
              << "existent stack widget " << _i << "."
              << std::endl;
    return;
  }
  m_stack->setCurrentIndex(_i);
}


void
main_window::
add_window(QWidget* const _w)
{
  int i = m_stack->indexOf(_w);
  if(!m_stack->widget(i))
    m_stack->addWidget(_w);
  else
    std::cerr << "main_window::add_window error: requested re-addition of a "
              << "widget that is already on the stack." << std::endl;
}


void
main_window::
delete_window(QWidget* const _w)
{
  int i = m_stack->indexOf(_w);
  if(m_stack->widget(i)) {
    m_stack->removeWidget(_w);
    delete _w;
  }
  else
    std::cerr << "main_window::delete_window error: requested deletion of a "
              << "widget that isn't on the stack." << std::endl;
}

/*------------------------------- Dialog Dock --------------------------------*/

void
main_window::
show_dialog(QDialog* const _dialog) {
  // Connect the dialog to the close_dialog slot so that it removes itself from
  // the dock when complete.
  connect(_dialog, SIGNAL(finished(int)),
          this,      SLOT(close_dialog()));

  // Set the widget flag to make _dialog delete itself when complete.
  _dialog->setAttribute(Qt::WA_DeleteOnClose, true);

  // Add this dialog to the tabs if not already there.
  QTabWidget* tabs = dialog_tabs();
  const int index = tabs->indexOf(_dialog);
  if(index == -1)
    tabs->addTab(_dialog, _dialog->windowTitle());

  // Set this as the current dialog and show it.
  tabs->setCurrentWidget(_dialog);
  show_dialog_dock();
  _dialog->show();
}


void
main_window::
close_dialog() {
  // Get the tab index of the connected dialog.
  QDialog* dialog = static_cast<QDialog*>(sender());
  QTabWidget* tabs = dialog_tabs();
  const int index = tabs->indexOf(dialog);

  // If the index is valid, remove the dialog from the tabs.
  if(index != -1)
  {
    tabs->removeTab(index);
    // If there are no more dialogs, hide the dock.
    if(tabs->count() == 0)
      hide_dialog_dock();
  }
}


void
main_window::
show_dialog_dock() {
  m_dialog_dock->show();
}


void
main_window::
hide_dialog_dock() {
  m_dialog_dock->hide();
}


void
main_window::
reset_dialog_dock() {
  // Close all tabs.
  QTabWidget* tabs = dialog_tabs();
  while(tabs->count() > 0)
    tabs->currentWidget()->close();

  hide_dialog_dock();
}


QTabWidget*
main_window::
dialog_tabs() {
  return static_cast<QTabWidget*>(m_dialog_dock->widget());
}

/*---------------------------------- Setup -----------------------------------*/

void
main_window::
initialize_layout()
{
  // Set window properties.
  setMinimumSize(800, 600);
  setWindowTitle("OpenGL Sandbox");

  initialize_window_stack();
  initialize_tool_bar();
  initialize_status_bar();
  initialize_dialog_dock();
  initialize_gl_widget();
}


void
main_window::
initialize_window_stack()
{
  // Create stack layout.
  m_stack = new QStackedWidget(this);
  this->setCentralWidget(m_stack);
}


void
main_window::
initialize_tool_bar()
{
  // Create the tool bar.
  m_tool_bar = new QToolBar(this);

  // Define buttons.
  m_tool_bar->addAction("Play", this, SIGNAL(start()));
  m_tool_bar->addAction("Reset", this, SIGNAL(reset()));

  // Make toolbar a permanent feature of the main window.
  m_tool_bar->setContextMenuPolicy(Qt::PreventContextMenu);
  m_tool_bar->setFloatable(false);
  m_tool_bar->setMovable(false);
  m_tool_bar->setToolButtonStyle(Qt::ToolButtonTextOnly);
  m_tool_bar->adjustSize();
}


void
main_window::
initialize_status_bar()
{
  // Create the status bar.
  m_status_bar = new QStatusBar(this);
  m_status_bar->setSizeGripEnabled(false);
  m_status_bar->showMessage("Ready.");
  this->setStatusBar(m_status_bar);
}


void
main_window::
initialize_dialog_dock()
{
  // The size policies use minimum horizontal and maximum vertical space.

  // Create the dialog dock.
  m_dialog_dock = new QDockWidget(this);
  m_dialog_dock->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
  m_dialog_dock->setVisible(false);
  addDockWidget(Qt::RightDockWidgetArea, m_dialog_dock);

  // Create a tabbed widget within the dock.
  QTabWidget* dialogTab = new QTabWidget(m_dialog_dock);
  dialogTab->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
  m_dialog_dock->setWidget(dialogTab);
}


void
main_window::
initialize_gl_widget()
{
  // Create the gl window.
  m_gl_widget = new gl_widget(this);
}


void
main_window::
initialize_key_maps()
{
  // Create the default key map.
  key_map default_map(
      [this](QKeyEvent* const _e)
      {
        switch(_e->key())
        {
          // The F1-F12 keys switch between the various windows.
          case Qt::Key_F1:  this->show_window(0);  break;
          case Qt::Key_F2:  this->show_window(1);  break;
          case Qt::Key_F3:  this->show_window(2);  break;
          case Qt::Key_F4:  this->show_window(3);  break;
          case Qt::Key_F5:  this->show_window(4);  break;
          case Qt::Key_F6:  this->show_window(5);  break;
          case Qt::Key_F7:  this->show_window(6);  break;
          case Qt::Key_F8:  this->show_window(7);  break;
          case Qt::Key_F9:  this->show_window(8);  break;
          case Qt::Key_F10: this->show_window(9);  break;
          case Qt::Key_F11: this->show_window(10); break;
          case Qt::Key_F12: this->show_window(11); break;
        }
      }
  );

  add_key_mapping("default", std::move(default_map));
}


void
main_window::
connect_components()
{
  // Add the gl widget to the window stack.
  m_stack->addWidget(m_gl_widget);

  // Connect the tool bar to the gl widget.
  connect(this,      SIGNAL(start()),
          m_gl_widget, SLOT(start()));
  connect(this,      SIGNAL(reset()),
          m_gl_widget, SLOT(reset()));

  // Connect the gl widget to the status bar.
  connect(m_gl_widget, SIGNAL(status_message(const QString&, int)),
          m_status_bar,  SLOT(showMessage(const QString&, int)));
}

/*------------------------------ Input Handling ------------------------------*/

void
main_window::
keyPressEvent(QKeyEvent* _e)
{
  // Acquire the keymap lock.
  std::lock_guard<std::mutex> lock(m_key_maps_guard);

  // Execute each keymap.
  for(auto& labelMap : m_key_maps)
  {
    auto& map_function = labelMap.second;
    map_function(_e);
  }
}

/*----------------------------------------------------------------------------*/
