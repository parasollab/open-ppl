#include <iostream>
#include <QApplication>

#include "gui/main_window.h"
#include "example_visualization.h"


////////////////////////////////////////////////////////////////////////////////
/// A short example illustrating how to create and use the sandbox.
///
/// Step 1: Subclass base_visualization (see example_visualization).
/// Step 2: Describe your objects by subclassing drawable (see
///         example_visualization).
/// Step 3: Profit.
////////////////////////////////////////////////////////////////////////////////
int
main(int _argc, char* _argv[])
{
  // Create application and main window.
  QApplication app(_argc, _argv);
  main_window window;

  example_visualization sim;
  window.visualization(&sim);

  // Show window and execute app.
  window.show();
  app.exec();

  return 0;
}
