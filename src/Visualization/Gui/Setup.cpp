#include "Setup.h"

#include "Visualization/DrawableMultiBody.h"
#include "Visualization/Gui/EditMultiBodyDialog.h"
#include "Visualization/Gui/TestWidget.h"

#include "sandbox/base_visualization.h"
#include "sandbox/gui/main_window.h"


void
SetupMainWindow(main_window* const _main) {
  // Add a test widget in the F2 slot for GUI development.
  TestWidget* testWidget = new TestWidget(_main);
  _main->add_window(testWidget);

  // Add a keymap to trigger our GL gui components.
  main_window::key_map keyMap(
      [_main](QKeyEvent* const _e)
      {
        switch(_e->key())
        {
          case Qt::Key_E: // Create an edit multibody dialog.
          {
            // Locate the currently selected multibody and ensure there is only
            // one.
            DrawableMultiBody* selected = nullptr;
            auto all = _main->visualization()->selected_drawables();
            for(auto* d : all) {
              DrawableMultiBody* test = dynamic_cast<DrawableMultiBody*>(d);
              if(test) {
                if(selected) {
                  std::cerr << "Cannot start edit dialog because multiple multi"
                            << "bodies are selected."
                            << std::endl;
                  return;
                }
                selected = test;
              }
            }

            // If no multibody was selected, refuse to start the dialog.
            if(!selected) {
              std::cerr << "Cannot start edit dialog because no multibodies "
                        << "are selected."
                        << std::endl;
              return;
            }

            // Start the dialog.
            EditMultiBodyDialog* e = new EditMultiBodyDialog(_main, selected);
            _main->show_dialog(e);
          }
        }


      }
  );
  _main->add_key_mapping("sim-gui", std::move(keyMap));
}
