#include "EditCfgDialog.h"

#include "sandbox/gui/main_window.h"



/*------------------------------- Construction -------------------------------*/

EditCfgDialog::
EditCfgDialog(main_window* const _parent, DrawableMultiBody* const _mb)
  : QDialog(_parent), m_main(_parent), m_multibody(_mb)
{
  setWindowTitle("Edit Cfg");
  setAttribute(Qt::WA_DeleteOnClose, true);
  setMinimumWidth(400); /// @TODO Check if this looks good.
}


EditCfgDialog::
~EditCfgDialog() = default;

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
