#include "EditCfgDialog.h"

#include "SliderTextWidget.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Visualization/DrawableMultiBody.h"

#include "sandbox/gui/main_window.h"



/*------------------------------- Construction -------------------------------*/

EditCfgDialog::
EditCfgDialog(main_window* const _parent, DrawableMultiBody* const _mb)
  : QDialog(_parent), m_main(_parent), m_drawable(_mb)
{
  setWindowTitle("Edit Cfg");
  setAttribute(Qt::WA_DeleteOnClose, true);
  setMinimumWidth(400);

  // Create a layout for the widget.
  QVBoxLayout* layout = new QVBoxLayout(this);
  setLayout(layout);

  // Create a slider for each dof.
  for(const auto& info : m_drawable->GetMultiBody()->GetDofInfo()) {
    SliderTextWidget* slider = new SliderTextWidget(this, info.name, info.range);

    // Add to the layout and connect the signal.
    layout->addWidget(slider);
    connect(slider, SIGNAL(ValueChanged(double)),
            this,     SLOT(UpdateCfg()));

    m_sliders.push_back(slider);
  }
}


EditCfgDialog::
~EditCfgDialog() = default;

/*--------------------------------- Helpers ----------------------------------*/

void
EditCfgDialog::
UpdateCfg() {
  auto mb = m_drawable->GetMultiBody();
  std::vector<double> cfg(mb->DOF(), 0);

  for(size_t i = 0; i < cfg.size(); ++i)
    cfg[i] = m_sliders[i]->GetValue();

  mb->Configure(cfg);
}

/*----------------------------------------------------------------------------*/
