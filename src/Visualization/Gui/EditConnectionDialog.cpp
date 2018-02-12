#include "EditConnectionDialog.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Simulator/Simulation.h"
#include "Visualization/DrawableBody.h"
#include "Visualization/DrawableMultiBody.h"
#include "Visualization/Gui/EditWidgets.h"

#include "sandbox/gui/main_window.h"

#include "Transformation.h"

using namespace mathtool;


/*------------------------------- Construction -------------------------------*/

EditConnectionDialog::
EditConnectionDialog(main_window* const _parent, DrawableMultiBody* const _mb,
    const size_t _index)
  : QDialog(_parent), m_main(_parent), m_drawable(_mb),
    m_connection(m_drawable->GetMultiBody()->GetJoint(_index))
{
  setWindowTitle("Edit Connection");
  setAttribute(Qt::WA_DeleteOnClose, true);
  setMinimumWidth(400);

  // Create Widgets to edit the transforms.
  m_transform1Editor = new EditTransformationWidget(this,
      m_connection->GetTransformationToDHFrame());
  m_transform2Editor = new EditTransformationWidget(this,
      m_connection->GetTransformationToBody2());

  // Create a widget to edit the DH params.
  m_dhParamsEditor = new EditDHParametersWidget(this,
      m_connection->GetDHParameters());

  // Create buttons.
  QDialogButtonBox* okCancel = new QDialogButtonBox(this);
  okCancel->setOrientation(Qt::Horizontal);
  okCancel->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);

  // Create a layout for the dialog and add the subcomponents.
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(m_transform1Editor);
  layout->addWidget(m_dhParamsEditor);
  layout->addWidget(m_transform2Editor);
  layout->addWidget(okCancel);
  setLayout(layout);

  // Connect the ok/cancel buttons.
  connect(okCancel, SIGNAL(accepted()), this, SLOT(accept()));
  connect(okCancel, SIGNAL(rejected()), this, SLOT(reject()));

  // Connect the edit tools.
  connect(m_transform1Editor, SIGNAL(ValueChanged()),
          this,               SLOT(UpdateTransformationToDHFrame()));
  connect(m_transform2Editor, SIGNAL(ValueChanged()),
          this,               SLOT(UpdateTransformationToBody2()));
  connect(m_dhParamsEditor,   SIGNAL(ValueChanged()),
          this,               SLOT(UpdateDHParameters()));
}

/*--------------------------------- Helpers ----------------------------------*/

void
EditConnectionDialog::
UpdateTransformationToDHFrame() {
  UpdateTransformation(m_transform1Editor,
                       m_connection->GetTransformationToDHFrame());
}


void
EditConnectionDialog::
UpdateTransformationToBody2() {
  UpdateTransformation(m_transform2Editor,
                       m_connection->GetTransformationToBody2());
}


void
EditConnectionDialog::
UpdateDHParameters() {
  // Lock the simulation while we update the data.
  auto simulation = Simulation::Get();
  simulation->Lock();

  std::cout << "Updating DH params" << std::endl;
  m_connection->GetDHParameters() = m_dhParamsEditor->GetValue();
  m_drawable->rebuild();
  simulation->RebuildMultiBody(m_drawable->GetMultiBody());

  simulation->Unlock();
}


void
EditConnectionDialog::
UpdateTransformation(EditTransformationWidget* const _w, Transformation& _t) {
  // Lock the simulation while we update the data.
  auto simulation = Simulation::Get();
  simulation->Lock();

  std::cout << "Updating Transformation" << std::endl;
  _t = _w->GetValue();
  m_drawable->rebuild();
  simulation->RebuildMultiBody(m_drawable->GetMultiBody());

  simulation->Unlock();
}

/*----------------------------------------------------------------------------*/
