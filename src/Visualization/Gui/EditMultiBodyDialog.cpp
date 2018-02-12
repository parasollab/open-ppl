#include "EditMultiBodyDialog.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Visualization/DrawableBody.h"
#include "Visualization/DrawableMultiBody.h"
#include "Visualization/Gui/EditBodyDialog.h"
#include "Visualization/Gui/EditConnectionDialog.h"

#include "sandbox/gui/main_window.h"


/*------------------------------- Construction -------------------------------*/

EditMultiBodyDialog::
EditMultiBodyDialog(main_window* const _parent, DrawableMultiBody* const _mb)
  : QDialog(_parent), m_main(_parent), m_drawable(_mb),
    m_multibody(_mb->GetMultiBody())
{
  setWindowTitle("Edit MultiBody");
  setAttribute(Qt::WA_DeleteOnClose, true);
  setMinimumWidth(400);

  // Create a list of bodies to edit.
  m_bodyList = new QListWidget(this);
  for(size_t i = 0; i < m_multibody->GetNumBodies(); ++i) {
    std::stringstream ss;
    ss << i << " (label '" << m_multibody->GetBody(i)->Label() << "')";

    m_bodyList->addItem(ss.str().c_str());
  }

  // Create a list of connections to edit.
  m_connectionList = new QListWidget(this);
  const auto& joints = m_multibody->GetJoints();
  for(size_t i = 0; i < joints.size(); ++i) {
    const Connection* connection = joints[i].get();

    std::stringstream ss;
    ss << i << " (bodies " << connection->GetPreviousBodyIndex() << ":"
       << connection->GetNextBodyIndex()
       << ")";

    m_connectionList->addItem(ss.str().c_str());
  }

  // Create buttons for editing bodies and connections.
  QPushButton* editBodyButton       = new QPushButton("Edit Body", this),
             * editConnectionButton = new QPushButton("Edit Connection", this);

  // Create buttons for OK and Cancel.
  QDialogButtonBox* okCancel = new QDialogButtonBox(this);
  okCancel->setOrientation(Qt::Horizontal);
  okCancel->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);

  // Create a layout for the dialog and add the subcomponents.
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(m_bodyList);
  layout->addWidget(editBodyButton);
  layout->addWidget(m_connectionList);
  layout->addWidget(editConnectionButton);
  layout->addWidget(okCancel);
  setLayout(layout);

  // Connect the buttons to their appropriate functions.
  connect(editBodyButton, SIGNAL(clicked()),
          this,           SLOT(EditBody()));
  connect(editConnectionButton, SIGNAL(clicked()),
          this,                 SLOT(EditConnection()));

  // Connect the ok/cancel buttons.
  connect(okCancel, SIGNAL(accepted()), this, SLOT(accept()));
  connect(okCancel, SIGNAL(rejected()), this, SLOT(reject()));
}

/*--------------------------------- Helpers ----------------------------------*/

void
EditMultiBodyDialog::
EditBody() {
  // Show an error if more or less than one item is selected.
  if(m_bodyList->selectedItems().size() != 1) {
    m_main->show_alert("Please select a single body to edit");
    return;
  }

  // Get the current item row.
  size_t row = m_bodyList->row(m_bodyList->currentItem());

  // Launch an edit body dialog for this body.
  DrawableBody* drawable = m_drawable->GetDrawableBody(row);
  m_main->show_dialog(new EditBodyDialog(m_main, drawable));
}


void
EditMultiBodyDialog::
EditConnection() {
  // Show an error if more or less than one item is selected.
  if(m_connectionList->selectedItems().size() != 1) {
    m_main->show_alert("Please select a single connection to edit");
    return;
  }

  // Get the current item row.
  size_t row = m_connectionList->row(m_connectionList->currentItem());

  // Launch an edit body dialog for this connection.
  m_main->show_dialog(new EditConnectionDialog(m_main, m_drawable, row));
}

/*----------------------------------------------------------------------------*/
