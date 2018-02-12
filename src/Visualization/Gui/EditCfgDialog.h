#ifndef EDIT_CFG_DIALOG_H_
#define EDIT_CFG_DIALOG_H_

#include <QtGui>

class main_window;
class DrawableMultiBody;


////////////////////////////////////////////////////////////////////////////////
/// A dialog for editing multibody configurations.
////////////////////////////////////////////////////////////////////////////////
class EditCfgDialog : public QDialog {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    /// Construct an edit dialog.
    /// @param _parent The owning main window.
    /// @param _mb The multibody to edit.
    EditCfgDialog(main_window* const _parent, DrawableMultiBody* const _mb);

    virtual ~EditCfgDialog();

    ///@}
    ///@name
    ///@{


    ///@}

  protected:

    ///@name
    ///@{

    main_window* const m_main;            ///< The owning main window.
    DrawableMultiBody* const m_multibody; ///< The edit multibody.

    ///@}
};

#endif
