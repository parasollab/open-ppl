#include "EditWidgets.h"

#include "Geometry/Bodies/DHParameters.h"
#include "Visualization/Gui/SliderTextWidget.h"

#include "Transformation.h"
using namespace mathtool;


/*~~~~~~~~~~~~~~~~~~~~~~~~~ EditTransformationWidget ~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

EditTransformationWidget::
EditTransformationWidget(QWidget* const _parent, const std::string& _label,
    const Transformation& _t)
  : QGroupBox(("Transformation: " + _label).c_str(), _parent)
{
  // Create slider text widgets for each transform value.
  /// @TODO How to set XYZ limits? Probably pass a boundary or vector<Range>?
  ///       Hard-coding for now to save time.
  m_sliders = {new SliderTextWidget(this, "X", -100, 100),
               new SliderTextWidget(this, "Y", -100, 100),
               new SliderTextWidget(this, "Z", -100, 100),
               new SliderTextWidget(this, "Alpha", -PI, PI),
               new SliderTextWidget(this, "Beta",  -PI, PI),
               new SliderTextWidget(this, "Gamma", -PI, PI)};

  // Create a layout for the slider text widgets.
  QVBoxLayout* layout = new QVBoxLayout(this);
  for(size_t i = 0; i < m_sliders.size(); ++i) {
    layout->addWidget(m_sliders[i]);

    // Re-emit a value changed signal whenever one of the sliders sends one.
    connect(m_sliders[i], SIGNAL(ValueChanged(double)),
            this, SIGNAL(ValueChanged()));
  }
  setLayout(layout);

  // Set the initial values.
  SetValue(_t);
}

/*--------------------------------- Helpers ----------------------------------*/

Transformation
EditTransformationWidget::
GetValue() {
  return Transformation(Vector3d(m_sliders[0]->GetValue(),      // x
                                 m_sliders[1]->GetValue(),      // y
                                 m_sliders[2]->GetValue()),     // z
                        EulerAngle(m_sliders[3]->GetValue(),    // alpha
                                   m_sliders[4]->GetValue(),    // beta
                                   m_sliders[5]->GetValue()));  // gamma
}


void
EditTransformationWidget::
SetValue(const Transformation& _t) {
  const auto& translation = _t.translation();
  EulerAngle orientation;
  convertFromMatrix(orientation, _t.rotation().matrix());

  m_sliders[0]->SetValue(translation[0]);   // x
  m_sliders[1]->SetValue(translation[1]);   // y
  m_sliders[2]->SetValue(translation[2]);   // z
  m_sliders[3]->SetValue(orientation.alpha());
  m_sliders[4]->SetValue(orientation.beta());
  m_sliders[5]->SetValue(orientation.gamma());
}

/*----------------------------------------------------------------------------*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~ EditDHParametersWidget ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

EditDHParametersWidget::
EditDHParametersWidget(QWidget* const _parent, const std::string& _label,
    const DHParameters& _dh)
  : QGroupBox(("DH Params: " + _label).c_str(), _parent)
{
  // Create slider text widgets for each transform value.
  /// @TODO How to set a/d limits? Hard-coding for now to save time. Probably we
  ///       will not need more than this unless a model is very large.
  m_sliders = {new SliderTextWidget(this, "Alpha", -PI, PI),
               new SliderTextWidget(this, "a", -100, 100),
               new SliderTextWidget(this, "d", -100, 100),
               new SliderTextWidget(this, "Theta", -PI, PI)};

  // Create a layout for the slider text widgets.
  QVBoxLayout* layout = new QVBoxLayout(this);
  for(size_t i = 0; i < m_sliders.size(); ++i) {
    layout->addWidget(m_sliders[i]);

    // Re-emit a value changed signal whenever one of the sliders sends one.
    connect(m_sliders[i], SIGNAL(ValueChanged(double)),
            this, SIGNAL(ValueChanged()));
  }
  setLayout(layout);

  // Set the initial values.
  SetValue(_dh);
}

/*--------------------------------- Helpers ----------------------------------*/

DHParameters
EditDHParametersWidget::
GetValue() {
  return DHParameters(m_sliders[0]->GetValue(),  // alpha
                      m_sliders[1]->GetValue(),  // a
                      m_sliders[2]->GetValue(),  // d
                      m_sliders[3]->GetValue()); // theta
}


void
EditDHParametersWidget::
SetValue(const DHParameters& _dh) {
  m_sliders[0]->SetValue(_dh.m_alpha);
  m_sliders[1]->SetValue(_dh.m_a);
  m_sliders[2]->SetValue(_dh.m_d);
  m_sliders[3]->SetValue(_dh.m_theta);
}

/*----------------------------------------------------------------------------*/
