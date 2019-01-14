#ifndef SVM_MODEL_H_
#define SVM_MODEL_H_

#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <functional>

#include "PolynomialKernelGradient.h"
#include "RadialKernelGradient.h"
#include "Geometry/Boundaries/Boundary.h"
#include "MPLibrary/MPBaseObject.h"
#include "Utilities/PMPLExceptions.h"

#include "dlib/svm.h"


/*------------------------------- SVM Model ----------------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// An encapsulated tool kit for a dlib svm_c_trainer. This object creates a
/// predictive model (a.k.a. decision function) from a set of input
/// configurations which are in one of two categories: {1, -1}. It will attempt
/// to estimate a 'class boundary' in c-space which separates the two
/// categories. The model can then compute which side of the estimated boundary
/// any new configurations lie within. A gradient for the decision function is
/// also determined.
///
/// This utility stores its own copies of the training samples. Those copies can
/// be released after training using ClearInput(). A boundary must
/// be provided, and the samples will be normalized with respect to that boundary.
/// This is required because the training algorithm is not guaranteed to
/// terminate for non-normalized data due to a possibly non-positive-definite
/// kernel matrix.
///
/// @usage
/// @code
/// // Training
/// Boundary* samplingBoundary;
/// std::vector<CfgType> freeCfgs; // free configurations within samplingBoundary
/// std::vector<CfgType> obstCfgs; // obst configurations within samplingBoundary
/// auto m = SVMModel<MPTraits>(this->GetMPLibrary());
/// m.SetBoundary(samplingBoundary->Clone());
/// m.LoadData(freeCfgs, obstCfgs);
/// m.Train();
///
/// // Usage
/// CfgType testCfg; // a cfg within samplingBoundary
/// double svmScore = m.Score(testCfg);
/// bool predictedValidity = m.Classify(testCfg);
/// @endcode
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class SVMModel final : public MPBaseObject<MPTraits> {

  private:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPLibrary    MPLibrary;
    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name SVM Types
    ///@{

    /// The supported kernels.
    enum class KernelType {Polynomial, Radial, Sigmoid};

    typedef double                                              LabelType;
    typedef dlib::matrix<double, 0, 1>                          SampleType;
    typedef dlib::any_trainer<SampleType, LabelType>            TrainerType;
    typedef dlib::any_decision_function<SampleType, LabelType>  DecisionType;
    typedef std::function<SampleType(const SampleType&)>        GradientType;

    //typedef dlib::svm_pegasos<dlib::radial_basis_kernel<SampleType>> IncTrainerType;

    ///@}
    ///@name Input State
    ///@{

    std::shared_ptr<Boundary> m_boundary;        ///< The domain boundary.
    std::vector<SampleType>   m_samples;         ///< Storage for samples.
    std::vector<LabelType>    m_labels;          ///< Storage for labels.
    size_t                    m_numPositive{0};  ///< The count of +1 samples.
    size_t                    m_numNegative{0};  ///< The count of -1 samples.

    ///@}
    ///@name Model State
    ///@{

    double              m_accuracy{0};      ///< The model accuracy.
    SampleType          m_separator;        ///< The separator normal in f-space.
    std::vector<size_t> m_positiveSupports; ///< The positive SV indexes.
    std::vector<size_t> m_negativeSupports; ///< The negative SV indexes.
    DecisionType        m_decision;         ///< Output decision of SVM.
    GradientType        m_gradient;         ///< Computes the decision gradient.

    //IncTrainerType m_pegasos;

    ///@}
    ///@name Parameters
    ///@{

    /// The set of parameters used in model training.
    struct ParameterSet {
      KernelType kernel{KernelType::Radial}; ///< Type of kernel to use.
      double c{10.};        ///< Fit: larger->exactness, smaller->generality.
      double epsilon{.001}; ///< Tolerance: larger values permit more error.
      double gamma{1};      ///< Vector op weight in the kernel calculation.
      double coeff{1};      ///< Offset weight in the kernel calculation.
      double degree{1};     ///< Polynomial order in the kernel calculation.
      double maxDegree{12}; ///< Maximum allowed polynomial order.
      double accuracy{.92}; ///< Desired accuracy threshold.
      bool refine{false};   ///< Iteratively refine the (polynomial) kernel?
    };

    ParameterSet m_params;                ///< This model's parameter values.
    static ParameterSet s_defaultParams;  ///< The default parameter values.

    ///@}

  public:

    ///@name Construction
    ///@{

    /// Set default parameter values from an XML node.
    /// @param _node The XML node.
    static void SetDefaultParameters(XMLNode& _node);

    /// Construct an SVM model with the default parameters.
    /// @param _library The MP library.
    SVMModel(MPLibrary* const _library);

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Training Interface
    ///@{

    void Reset();                  ///< Reset to initial state.
    void ClearInput();             ///< Clear all input data.
    void ClearDecision();          ///< Clear computed decision.
    void ResetParameters();        ///< Reset parameters to their default values.
    ParameterSet& GetParameters(); ///< Get the parameter set.

    /// Load two sets of samples, one valid and the other invalid, into the
    /// model. These are assumed to be located within m_boundary. Samples can be
    /// CfgType, VID, or pair<VID, double>.
    /// @param _free The input set of valid samples.
    /// @param _obst The input set of invalid samples.
    template <typename ContainerType>
    void LoadData(const ContainerType& _free, const ContainerType& _obst);

    /// Set the domain boundary for this model.
    /// @param _b The domain boundary. Input samples will be scaled to the
    ///           boundary so that all values fall within [-1, 1].
    void SetBoundary(std::shared_ptr<Boundary> _b);

    /// Train an SVM model on the input data using a polynomial kernel.
    /// @throws PMPLException Fewer than three free or obstacle configurations
    ///                       provided.
    void Train();

    //void IncTrain(const CfgType& _cfg, const bool _valid);
    //size_t GetNumPositive() const noexcept {return m_numPositive;}
    //size_t GetNumNegative() const noexcept {return m_numNegative;}

    ///@}
    ///@name Prediction Interface
    ///@{

    /// Score a configuration using the decision function.
    /// @return A double indicating the raw score for this configuration.
    double Score(const CfgType& _cfg) const;

    /// Classify a configuration using the decision function.
    /// @return A bool indicating the predicted class of this configuration.
    bool Classify(const CfgType& _cfg) const;

    /// Compute the gradient at a given configuration.
    /// @return A double indicating the raw score for this sample.
    CfgType Gradient(const CfgType& _cfg) const;

    /// Get the model accuracy.
    double GetAccuracy() const;

    /// Get the indexes of the samples that were support vectors.
    std::pair<std::vector<size_t>, std::vector<size_t>> GetSupportVectorIndexes()
        const;

    ///@}

  private:

    ///@name Training Helpers
    ///@{

    /// Add a sample.
    /// @param _s The configuration to add.
    /// @param _l The validity label (+1 or -1).
    void AddSample(const CfgType& _cfg, const LabelType& _l);
    /// @overload
    void AddSample(const VID& _vid, const LabelType& _l);
    /// @overload
    void AddSample(const std::pair<VID, double>& _pair, const LabelType& _l);

    /// Get a trainer object. Trainers create a prediction model by running a
    /// training algorithm on the input data.
    TrainerType GetTrainer() const;

    /// Initialize the gradient function. Requires that the model is trained.
    void ComputeGradient();

    /// Assess the trained model by checking every input sample for correct
    /// classification.
    /// @return The ratio of correctly classified samples.
    double ComputeAccuracy() const;

    /// Identify the support vectors' indexes within the input container.
    void IdentifySupportVectors();

    /// Helper for identifying support vectors.
    template <typename Kernel>
    void ExtractSupportVectors();

    ///@}
    ///@name Prediction Helpers
    ///@{

    /// Score a sample using the decision function.
    /// @param _s The scaled sample point.
    /// @return A double indicating the raw score for this sample.
    double Score(const SampleType& _s) const;

    /// Classify a sample using the decision function.
    /// @param _s The scaled sample point.
    /// @return A bool indicating the predicted class of this sample.
    bool Classify(const SampleType& _s) const;

    /// Compute the decision gradient in c-space at a given sample point.
    /// @param _s The scaled sample point.
    /// @return The scaled gradient direction in c-space.
    SampleType Gradient(const SampleType& _s) const;

    ///@}
    ///@name Conversion Functions
    ///@{

    /// Convert a CfgType to a scaled SampleType.
    /// @param _cfg The unscaled configuration.
    /// @return The scaled sample point.
    SampleType ToSample(const CfgType& _cfg) const;

    /// Convert a scaled SampleType to a CfgType.
    /// @param _s The scaled sample point.
    /// @return The unscaled configuration.
    CfgType ToCfg(const SampleType& _s) const;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
typename SVMModel<MPTraits>::ParameterSet
SVMModel<MPTraits>::s_defaultParams = SVMModel<MPTraits>::ParameterSet{};


template <typename MPTraits>
void
SVMModel<MPTraits>::
SetDefaultParameters(XMLNode& _node) {
  // Set kernel type.
  std::string kernel = _node.Read("kernel", false, "radial", "Kernel type");
  std::transform(kernel.begin(), kernel.end(), kernel.begin(), ::tolower);
  if(kernel == "polynomial")
    s_defaultParams.kernel = KernelType::Polynomial;
  else if(kernel == "radial")
    s_defaultParams.kernel = KernelType::Radial;
  else if(kernel == "sigmoid")
    s_defaultParams.kernel = KernelType::Sigmoid;
  else
    throw ParseException(WHERE, "Unrecognized kernel type '" + kernel + "'");

  // Set other parameters.
  s_defaultParams.c = _node.Read("c", false, s_defaultParams.c, 0., 10000.,
      "Tightness");
  s_defaultParams.epsilon = _node.Read("epsilon", false, s_defaultParams.epsilon,
      0., 1., "Error tolerance");
  s_defaultParams.gamma = _node.Read("gamma", false, s_defaultParams.gamma,
      0., std::numeric_limits<double>::max(),
      "Kernel vector operation coefficient");
  s_defaultParams.coeff = _node.Read("coeff", false, s_defaultParams.coeff,
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
      "Kernel offset");
  s_defaultParams.degree = _node.Read("degree", false, s_defaultParams.degree,
      1., 500., "Starting kernel order");
  s_defaultParams.maxDegree = _node.Read("maxDegree", false,
      s_defaultParams.maxDegree, s_defaultParams.degree,
      s_defaultParams.degree + 500.,
      "Maximum kernel order");
  s_defaultParams.accuracy = _node.Read("accuracy", false,
      s_defaultParams.accuracy, .5, 1., "Minimum accuracy threshold");
  s_defaultParams.refine = _node.Read("refine", false,
      s_defaultParams.refine, "Use iterative kernel refinement?");

  if(s_defaultParams.refine and kernel != "polynomial")
    throw ParseException(WHERE, "Refinement only supported with polynomial "
        "kernel (" + kernel + " selected).");
}


template <typename MPTraits>
SVMModel<MPTraits>::
SVMModel(MPLibrary* const _library) {
  this->SetName("SVMModel");
  this->SetMPLibrary(_library);
  Reset();
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
SVMModel<MPTraits>::
Print(std::ostream& _os) const {
  std::string kernel;
  switch(m_params.kernel)
  {
    case KernelType::Polynomial:
      kernel = "polynomial";
      break;
    case KernelType::Radial:
      kernel = "radial";
      break;
    case KernelType::Sigmoid:
      kernel = "sigmoid";
      break;
  }

  _os << "SVMModel: "
      << "\n\tSamples: " << m_samples.size()
      << " (" << m_numPositive << "/" << m_numNegative << ")."
      << "\n\tTraining Accuracy: " << m_accuracy
      //<< "\n\t\tSupport vector count: "
      //<< _svm.m_positiveSupports.size() << "(+) + "
      //<< _svm.m_negativeSupports.size() << "(-) = "
      //<< _svm.m_positiveSupports.size() + _svm.m_negativeSupports.size()
      //<< "\n\t\tC-space has dimension " << _svm.m_samples[0].size()
      << "\n\tkernel     = " << kernel
      << "\n\tc          = " << s_defaultParams.c
      << "\n\tepsilon    = " << s_defaultParams.epsilon
      << "\n\tgamma      = " << s_defaultParams.gamma
      << "\n\tcoeff      = " << s_defaultParams.coeff
      << "\n\tdegree     = " << s_defaultParams.degree
      << "\n\tmaxDegree  = " << s_defaultParams.maxDegree
      << "\n\taccuracy   = " << s_defaultParams.accuracy
      << "\n\tvalidating = " << (s_defaultParams.refine ? "yes" : "no")
      << std::endl;
}

/*--------------------------- Training Interface -----------------------------*/

template <typename MPTraits>
void
SVMModel<MPTraits>::
Reset() {
  ClearInput();
  ClearDecision();
  ResetParameters();
  m_boundary.reset();

  //m_pegasos.set_lambda(.0001);
  //m_pegasos.set_kernel(dlib::radial_basis_kernel<SampleType>(m_params.gamma));
  //m_pegasos.set_tolerance(m_params.epsilon);
  //m_pegasos.set_max_num_sv(64);
}


template <typename MPTraits>
void
SVMModel<MPTraits>::
ClearInput() {
  m_samples.clear();
  m_labels.clear();
  m_numPositive = 0;
  m_numNegative = 0;

  //m_pegasos.clear();
}


template <typename MPTraits>
void
SVMModel<MPTraits>::
ClearDecision() {
  // Clear computed decision and map
  m_separator = SampleType();
  m_decision = DecisionType();
  m_accuracy = 0.;
  m_gradient = nullptr;
}


template <typename MPTraits>
void
SVMModel<MPTraits>::
ResetParameters() {
  m_params = s_defaultParams;
}


template <typename MPTraits>
typename SVMModel<MPTraits>::ParameterSet&
SVMModel<MPTraits>::
GetParameters() {
  return m_params;
}


template <typename MPTraits>
template <typename ContainerType>
void
SVMModel<MPTraits>::
LoadData(const ContainerType& _free, const ContainerType& _obst) {
  // We must have a boundary to load data as it must be scaled to the model
  // domain.
  if(!m_boundary.get())
    throw RunTimeException(WHERE, "Non-null boundary is required before adding "
        "data.");

  m_numPositive += _free.size();
  m_numNegative += _obst.size();

  const size_t num = m_numPositive + m_numNegative;
  m_samples.reserve(num);
  m_labels.reserve(num);

  for(const auto& sample : _free)
    AddSample(sample,  1);
  for(const auto& sample : _obst)
    AddSample(sample, -1);
}


template <typename MPTraits>
void
SVMModel<MPTraits>::
SetBoundary(std::shared_ptr<Boundary> _b) {
  // Ensure _b is valid.
  if(!_b.get())
    throw RunTimeException(WHERE, "Null boundary provided!");

  m_boundary = _b;
}


template <typename MPTraits>
void
SVMModel<MPTraits>::
Train() {
  MethodTimer mt(this->GetStatClass(), "SVMModel::Train");

  // Check that there are both free and obstacle cfgs
  if(m_numPositive < 1)
    throw PMPLException("SVMModel Error", WHERE, "No free configurations "
        "provided!");
  if(m_numNegative < 1)
    throw PMPLException("SVMModel Error", WHERE, "No obstacle configurations "
        "provided!");

  // If enough data was given, train the model.
  ClearDecision();
  if(this->m_debug)
    cout << "\tTraining SVMModel (" << m_numPositive << "/" << m_numNegative
         << ")..." << endl;

  // Train model, iteratively increasing kernel complexity until desired
  // accuracy or maximum complexity is reached
  while(true) {

    if(this->m_debug && m_params.kernel == KernelType::Polynomial)
      cout << "\t\tFitting order-" << m_params.degree << " polynomial model...";

    // Set kernel parameters
    TrainerType trainer = GetTrainer();

    // Train model
    m_decision = trainer.train(m_samples, m_labels);

    // Check model to determine whether to continue refining or quit.
    m_accuracy = ComputeAccuracy();
    bool refine = m_params.kernel == KernelType::Polynomial
              and m_params.refine
              and m_params.degree < m_params.maxDegree
              and m_accuracy < m_params.accuracy;

    if(this->m_debug)
      cout << " Accuracy: " << setprecision(4) << m_accuracy << endl;

    // If done, quit. Otherwise, increase kernel order and retrain.
    if(!refine)
      break;
    ++m_params.degree;
  }

  // Training finished, now get decision info and compute the mapping
  //IdentifySupportVectors();
  ComputeGradient();

  // Print debugging information if requested
  if(this->m_debug)
    std::cout << *this;
}


//template <typename MPTraits>
//void
//SVMModel<MPTraits>::
//IncTrain(const CfgType& _cfg, const bool _valid) {
//  MethodTimer mt(this->GetStatClass(), "SVMModel::IncTrain");
//
//  m_pegasos.train(ToSample(_cfg), _valid ? 1 : -1);
//
//  auto& count = _valid ? m_numPositive : m_numNegative;
//  ++count;
//}

/*--------------------------- Prediction Interface ---------------------------*/

template <typename MPTraits>
double
SVMModel<MPTraits>::
Score(const CfgType& _cfg) const {
  return Score(ToSample(_cfg));
}


template <typename MPTraits>
bool
SVMModel<MPTraits>::
Classify(const CfgType& _cfg) const {
  return Classify(ToSample(_cfg));
}


template <typename MPTraits>
typename SVMModel<MPTraits>::CfgType
SVMModel<MPTraits>::
Gradient(const CfgType& _cfg) const {
  return ToCfg(Gradient(ToSample(_cfg)));
}


template <typename MPTraits>
double
SVMModel<MPTraits>::
GetAccuracy() const {
  return m_accuracy;
}


template <typename MPTraits>
std::pair<std::vector<size_t>, std::vector<size_t>>
SVMModel<MPTraits>::
GetSupportVectorIndexes() const {
  /// @TODO The SV computation does not work - see ExtractSupportVectors.
  throw RunTimeException(WHERE, "This function is broken.");
  return std::make_pair(m_positiveSupports, m_negativeSupports);
}

/*------------------------------- Data Helpers -------------------------------*/

template <typename MPTraits>
void
SVMModel<MPTraits>::
AddSample(const CfgType& _cfg, const LabelType& _l) {
  m_samples.push_back(ToSample(_cfg));
  m_labels.push_back(_l);
}


template <typename MPTraits>
void
SVMModel<MPTraits>::
AddSample(const VID& _vid, const LabelType& _l) {
  auto g = (_l == 1) ? this->GetRoadmap():
                       this->GetBlockRoadmap();
  AddSample(g->GetVertex(_vid), _l);
}


template <typename MPTraits>
void
SVMModel<MPTraits>::
AddSample(const std::pair<VID, double>& _pair, const LabelType& _l) {
  AddSample(_pair.first, _l);
}

/*---------------------------- Training Helpers ------------------------------*/

template <typename MPTraits>
typename SVMModel<MPTraits>::TrainerType
SVMModel<MPTraits>::
GetTrainer() const {
  switch(m_params.kernel) {
    case KernelType::Polynomial:
      {
        using Kernel = dlib::polynomial_kernel<SampleType>;
        using Trainer = dlib::svm_c_trainer<Kernel>;
        Trainer t;
        t.set_c(m_params.c);
        t.set_epsilon(m_params.epsilon);
        t.set_kernel(Kernel(m_params.gamma, m_params.coeff, m_params.degree));
        return TrainerType(t);
      }
    case KernelType::Radial:
      {
        using Kernel = dlib::radial_basis_kernel<SampleType>;
        using Trainer = dlib::svm_c_trainer<Kernel>;
        Trainer t;
        t.set_c(m_params.c);
        t.set_epsilon(m_params.epsilon);
        t.set_kernel(Kernel(m_params.gamma));
        return TrainerType(t);
      }
    case KernelType::Sigmoid:
      {
        using Kernel = dlib::sigmoid_kernel<SampleType>;
        using Trainer = dlib::svm_c_trainer<Kernel>;
        Trainer t;
        t.set_c(m_params.c);
        t.set_epsilon(m_params.epsilon);
        t.set_kernel(Kernel(m_params.gamma, m_params.coeff));
        return TrainerType(t);
      }
    default:
      throw PMPLException("SVMModel Error", WHERE, "Unrecognized kernel type.");
      return TrainerType();
  }
}


template <typename MPTraits>
void
SVMModel<MPTraits>::
ComputeGradient() {
  switch(m_params.kernel) {
    case KernelType::Polynomial:
      {
        using Gradient = PolynomialKernelGradient;
        using Kernel   = dlib::polynomial_kernel<SampleType>;
        using Decision = dlib::decision_function<Kernel>;
        const auto& decision       = m_decision.get<Decision>();
        const auto& supportVectors = decision.basis_vectors;
        const auto& weights        = decision.alpha;
        m_gradient = Gradient(supportVectors, weights, m_params.degree,
            m_params.gamma);
        break;
      }
    case KernelType::Radial:
      {
        using Kernel   = dlib::radial_basis_kernel<SampleType>;
        using Decision = dlib::decision_function<Kernel>;
        m_gradient = RadialKernelGradient(m_decision.get<Decision>(),
            m_params.gamma);
        break;
      }
    case KernelType::Sigmoid:
      throw RunTimeException(WHERE, "Not yet implemented.");
    default:
      throw PMPLException("SVMModel Error", WHERE, "Unrecognized kernel type.");
  }
}


template <typename MPTraits>
double
SVMModel<MPTraits>::
ComputeAccuracy() const {
  size_t numCorrect = 0;
  const size_t num = m_numPositive + m_numNegative;

  for(size_t i = 0; i < num; ++i) {
    const bool classify = Classify(m_samples[i]),
               expected = (m_labels[i] == 1);
    if(classify == expected)
      ++numCorrect;
  }

  return static_cast<double>(numCorrect) / static_cast<const double>(num);
}


template <typename MPTraits>
void
SVMModel<MPTraits>::
IdentifySupportVectors() {
  // Get current support vectors.
  switch(m_params.kernel) {
    case KernelType::Polynomial:
      {
        using Kernel = dlib::polynomial_kernel<SampleType>;
        ExtractSupportVectors<Kernel>();
        break;
      }
    case KernelType::Radial:
      {
        using Kernel = dlib::radial_basis_kernel<SampleType>;
        ExtractSupportVectors<Kernel>();
        break;
      }
    case KernelType::Sigmoid:
      {
        using Kernel = dlib::sigmoid_kernel<SampleType>;
        ExtractSupportVectors<Kernel>();
        break;
      }
    default:
      throw PMPLException("SVMModel Error", WHERE, "Unrecognized kernel type.");
  }
}


template <typename MPTraits>
template <typename Kernel>
void
SVMModel<MPTraits>::
ExtractSupportVectors() {
  throw RunTimeException(WHERE, "This function is broken.");

  /// @TODO This implementation is wrong - the labels are not index-aligned with
  ///       the decision weights. We cannot recover the input indexes which
  ///       became support vectors without comparing m_decision's basis_vectors
  ///       against the input set.

#if 0
  // Clear previous supports.
  m_positiveSupports.clear();
  m_negativeSupports.clear();

  // Pick out the current support vectors.
  using Decision = dlib::decision_function<Kernel>;
  const auto& alpha = m_decision.get<Decision>().alpha;
  for(size_t i = 0; i < (size_t)alpha.size(); ++i) {
    if(alpha(i) == 0)
      continue;
    else if(m_labels[i] > 0)
      m_positiveSupports.push_back(i);
    else
      m_negativeSupports.push_back(i);
  }
#endif
}

/*--------------------------- Prediction Helpers -----------------------------*/

template <typename MPTraits>
double
SVMModel<MPTraits>::
Score(const SampleType& _s) const {
  return m_decision(_s);
  //return m_pegasos(_s);
}


template <typename MPTraits>
bool
SVMModel<MPTraits>::
Classify(const SampleType& _s) const {
  return Score(_s) > 0;
}


template <typename MPTraits>
typename SVMModel<MPTraits>::SampleType
SVMModel<MPTraits>::
Gradient(const SampleType& _s) const {
  return m_gradient(_s);
}

/*-------------------------- Conversion Functions ----------------------------*/

template <typename MPTraits>
typename SVMModel<MPTraits>::SampleType
SVMModel<MPTraits>::
ToSample(const CfgType& _cfg) const {
  MethodTimer mt(this->GetStatClass(), "SVMModel::ToSample");

  const size_t i = _cfg.PosDOF();

  // Scale the DOF data to the domain boundary and convert to SampleType.
  std::vector<double> data = _cfg.GetData();

  // Convert any orientation DOFs from euler angles to euler vectors.
  if(_cfg.GetMultiBody()->GetBase()->GetMovementType() ==
      Body::MovementType::Rotational)
  {
    const Vector3d eulerVector = _cfg.GetAngularPosition();

    switch(_cfg.OriDOF()) {
      case 1:
        data[i] = eulerVector[2] / PI;
        break;
      case 3:
        data[i]     = eulerVector[0] / PI;
        data[i + 1] = eulerVector[1] / PI;
        data[i + 2] = eulerVector[2] / PI;
        break;
      default:;
    }
  }

  // Scale w.r.t. boundary limits.
  // For cspace boundaries, scale all of the points.
  if(m_boundary->Type() == Boundary::Space::CSpace)
    m_boundary->ScalePoint(data);
  // For workspace boundaries, scale only the positional points.
  else {
    std::vector<double> position(data.begin(), data.begin() + i);
    m_boundary->ScalePoint(position);
    std::copy(position.begin(), position.end(), data.begin());
  }
  return dlib::mat(data);
}


template <typename MPTraits>
typename SVMModel<MPTraits>::CfgType
SVMModel<MPTraits>::
ToCfg(const SampleType& _s) const {
  MethodTimer mt(this->GetStatClass(), "SVMModel::ToCfg");

  // Copy the data from _s into a std::vector.
  std::vector<double> data(_s.size());
  for(size_t i = 0; i < data.size(); ++i)
    data[i] = _s(i);

  CfgType cfg(this->GetTask()->GetRobot());
  const size_t i = cfg.PosDOF();

  // Unscale the data relative to the domain boundary.
  // For cspace boundaries, unscale all of the points.
  if(m_boundary->Type() == Boundary::Space::CSpace)
    m_boundary->UnscalePoint(data);
  // For workspace boundaries, unscale only the positional points.
  else {
    std::vector<double> position(data.begin(), data.begin() + i);
    m_boundary->UnscalePoint(position);
    std::copy(position.begin(), position.end(), data.begin());
  }

  cfg.SetData(data);

  // Convert any orientation DOFs from euler vectors to euler angles.
  switch(cfg.OriDOF()) {
    case 1:
      {
        const Vector3d eulerVector(0, 0, data[i] * PI);;
        cfg.SetAngularPosition(eulerVector);
        break;
      }
    case 3:
      {
        const Vector3d eulerVector(data[i] * PI,
                                   data[i + 1] * PI,
                                   data[i + 2] * PI);;
        cfg.SetAngularPosition(eulerVector);
        break;
      }
    default:;
  }

  return cfg;
}

/*-------------------------------- Debugging ---------------------------------*/

template <typename MPTraits>
std::ostream&
operator<<(std::ostream& _os, const SVMModel<MPTraits>& _svm) {
  _svm.Print(_os);
  return _os;
}

/*----------------------------------------------------------------------------*/

#endif
