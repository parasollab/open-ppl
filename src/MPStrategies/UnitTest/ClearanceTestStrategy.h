#ifndef CLEARANCE_TEST_STRATEGY_H_
#define CLEARANCE_TEST_STRATEGY_H_

#include "MPStrategies/MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class ClearanceTestStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    ClearanceTestStrategy(
        const map<string, pair<size_t, size_t> >& _samplerLabels =
        map<string, pair<size_t,size_t> >(),
        const vector<ClearanceUtility<MPTraits> >& _utilities =
        vector<ClearanceUtility<MPTraits> >(),
        string _dmLabel = "");
    ClearanceTestStrategy(typename MPTraits::MPProblemType* _problem,
        XMLNode& _node);
    virtual ~ClearanceTestStrategy();

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize(){}
    virtual void Run();
    virtual void Finalize(){}

  private:
    map<string, pair<size_t, size_t> > m_samplerLabels;
    vector<ClearanceUtility<MPTraits> > m_clearanceUtilities;
    string m_dmLabel;
};


template <class MPTraits>
ClearanceTestStrategy<MPTraits>::
ClearanceTestStrategy(const map<string, pair<size_t, size_t> >& _samplerLabels,
    const vector<ClearanceUtility<MPTraits> >& _utilities, string _dmLabel) :
  MPStrategyMethod<MPTraits>(),
  m_samplerLabels(_samplerLabels), m_clearanceUtilities(_utilities),
  m_dmLabel(_dmLabel) {
    this->SetName("ClearanceTest");
  }

template <class MPTraits>
ClearanceTestStrategy<MPTraits>::
ClearanceTestStrategy(typename MPTraits::MPProblemType* _problem,
    XMLNode& _node) : MPStrategyMethod<MPTraits>(_problem, _node) {
  this->SetName("ClearanceTest");
  ParseXML(_node);
}

template <class MPTraits>
ClearanceTestStrategy<MPTraits>::
~ClearanceTestStrategy() {
}

template <class MPTraits>
void
ClearanceTestStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  m_dmLabel = _node.Read("dmLabel", true, "", "distance metric label");

  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      string generationMethod = child.Read("label", true, "",
          "sampler method label");
      size_t numberSamples = child.Read("number", false, 1, 0,
          MAX_INT, "number of samples");
      size_t numberAttempts = child.Read("attempts", false, 100,
          1, MAX_INT, "number of attempts");
      m_samplerLabels[generationMethod] =
        make_pair(numberSamples, numberAttempts);
    }
    else if(child.Name() == "Clearance") {
      m_clearanceUtilities.push_back(
          ClearanceUtility<MPTraits>(this->GetMPProblem(), child));
    }
  }

  if(m_clearanceUtilities.size() < 2)
    throw ParseException(_node.Where(), "Must specify at least two clearance"
        "measurement settings. The first as a baseline and the remaining to"
        "compare to it.");
}

template <class MPTraits>
void
ClearanceTestStrategy<MPTraits>::
Print(ostream& _os) const {
  _os << "ClearanceTestStrategy ::"
    << "\tdmLabel = \"" << m_dmLabel << "\""
    << "\n\tSamplers\n";
  for(auto&  sampler : m_samplerLabels)
    _os << "\t\t\"" << sampler.first << "\", "
      << sampler.second.first << " samples, "
      << sampler.second.second << " attempts\n";
  _os << "\tClearance Measurements\n";
  for(auto&  cu : m_clearanceUtilities) {
    cu.Print(_os);
    _os << endl;
  }
  _os << endl;
}

template <class MPTraits>
void
ClearanceTestStrategy<MPTraits>::
Run() {
  ClockClass clock;
  StatClass *stats = this->GetStatClass();
  stats->StartClock("Clearance Test");

  //generate samples
  vector<CfgType> samples;
  for(auto&  sampler : m_samplerLabels) {
    typename MPTraits::MPProblemType::SamplerPointer s =
      this->GetSampler(sampler.first);
    s->Sample(sampler.second.first, sampler.second.second,
        this->m_boundary, back_inserter(samples));
  }
  if(this->m_debug)
    cout << "\tgenerated " << samples.size() << " samples total" << endl;

  //for each sample, compare baseline clearance and input clearance
  for(auto&  sample : samples) {
    if(this->m_debug)
      cout << "computing clearances for cfg: " << sample << endl;

    //compute baseline clearance
    double baselineClearance = MAX_INT;
    CfgType baselineWitness;
    CDInfo baselineInfo;
    if(m_clearanceUtilities.front().CollisionInfo(
          sample, baselineWitness, this->m_boundary, baselineInfo))
      baselineClearance = baselineInfo.m_minDist;
    else
      cerr << "Error, could not compute base collision information for cfg: "
        << sample << endl;
    if(this->m_debug)
      cout << "\tbaseline = " << baselineClearance
        << "\twitness = " << baselineWitness << endl;
    sample.m_witnessCfg = shared_ptr<Cfg>(); //reset clearance cache

    for(typename vector<ClearanceUtility<MPTraits> >::iterator cit =
        m_clearanceUtilities.begin()+1;
        cit != m_clearanceUtilities.end(); ++cit) {
      //compute input clearance
      double approxClearance = MAX_INT;
      CfgType approxWitness;
      CDInfo approxInfo;
      if(cit->CollisionInfo(sample, approxWitness,
            this->m_boundary, approxInfo))
        approxClearance = approxInfo.m_minDist;
      else
        cerr << "Error, could not compute approx collision information for cfg:"
          << sample << endl;
      if(this->m_debug)
        cout << "\tapprox = " << approxClearance
          << "\twitness = " << approxWitness << endl;
      sample.m_witnessCfg = shared_ptr<Cfg>(); //reset clearance cache

      //calculate difference
      if(this->m_debug)
        cout << "\tdifference = " << approxClearance - baselineClearance
          << "\tdistance = "
          << this->GetDistanceMetric(m_dmLabel)->Distance(
              approxWitness, baselineWitness) << endl;
    }
  }

  stats->StopClock("Clearance Test");
  cout << ":" << stats->GetSeconds("Clearance Test")
    << " sec (ie, " << stats->GetUSeconds("Clearance Test") << " usec)";
  cout << endl;
}

#endif
