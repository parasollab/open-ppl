#ifndef CLEARANCETESTSTRATEGY_H_
#define CLEARANCETESTSTRATEGY_H_

#include "MPStrategies/MPStrategyMethod.h"

template <class MPTraits>
class ClearanceTestStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    ClearanceTestStrategy(const map<string, pair<int, int> >& _samplerLabels = (map<string, pair<int,int> >()), const vector<ClearanceUtility<MPTraits> >& _utilities = (vector<ClearanceUtility<MPTraits> >()), string _dmLabel = "");
    ClearanceTestStrategy(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~ClearanceTestStrategy();

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _out) const;

    virtual void Initialize(){}
    virtual void Run();
    virtual void Finalize(){}

  private:
    map<string, pair<int, int> > m_samplerLabels;
    vector<ClearanceUtility<MPTraits> > m_clearanceUtilities;
    string m_dmLabel;
};


template <class MPTraits>
ClearanceTestStrategy<MPTraits>::ClearanceTestStrategy(const map<string, pair<int, int> >& _samplerLabels, const vector<ClearanceUtility<MPTraits> >& _utilities, string _dmLabel) : MPStrategyMethod<MPTraits>(),
  m_samplerLabels(_samplerLabels), m_clearanceUtilities(_utilities), m_dmLabel(_dmLabel)
  {
    this->SetName("ClearanceTest");
  }

template <class MPTraits>
ClearanceTestStrategy<MPTraits>::
ClearanceTestStrategy(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node)
  : MPStrategyMethod<MPTraits>(_problem, _node) {
    this->SetName("ClearanceTest");
    ParseXML(_node);
  }

template <class MPTraits>
ClearanceTestStrategy<MPTraits>::
~ClearanceTestStrategy()
{}

template <class MPTraits>
void
ClearanceTestStrategy<MPTraits>::
ParseXML(XMLNodeReader& _node)
{
  m_dmLabel = _node.stringXMLParameter("dmLabel", true, "", "distance metric label");
  _node.warnUnrequestedAttributes();

  for(XMLNodeReader::childiterator I = _node.children_begin(); I != _node.children_end(); ++I) {
    if(I->getName() == "node_generation_method") {
      string generationMethod = I->stringXMLParameter("Method", true, "", "node generation method");
      int numberSamples = I->numberXMLParameter("Number", false, 1, 0, MAX_INT, "number of samples");
      int numberAttempts = I->numberXMLParameter("Attempts", false, 100, 1, MAX_INT, "number of attempts");
      m_samplerLabels[generationMethod] = make_pair(numberSamples, numberAttempts);
      I->warnUnrequestedAttributes();
    } else if(I->getName() == "clearance") {
      m_clearanceUtilities.push_back(ClearanceUtility<MPTraits>(this->GetMPProblem(), *I));
    } else
      I->warnUnknownNode();
  }
}

template <class MPTraits>
void
ClearanceTestStrategy<MPTraits>::PrintOptions(ostream& _out) const {
  _out << "ClearanceTestStrategy ::"
       << "\tdmLabel = \"" << m_dmLabel << "\""
       << "\n\tSamplers\n";
  for(map<string, pair<int,int> >::const_iterator I = m_samplerLabels.begin(); I != m_samplerLabels.end(); ++I)
    _out << "\t\t\"" << I->first << "\", " << I->second.first << " samples, " << I->second.second << " attempts\n";
  _out << "\tClearance Measurements\n";
  for(typename vector<ClearanceUtility<MPTraits> >::const_iterator I = m_clearanceUtilities.begin(); I != m_clearanceUtilities.end(); ++I) {
    I->Print(_out);
    _out << endl;
  }
  _out << endl;
}

template <class MPTraits>
void
ClearanceTestStrategy<MPTraits>::
Run()
{
  PrintOptions(cout);

  if(m_clearanceUtilities.size() < 2) {
    cerr << "\n\nERROR in ClearanceTestStrategy::Run(), you must specify at least 2 clearance measurement settings, the first one as the baseline and the remaining to compare to it, exiting.\n";
    exit(-1);
  }

  ClockClass clock;
  StatClass *stats = this->GetMPProblem()->GetStatClass();
  stats->StartClock("Clearance Test");

  //generate samples
  vector<CfgType> samples;
  for(auto sampler : m_samplerLabels) {
    typename MPTraits::MPProblemType::SamplerPointer s = this->GetSampler(sampler.first);
    s->Sample(sampler.second.first, sampler.second.second,
        this->m_boundary, back_inserter(samples));
  }
  if(this->m_debug)
    cout << "\tgenerated " << samples.size() << " samples total" << endl;

  //for each sample, compare baseline clearance and input clearance
  for(auto sample : samples) {
    if(this->m_debug) cout << "computing clearances for cfg: " << sample << endl;

    //compute baseline clearance
    double baselineClearance = MAX_INT;
    CfgType baselineWitness;
    CDInfo baselineInfo;
    if(m_clearanceUtilities.front().CollisionInfo(sample, baselineWitness, this->m_boundary, baselineInfo)) {
      baselineClearance = baselineInfo.m_minDist;
    } else {
      cerr << "Error, could not compute baseline collision information for cfg: " << sample << endl;
    }
    if(this->m_debug) cout << "\tbaseline = " << baselineClearance << "\twitness = " << baselineWitness << endl;
    sample.m_witnessCfg = shared_ptr<Cfg>(); //reset clearance calculation cache

    for(typename vector<ClearanceUtility<MPTraits> >::iterator C = m_clearanceUtilities.begin()+1; C != m_clearanceUtilities.end(); ++C) {
      //compute input clearance
      double approxClearance = MAX_INT;
      CfgType approxWitness;
      CDInfo approxInfo;
      if(C->CollisionInfo(sample, approxWitness, this->m_boundary, approxInfo)) {
        approxClearance = approxInfo.m_minDist;
      } else {
        cerr << "Error, could not compute approx collision information for cfg: " << sample << endl;
      }
      if(this->m_debug) cout << "\tapprox = " << approxClearance << "\twitness = " << approxWitness << endl;
      sample.m_witnessCfg = shared_ptr<Cfg>(); //reset clearance calculation cache

      //calculate difference
      if(this->m_debug) cout << "\tdifference = " << approxClearance - baselineClearance << "\tdistance = " << this->GetMPProblem()->GetDistanceMetric(m_dmLabel)->Distance(approxWitness, baselineWitness) << endl;
    }
  }

  stats->StopClock("Clearance Test");
  cout << ":" << stats->GetSeconds("Clearance Test") << " sec (ie, " << stats->GetUSeconds("Clearance Test") << " usec)";
  cout << endl;
}

#endif
