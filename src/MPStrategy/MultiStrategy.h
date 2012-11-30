
class MPMSContainer : public MPSMContainer {
public:
  MPMSContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
  vector< string > m_strategy_methods; 
  MPSMContainer parent;

};

class MPMultiStrategy : public MPStrategyMethod { 
  public: 
    MPMultiStrategy(MPMSContainer cont) : MPStrategyMethod(cont.parent) {
      m_strategy_methods = cont.m_strategy_methods;
    }
    MPMultiStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem);
    virtual ~MPMultiStrategy() {}

    virtual void PrintOptions(ostream& out_os);  
    virtual void ParseXML(XMLNodeReader& in_Node);

    virtual void operator()();
    virtual void Initialize(){}
    virtual void Run(){}
    virtual void Finalize(){}


  private:
    vector< string > m_strategy_methods;
};

