
class MPCContainer : public MPSMContainer {
public:
  MPCContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
   vector<string> m_input_methods;
   vector<string> m_comparer_methods;
   MPSMContainer parent;

};

class MPComparer : public MPStrategyMethod {
  public: 
    MPComparer(MPCContainer cont) : MPStrategyMethod(cont.parent) {
      m_input_methods = cont.m_input_methods;
      m_comparer_methods = cont.m_comparer_methods;

    }
    MPComparer(XMLNodeReader& in_Node, MPProblem* in_pProblem);
    virtual ~MPComparer() {}

    virtual void PrintOptions(ostream& out_os);  
    virtual void ParseXML(XMLNodeReader& in_Node);

    virtual void operator()();

    virtual void Initialize(){}
    virtual void Run(){}
    virtual void Finalize(){}

  private:
    vector<string> m_input_methods;
    vector<string> m_comparer_methods;
};

