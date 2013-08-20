MPMultiStrategy::
MPMultiStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPStrategyMethod(in_Node,in_pProblem) {
    ParseXML(in_Node);
  }

void
MPMultiStrategy::
PrintOptions(ostream& out_os) const {
}

void
MPMultiStrategy::
ParseXML(XMLNodeReader& in_Node) {
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if (citr->getName() == "strategy") {
      string strategy = citr->stringXMLParameter("Method",true,"","Method");
      m_strategy_methods.push_back(strategy);
    } else {
      citr->warnUnknownNode();
    }
  }
}

void
MPMultiStrategy::
operator()() {
  // initializing from input
  typedef vector< string >::iterator VITRTR;
  for (VITRTR s_itrtr = m_strategy_methods.begin(); s_itrtr < m_strategy_methods.end(); s_itrtr++) {
    MPStrategyMethod* strategy = GetMPProblem()->GetMPStrategy()->
      GetMPStrategyMethod(*s_itrtr);
    (*strategy)();
  }
}


